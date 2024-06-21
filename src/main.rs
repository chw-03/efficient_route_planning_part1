#[allow(unused)]
mod graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::routing::*;
    use core::num;
    use osmpbfreader::objects::OsmObj;
    use std::{collections::HashMap, ops::Index};

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord)]
    pub struct Node {
        //nodes from OSM, each with unique ID and coordinate position
        pub id: i64,
        pub lat: i64,
        pub lon: i64,
    }

    #[derive(Debug, PartialEq, Hash, Eq, Clone)]
    pub struct Way {
        //ways from OSM, each with unique ID, speed from highway type, and referenced nodes that it connects
        pub id: i64,
        pub speed: u64,
        pub refs: Vec<i64>,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct RoadNetwork {
        //graph struct that will be used to route
        pub nodes: HashMap<i64, Node>, // <node.id, node>
        pub edges: HashMap<i64, HashMap<i64, (u64, bool)>>, // tail.id, <head.id, (cost, arcflag)>
        pub raw_ways: Vec<Way>,
    }

    fn speed_calc(highway: &str) -> Option<u64> {
        //calculates speed of highway based on given values
        match highway {
            "motorway" => Some(110),
            "trunk" => Some(110),
            "primary" => Some(70),
            "secondary" => Some(60),
            "tertiary" => Some(50),
            "motorway_link" => Some(50),
            "trunk_link" => Some(50),
            "primary_link" => Some(50),
            "secondary_link" => Some(50),
            "road" => Some(40),
            "unclassified" => Some(40),
            "residential" => Some(30),
            "unsurfaced" => Some(30),
            "living_street" => Some(10),
            "service" => Some(5),
            _ => None,
        }
    }

    impl RoadNetwork {
        pub fn new(mut nodes: HashMap<i64, Node>, ways: Vec<Way>) -> Self {
            //init new RoadNetwork based on results from reading .pbf file
            let mut edges: HashMap<i64, HashMap<i64, (u64, bool)>> = HashMap::new();
            //println!("# of ways {}", ways.len());
            for way in ways.clone() {
                let mut previous_head_node_now_tail: Option<&Node> = None;
                let mut previous_head_node_index: usize = 0;
                for i in 0..way.refs.len() - 1 {
                    let tail_id = way.refs[i];
                    let tail: Option<&Node> = match previous_head_node_now_tail {
                        Some(previous_head_node_now_tail) => match previous_head_node_index == i {
                            true => Some(previous_head_node_now_tail),
                            false => nodes.get(&tail_id),
                        },
                        None => nodes.get(&tail_id),
                    };

                    let head_id = way.refs[i + 1];
                    let head = nodes.get(&head_id);
                    if let (Some(tail), Some(head)) = (tail, head) {
                        //following math converts lon/lat into distance of segment
                        let a = i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64
                            / f64::powi(10.0, 14);
                        let b = i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64
                            / f64::powi(10.0, 14);
                        let c = (a + b).sqrt();
                        let cost = (c as u64) / ((way.speed as f64) * 5.0 / 18.0) as u64; //seconds needed to traverse segment based on road type
                        let flag = false;
                        edges
                            .entry(tail_id)
                            .and_modify(|inner| {
                                inner.insert(head_id, (cost, flag));
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(head_id, (cost, flag));
                                a
                            });
                        edges
                            .entry(head.id)
                            .and_modify(|inner| {
                                inner.insert(tail_id, (cost, flag));
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(tail_id, (cost, flag));
                                a
                            });
                        previous_head_node_now_tail = Some(head);
                        previous_head_node_index = i + 1;
                    }
                }
            }
            let node_to_remove = nodes
                .iter()
                .filter(|(node, _)| !edges.contains_key(node))
                .map(|(x, _)| *x)
                .collect::<Vec<i64>>();
            for node in &node_to_remove {
                nodes.remove(node);
            }

            Self {
                nodes,
                raw_ways: ways,
                edges,
            }
        }

        pub fn read_from_osm_file(path: &str) -> Option<(HashMap<i64, Node>, Vec<Way>)> {
            //reads osm.pbf file, values are used to make RoadNetwork
            let mut nodes = HashMap::new();
            let mut ways = Vec::new();
            let path_cleaned = std::path::Path::new(&path);
            let r = std::fs::File::open(path_cleaned).unwrap();
            let mut reader = osmpbfreader::OsmPbfReader::new(r);
            for obj in reader.iter().map(Result::unwrap) {
                match obj {
                    OsmObj::Node(e) => {
                        nodes.insert(
                            e.id.0,
                            Node {
                                id: e.id.0,
                                lat: (e.lat() * f64::powi(10.0, 7)) as i64,
                                lon: (e.lon() * f64::powi(10.0, 7)) as i64,
                            },
                        );
                    }
                    OsmObj::Way(e) => {
                        if let Some(road_type) =
                            e.tags.clone().iter().find(|(k, _)| k.eq(&"highway"))
                        {
                            if let Some(speed) = speed_calc(road_type.1.as_str()) {
                                ways.push(Way {
                                    id: e.id.0,
                                    speed,
                                    refs: e.nodes.into_iter().map(|x| x.0).collect(),
                                });
                            }
                        }
                    }
                    _ => {}
                }
            }
            Some((nodes, ways))
        }

        pub fn reduce_to_largest_connected_component(self) -> Self {
            //reduces graph to largest connected component through nodes visited with dijkstra
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<i64, i32> = HashMap::new();
            let mut shortest_path_graph = Dijkstra::new(&self);
            let mut max_connections = 0;

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter += 1;
                let mut shortest_path_graph = Dijkstra::new(&self);
                shortest_path_graph.dijkstra(source_id, -1, &None, false);
                for node in shortest_path_graph.visited_nodes.keys() {
                    number_times_node_visted.insert(*node, counter);
                }
                if number_times_node_visted.len() > (self.nodes.len() / 2) {
                    break;
                }
            }
            let mut new_node_list = Vec::new();
            new_node_list = number_times_node_visted.iter().collect();
            new_node_list.sort_by(|(node1, counter1), (node2, counter2)| counter1.cmp(counter2));

            let connected_components = &mut new_node_list
                .chunk_by(|(node1, counter1), (node2, counter2)| counter1 == counter2);

            let mut largest_node_set = Vec::new();
            let mut prev_set_size = 0;

            while let Some(node_set) = connected_components.next() {
                if node_set.len() > prev_set_size {
                    largest_node_set = node_set.to_vec();
                    prev_set_size = node_set.len();
                }
            }

            let lcc_nodes = largest_node_set
                .iter()
                .map(|(id, _)| (**id, *self.nodes.get(id).unwrap()))
                .collect::<HashMap<i64, Node>>();

            RoadNetwork::new(lcc_nodes, self.raw_ways)
        }
    }
}

#[allow(unused)]
mod routing {
    //routing algorithms and helper functiions
    use crate::graph_construction::*;
    use crate::landmark_algo::*;
    use rand::Rng;
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::rc::Rc;
    use std::time::Instant;
    use std::u64::MAX;

    pub struct Dijkstra {
        //handle dijkstra calculations
        pub graph: RoadNetwork,
        pub visited_nodes: HashMap<i64, u64>,
        cost_upper_bound: u64,
        max_settled_nodes: u64,
    }

    #[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord)]
    pub struct PathedNode {
        //node that references parent nodes, used to create path from goal node to start node
        pub node_self: Node,
        pub distance_from_start: u64,
        pub parent_node: Option<Rc<PathedNode>>,
    }

    impl PathedNode {
        pub fn extract_parent<PathedNode: std::clone::Clone>(
            //returns parent of a pathed node
            last_elem: Rc<PathedNode>,
        ) -> PathedNode {
            let inner: PathedNode = Rc::unwrap_or_clone(last_elem);
            inner
        }
        pub fn get_path(self) -> (Vec<Node>, u64) {
            //uses reference to find the source node with parent_node == None
            let mut shortest_path: Vec<Node> = Vec::new();
            let mut total_distance: u64 = self.distance_from_start;
            let mut current = self;
            while let Some(previous_node) = current.parent_node {
                shortest_path.push(current.node_self);
                current = PathedNode::extract_parent(previous_node);
            }
            shortest_path.push(current.node_self);
            (shortest_path, total_distance)
        }
    }

    pub fn a_star_heuristic(graph: &RoadNetwork, target: i64) -> HashMap<i64, u64> {
        let tail = *graph.nodes.get(&target).unwrap();
        //for each current i64 id, enter euciladan distance from current to target, divided by max speed on that path
        let heuristics = graph
            .nodes
            .iter()
            .map(|(id, head)| {
                (
                    *id,
                    ((i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64
                        / f64::powi(10.0, 14)
                        + i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64
                            / f64::powi(10.0, 14))
                    .sqrt() as u64)
                        / ((110_f64) * 5.0 / 18.0) as u64, //110 is motorway speed --> max speed possible on road network
                )
            })
            .collect::<HashMap<i64, u64>>();
        heuristics
    }

    impl Dijkstra {
        //implementation of dijkstra's shortest path algorithm
        pub fn new(graph: &RoadNetwork) -> Self {
            let visited_nodes = HashMap::new();
            Self {
                graph: graph.clone(),
                visited_nodes,
                cost_upper_bound: MAX,
                max_settled_nodes: MAX,
            }
        }

        pub fn set_cost_upper_bound(&mut self, upper_bound: u64) {
            self.cost_upper_bound = upper_bound;
        }

        pub fn set_max_settled_nodes(&mut self, max_settled: u64) {
            self.max_settled_nodes = max_settled;
        }

        pub fn get_neighbors(
            &mut self,
            current: &PathedNode,
            consider_arc_flags: bool,
        ) -> Vec<(Node, u64)> {
            //return node id of neighbors
            let mut paths = Vec::new();
            let mut next_node_edges = HashMap::new();
            //need some case to handle neighbor to parent instead of just parent to neighbor
            if let Some(connections) = self.graph.edges.get_mut(&current.node_self.id) {
                next_node_edges.clone_from(connections);
            }
            for path in next_node_edges {
                if self.visited_nodes.contains_key(&path.0) {
                    continue;
                }
                if (consider_arc_flags && !path.1 .1) {
                    continue;
                }
                paths.push((*self.graph.nodes.get(&path.0).unwrap(), path.1 .0));
            }
            paths
        }

        pub fn dijkstra(
            &mut self,
            source_id: i64,
            target_id: i64,
            heuristics: &Option<HashMap<i64, u64>>,
            consider_arc_flags: bool,
        ) -> (Option<PathedNode>, HashMap<i64, i64>) {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
            let mut previous_nodes = HashMap::new();

            //set target (-1) for all-node-settle rather than just target settle or smth
            self.visited_nodes.clear();

            let source = *self
                .graph
                .nodes
                .get(&source_id)
                .unwrap_or_else(|| panic!("source node not found"));

            let source_node: PathedNode = PathedNode {
                node_self: (source),
                distance_from_start: 0,
                parent_node: (None),
            };

            let mut gscore: HashMap<i64, u64> = HashMap::new();
            gscore.insert(source_id, 0);

            priority_queue.push(Reverse((0, source_node.clone())));

            let mut target: Node = Node {
                id: 0,
                lon: 0,
                lat: 0,
            };
            if target_id > 0 {
                target = *self
                    .graph
                    .nodes
                    .get(&target_id)
                    .unwrap_or_else(|| panic!("target node not found"));
            }

            let mut counter = 1;
            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                let cost = pathed_current_node.distance_from_start;
                let idx = pathed_current_node.node_self.id;
                //something to check if node was already done stuff to and skip this whole thing if yes
                self.visited_nodes.insert(idx, cost);

                if cost > *gscore.get(&idx).unwrap_or(&MAX) {
                    continue;
                }

                if cost > self.cost_upper_bound
                    || self.visited_nodes.len() > self.max_settled_nodes as usize
                {
                    return (None, previous_nodes);
                }

                if idx.eq(&target_id) {
                    return (Some(pathed_current_node), previous_nodes);
                }

                for neighbor in self.get_neighbors(&pathed_current_node, consider_arc_flags) {
                    let temp_distance = pathed_current_node.distance_from_start + neighbor.1;
                    let next_distance = *gscore.get(&neighbor.0.id).unwrap_or(&MAX);
                    if temp_distance < next_distance {
                        gscore.insert(neighbor.0.id, temp_distance);
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
                            node_self: neighbor.0,
                            distance_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };
                        let h;
                        if let Some(heuristic) = heuristics {
                            h = heuristic.get(&neighbor.0.id).unwrap_or(&0);
                        } else {
                            h = &0;
                        }
                        priority_queue.push(Reverse((temp_distance + h, tentative_new_node)));
                        previous_nodes.insert(neighbor.0.id, pathed_current_node.node_self.id);
                    }
                }
                counter += 1;
            }
            if consider_arc_flags {
                //print!(" f{} ", counter);
            }
            (None, previous_nodes)
        }

        pub fn get_random_node_id(&mut self) -> Option<i64> {
            //returns ID of a random valid node from a graph
            let mut rng = rand::thread_rng();
            let mut full_node_list = Vec::new();
            for node in &self.graph.nodes {
                full_node_list.push(node.0);
            }
            let mut random: usize = rng.gen_range(0..full_node_list.len());
            let mut node_id = full_node_list.get(random).unwrap();

            Some(**node_id)
        }

        pub fn get_random_node_area_id(
            &mut self,
            lat_min: f32,
            lat_max: f32,
            lon_min: f32,
            lon_max: f32,
        ) -> i64 {
            let lat_range =
                (lat_min * f32::powi(10.0, 7)) as i64..(lat_max * f32::powi(10.0, 7)) as i64;
            let lon_range =
                (lon_min * f32::powi(10.0, 7)) as i64..(lon_max * f32::powi(10.0, 7)) as i64;
            let mut found = false;
            let mut id = -1;
            while (!found) {
                if let Some(node_id) = self.get_random_node_id() {
                    if let Some(node) = self.graph.nodes.get(&node_id) {
                        found = lat_range.contains(&node.lat) && lon_range.contains(&node.lon);
                        id = node_id
                    }
                }
            }
            id
        }

        pub fn get_unvisted_node_id(
            //returns the first unvisted node that function parses upon (used to find largest connected component)
            &mut self,
            other_located_nodes: &HashMap<i64, i32>,
        ) -> Option<i64> {
            if other_located_nodes.len() == self.graph.nodes.len() {
                println!("all nodes visted");
                return None;
            }
            let other_located_nodes = other_located_nodes
                .iter()
                .filter(|(id, count)| **count > 0)
                .map(|(id, _)| id)
                .collect::<Vec<&i64>>();

            for node in &self.graph.nodes {
                if !other_located_nodes.contains(&node.0) {
                    return Some(*node.0);
                }
            }
            None
        }
    }
}

#[allow(dead_code)]
mod landmark_algo {
    use crate::routing::*;
    use std::collections::HashMap;

    pub fn landmark_heuristic_precompute(
        dijkstra_graph: &mut Dijkstra,
        num_landmarks: usize,
    ) -> HashMap<i64, HashMap<i64, u64>> {
        let roads = dijkstra_graph.graph.clone();
        let mut landmarks = Vec::new();
        for _ in 0..num_landmarks {
            landmarks.push(dijkstra_graph.get_random_node_id().unwrap());
        }
        let mut graph = Dijkstra::new(&roads);
        landmarks
            .iter()
            .map(|&l| {
                (l, {
                    graph.dijkstra(l, -1, &None, false);
                    graph
                        .visited_nodes
                        .iter()
                        .map(|(id, dist)| (*id, *dist))
                        .collect()
                })
            })
            .collect::<HashMap<i64, HashMap<i64, u64>>>() //landmark_id, node_id, distance
    }

    pub fn landmark_heuristic(
        landmark_precompute: &HashMap<i64, HashMap<i64, u64>>,
        dijkstra_graph: &Dijkstra,
        target: i64,
    ) -> HashMap<i64, u64> {
        dijkstra_graph
            .graph
            .nodes
            .iter()
            .map(|(source, _)| {
                (*source, {
                    landmark_precompute
                        .iter()
                        .map(|(_, arr)| {
                            let dist_lu = *arr.get(source).unwrap();
                            let dist_tu = *arr.get(&target).unwrap();
                            dist_lu.abs_diff(dist_tu)
                        })
                        .max()
                        .unwrap()
                })
            })
            .collect()
    }
}

mod arc_flags_algo {
    use crate::routing::*;
    use core::ops::Range;
    use std::collections::HashSet;
    pub struct ArcFlags {
        //precomputation stuff for arc flag routing algorithm
        pub lat_range: Range<i64>,
        pub lon_range: Range<i64>,
    }
    #[allow(dead_code)]
    impl ArcFlags {
        pub fn new(lat_min: f32, lat_max: f32, lon_min: f32, lon_max: f32) -> ArcFlags {
            ArcFlags {
                lat_range: (lat_min * f32::powi(10.0, 7)) as i64
                    ..(lat_max * f32::powi(10.0, 7)) as i64,
                lon_range: (lon_min * f32::powi(10.0, 7)) as i64
                    ..(lon_max * f32::powi(10.0, 7)) as i64,
            }
        }

        pub fn arc_flags_precompute(self, dijkstra_graph: &mut Dijkstra) {
            let mut boundary_node = HashSet::new();
            let region_nodes = dijkstra_graph
                .graph
                .nodes
                .iter()
                .filter(|(_, &node)| {
                    self.lat_range.contains(&node.lat) && self.lon_range.contains(&node.lon)
                })
                .map(|(id, _)| *id)
                .collect::<Vec<i64>>();

            for node in region_nodes.clone() {
                if let Some(edge_list) = dijkstra_graph.graph.edges.get_mut(&node) {
                    for edge in edge_list.iter_mut() {
                        if region_nodes.contains(edge.0) {
                            edge.1 .1 = true;
                            continue;
                        }
                        boundary_node.insert(node);
                    }
                }
            }

            println!("boundary nodes: {}", boundary_node.len());

            for node in boundary_node {
                let (_, edges) = dijkstra_graph.dijkstra(node, -1, &None, false);
                for (head, tail) in edges {
                    if let Some(edgelist) = dijkstra_graph.graph.edges.get_mut(&head) {
                        for (&id, (_, arcflag)) in edgelist {
                            if id == tail {
                                *arcflag = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}

#[allow(dead_code)]
mod contraction_hierarchies {
    use std::collections::HashMap;

    use crate::routing::*;

    pub struct ContractedGraph {
        pub ordered_nodes_to_contract: Vec<i64>,
        costs_of_uv: HashMap<i64, u64>, //incoming arcs to v
        costs_of_vw: HashMap<i64, u64> //outgoing arcs from v

    }

    impl ContractedGraph {
        pub fn new() -> ContractedGraph {
            ContractedGraph {
                ordered_nodes_to_contract: Vec::new(),
                costs_of_uv: HashMap::new(), 
                costs_of_vw: HashMap::new()
            }
        }

        pub fn compute_random_node_ordering(&mut self, graph: &mut Dijkstra, length: usize) {
            self.ordered_nodes_to_contract.push(0);
            while self.ordered_nodes_to_contract.len() < length + 1 {
                self.ordered_nodes_to_contract
                    .push(graph.get_random_node_id().unwrap_or_default());
            }
            for (_, edgelist) in graph.graph.edges.iter_mut() {
                for edge in edgelist.iter_mut() {
                    edge.1 .1 = true;
                }
            }
        }

        pub fn contract_node(&mut self, nth_order: usize, graph: &mut Dijkstra) {
            let nth_node = self.ordered_nodes_to_contract[nth_order + 1];
            /*let edgelist = graph.graph.edges.get_mut(&nth_node).unwrap();
            for edge in edgelist.iter_mut() {
                edge.1 .1 = false;
            } */
            for (u, u_edgelist) in graph.graph.edges.iter_mut() {
                if *u == nth_node {
                    //the current node is the removed-node V
                    for (w, (cost, flag)) in u_edgelist {
                        self.costs_of_vw.insert(*w, *cost);
                        *flag = false;
                    }
                } else {
                    for (v, (cost, flag)) in u_edgelist {
                        if *v == nth_node {
                            self.costs_of_uv.insert(*v, *cost);
                            *flag = false;
                        }
                    }
                }
            }
        }

        pub fn generate_shortcuts(&self, nth_order: usize, graph: &mut Dijkstra) -> (u8, i8) {
            //(#shortcuts, #shortcuts - arcs removed)
            let mut num_shortcuts: u8 = 0;
            let mut edge_diff: i8 = 0;
            let v = self.ordered_nodes_to_contract[nth_order + 1];
            
            
            
            (num_shortcuts, (edge_diff + num_shortcuts as i8)/2)
        }
    }
}

fn main() {}

#[cfg(test)]
mod tests {
    //use crate::arc_flags_algo::ArcFlags;
    use crate::graph_construction::*;
    //use crate::landmark_algo::*;
    use crate::routing::*;
    //use std::collections::HashMap;
    use crate::contraction_hierarchies::*;
    use std::time::Instant;

    #[test]
    fn run_algo() {
        //let path = "bw.pbf";
        //let path = "uci.pbf";
        let path = "saarland.pbf";
        let data = RoadNetwork::read_from_osm_file(path).unwrap();
        let mut roads = RoadNetwork::new(data.0, data.1);
        println!(
            "{} Base Graph Nodes: {}, Edges: {}",
            path,
            roads.nodes.len(),
            roads.edges.len()
        );
        let now = Instant::now();
        roads = roads.reduce_to_largest_connected_component();
        let mut time = now.elapsed().as_millis() as f32 * 0.001;
        println!(
            "time: {}, reduced map nodes: {}, edges: {}",
            time,
            roads.nodes.len(),
            roads
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
                / 2
        );

        let mut contraction_time = Vec::new();
        let mut shortcut_hg = vec![0, 0, 0, 0, 0];
        let mut edge_diff_hg = vec![0, 0, 0, 0, 0];
        let mut routing_graph = Dijkstra::new(&roads);
        let mut ch_algo = ContractedGraph::new();
        ch_algo.compute_random_node_ordering(&mut routing_graph, 1000); //here
        for n in 0..ch_algo.ordered_nodes_to_contract.len()-1 {
            ch_algo.contract_node(n, &mut routing_graph);
            let (num_shortcut, num_edge_diff) =
                ch_algo.generate_shortcuts(n, &mut routing_graph);
            time = now.elapsed().as_millis() as f32 * 0.001;
            contraction_time.push(time);
            if num_shortcut >= 4 {
                shortcut_hg[4] += 1;
            } else if num_shortcut == 3 {
                shortcut_hg[3] += 1;
            } else if num_shortcut == 2 {
                shortcut_hg[2] += 1;
            } else if num_shortcut == 1 {
                shortcut_hg[1] += 1;
            } else if num_shortcut == 0{
                shortcut_hg[0] += 1;
            }

            if num_edge_diff <= -3 {
                edge_diff_hg[0] += 1;
            } else if num_edge_diff == -2 {
                edge_diff_hg[1] += 1;
            } else if num_edge_diff == 2 {
                edge_diff_hg[3] += 1;
            } else if num_edge_diff >= 3 {
                edge_diff_hg[4] += 1;
            } else {
                edge_diff_hg[2] += 1;
            }
        }

        println!(
            "average contraction time in seconds {}",
            contraction_time.iter().sum::<f32>() / contraction_time.len() as f32
        );

        println!(
            "shortcut histogram {:?}",
            shortcut_hg
        );

        println!(
            "edge difference histogram {:?}",
            edge_diff_hg
        );

        /*
        let mut shortest_path_costs = Vec::new();
        let mut query_time = Vec::new();
        let mut settled_nodes = Vec::new();
        let heuristics = None;

        //let precompute = landmark_heuristic_precompute(&mut routing_graph, 42);
        let arc_flag_thing = ArcFlags::new(49.20, 49.25, 6.95, 7.05); //saar
        //let arc_flag_thing = ArcFlags::new(47.95, 48.05, 7.75, 7.90); //ba-wu
        //let arc_flag_thing = ArcFlags::new(33.63, 33.64, -117.84, -117.83); //uci
        arc_flag_thing.arc_flags_precompute(&mut routing_graph);
        time = now.elapsed().as_millis() as f32 * 0.001;
        println!("pre done {} \n", time);


        for _ in 0..100 {
            let source = routing_graph.get_random_node_id().unwrap();
            //let target = routing_graph.get_random_node_id().unwrap();
            let target = routing_graph.get_random_node_area_id(49.20, 49.25, 6.95, 7.05); //saar
            //let target = routing_graph.get_random_node_area_id(47.95, 48.05, 7.75, 7.90); //ba-wu
            //let target = routing_graph.get_random_node_area_id(33.63, 33.64, -117.84, -117.83); //uci
            //heuristics = a_star_heuristic(&roads, target);
            //heuristics = landmark_heuristic(&precompute, &routing_graph, target);
            let now = Instant::now();
            let result = routing_graph.dijkstra(source, target, &heuristics, true);
            time = now.elapsed().as_millis() as f32 * 0.001;
            query_time.push(time);

            if let Some(cost) = result.0 {
                shortest_path_costs.push(cost.get_path().1);
            } else {
                shortest_path_costs.push(0);
            }
            settled_nodes.push(routing_graph.visited_nodes.len() as u64);
        }

        println!(
            "average cost in minutes {}",
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 60
        );
        println!(
            "average query time in seconds {}",
            query_time.iter().sum::<f32>() / query_time.len() as f32
        );
        println!(
            "average settle node number {}",
            settled_nodes.iter().sum::<u64>() / settled_nodes.len() as u64
        );*/
    }

    /*
    #[test]
    fn test() {
        let node0 = Node {
            id: 0,
            lat: 490000000,
            lon: 65000000,
        };
        let node1 = Node {
            id: 1,
            lat: 491000000,
            lon: 65100000,
        };
        let node2 = Node {
            id: 2,
            lat: 495000000,
            lon: 70000000,
        };
        let node3 = Node {
            id: 3,
            lat: 493500000,
            lon: 71250000,
        };
        let node4 = Node {
            id: 4,
            lat: 492500000,
            lon: 72500000,
        };
        let node5 = Node {
            id: 5,
            lat: 497500000,
            lon: 72500000,
        };
        let roads = RoadNetwork {
            nodes: HashMap::from([ (0, node0), (1, node1), (2, node2), (3, node3), (4, node4),  (5, node5)]),
            edges: HashMap::from([
                (0, HashMap::from([(1, (5, false))])),
                (1, HashMap::from([(0, (5, false)), (2, (5, false))])),
                (2, HashMap::from([(1, (5, false)), (3, (5, false)), (4, (5, false))])),
                (3, HashMap::from([(2, (5, false)), (4, (5, false))])),
                (4, HashMap::from([(3, (5, false)), (5, (5, false)), (2, (5, false))])),
                (5, HashMap::from([(4, (5, false))])),
            ]),
            raw_ways: vec![Way{id:0,speed:0, refs:vec![0,0]}]
        };
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
        let heuristics = HashMap::new();
        let mut graph = Dijkstra::new(&roads);
        let arc_flag_thing = ArcFlags::new(49.0, 49.2, 6.5, 6.52);
        arc_flag_thing.arc_flags_precompute(&mut graph);
        println!("roadnet {:?}", graph.graph.edges);
        let source = 5;
        let target = 0;
        println!(
            "\ndijiktra path and cost {:?}",
            graph.dijkstra(source, target, &heuristics, true)
        );
    }*/
}
