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
        pub nodes: HashMap<i64, Node>,              // <node.id, node>
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
                        previous_head_node_now_tail = Some(&head);
                        previous_head_node_index = i + 1;
                    }
                }
            }
            let node_to_remove = nodes
                .iter()
                .filter(|(node, _)| !edges.contains_key(node))
                .map(|(x, _)| x.clone())
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
            let r = std::fs::File::open(&path_cleaned).unwrap();
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
            let heuristics = HashMap::new();

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter = counter + 1;
                let mut shortest_path_graph = Dijkstra::new(&self);
                shortest_path_graph.dijkstra(source_id, -1, &heuristics, false);
                for (node, _) in &shortest_path_graph.visited_nodes {
                    number_times_node_visted.insert(*node, counter);
                }
                if number_times_node_visted.len() > (self.nodes.len() / 2) {
                    break;
                }
            }
            let mut new_node_list = Vec::new();
            new_node_list = number_times_node_visted
                .iter()
                .map(|(node, counter)| (node, counter))
                .collect();
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
    use std::u64::MAX;

    pub struct Dijkstra {
        //handle dijkstra calculations
        pub graph: RoadNetwork,
        pub visited_nodes: HashMap<i64, u64>,
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
                        / ((110 as f64) * 5.0 / 18.0) as u64, //110 is motorway speed --> max speed possible on road network
                )
            })
            .collect::<HashMap<i64, u64>>();
        //println!("to {} is distance est {:?}", target, heuristics.clone().into_values());
        heuristics
    }

    impl Dijkstra {
        //implementation of dijkstra's shortest path algorithm
        pub fn new(graph: &RoadNetwork) -> Self {
            let visited_nodes = HashMap::new();
            Self {
                graph: graph.clone(),
                visited_nodes,
            }
        }


        //to-do: block off arcs that have false arc flag
        pub fn get_neighbors(&mut self, current: &PathedNode, consider_arc_flags: bool) -> Vec<(PathedNode, u64)> {
            //return node id of neighbors
            let mut paths = Vec::new();
            let mut next_node_edges = HashMap::new();
            if let Some(connections) = self.graph.edges.get(&current.node_self.id) {
                next_node_edges = connections.clone();
            }
            let current: Rc<PathedNode> = Rc::new(current.clone());
            for path in next_node_edges {
                let node_self: Node = *self.graph.nodes.get(&path.0).unwrap();
                paths.push((
                    PathedNode {
                        node_self: node_self,
                        distance_from_start: MAX,
                        parent_node: Some(current.clone()),
                    },
                    path.1.0,
                ));
            }
            paths
        }

        pub fn get_path(&mut self, target: PathedNode) -> (Vec<Node>, u64) {
            //uses reference to find the source node with parent_node == None
            let mut shortest_path: Vec<Node> = Vec::new();
            let mut total_distance: u64 = target.distance_from_start;
            let mut current = target;
            while let Some(previous_node) = current.parent_node {
                shortest_path.push(current.node_self);
                current = PathedNode::extract_parent(previous_node);
            }
            shortest_path.push(current.node_self);
            (shortest_path, total_distance)
        }

        pub fn dijkstra(
            &mut self,
            source_id: i64,
            target_id: i64,
            heuristics: &HashMap<i64, u64>,
            consider_arc_flags: bool
        ) -> Option<(Vec<Node>, u64)> {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
            //set target (-1) for all-node-settle rather than just target settle or smth

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
                    .unwrap_or_else(|| panic!("source node not found"));
            }

            let mut counter = 1;
            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                if let Some(_) = (self.visited_nodes.insert(
                    pathed_current_node.node_self.id,
                    pathed_current_node.distance_from_start,
                )) {
                    continue;
                }

                if pathed_current_node.node_self.id.eq(&target_id) {
                    return Some(self.get_path(pathed_current_node));
                }

                for neighbor_node in self.get_neighbors(&pathed_current_node, consider_arc_flags) {
                    // something to check if node was already done stuff to and skip this whole thing if yes
                    let temp_distance = pathed_current_node.distance_from_start + neighbor_node.1;
                    let next_distance = neighbor_node.0.distance_from_start;
                    if temp_distance < next_distance {
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
                            node_self: neighbor_node.0.node_self,
                            distance_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };
                        priority_queue.push(Reverse((
                            temp_distance
                                + *heuristics.get(&neighbor_node.0.node_self.id).unwrap_or(&0),
                            tentative_new_node,
                        )));
                    }
                }
                counter = counter + 1;
            }
            None
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

            //problem section
            for node in &self.graph.nodes {
                if !other_located_nodes.contains(&node.0) {
                    return Some(*node.0);
                }
            }
            None
        }

        pub fn set_dijkstra(&mut self, node_set: Vec<i64>, target_id: i64) -> Option<u64> {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
            //set target (-1) for all-node-settle rather than just target settle or smth

            for id in &node_set {
                //pushes all nodes in set into PQ with distance 0
                let node_from_set = *self.graph.nodes.get(&id).unwrap_or_else(|| {
                    panic!("node {} not found, now length is {}", id, node_set.len())
                });

                let pathed_node_from_set: PathedNode = PathedNode {
                    node_self: (node_from_set),
                    distance_from_start: 0,
                    parent_node: (None),
                };
                priority_queue.push(Reverse((0, pathed_node_from_set.clone())));
            }

            let mut counter = 1;
            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                if let Some(_) = (self.visited_nodes.insert(
                    pathed_current_node.node_self.id,
                    pathed_current_node.distance_from_start,
                )) {
                    continue;
                }

                if pathed_current_node.node_self.id.eq(&target_id) {
                    return Some(self.get_path(pathed_current_node).1);
                }

                for neighbor_node in self.get_neighbors(&pathed_current_node, false) {
                    // something to check if node was already done stuff to and skip this whole thing if yes
                    let temp_distance = pathed_current_node.distance_from_start + neighbor_node.1;
                    let next_distance = neighbor_node.0.distance_from_start;
                    if temp_distance < next_distance {
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
                            node_self: neighbor_node.0.node_self,
                            distance_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };
                        priority_queue.push(Reverse((temp_distance, tentative_new_node)));
                    }
                }
                counter = counter + 1;
            }
            None
        }
    }
}

mod landmark_algo {
    use crate::routing::*;
    use std::collections::HashMap;
    /* //failed greedy node that i didnt need to make anyway

    pub fn select_landmarks(dijkstra_graph: &mut Dijkstra, num_landmarks: usize) -> Vec<i64> {
        let mut node_set: Vec<i64> = Vec::new();
        node_set.push(dijkstra_graph.get_random_node_id().unwrap()); //push random first node to start
        let mut distance = 0;
        let mut max_dist_node = -1;
        while node_set.len() < num_landmarks {
            let target_set = dijkstra_graph.graph.nodes.clone();
            let target_ids = target_set.keys();
            for target_id in target_ids {
                let temp = dijkstra_graph.set_dijkstra(&node_set, *target_id).unwrap();
                if temp > distance {
                    distance = temp;
                    max_dist_node = *target_id
                }
            }
            node_set.push(max_dist_node);
        }
        node_set
    }

    pub fn landmark_heuristic(graph: &RoadNetwork, target: i64, landmarks: &Vec<i64>) -> HashMap<i64, u64> {
        let target = *graph.nodes.get(&target).unwrap();
        let landmark_nodes = landmarks
            .iter()
            .map(|id| *graph.nodes.get(id).unwrap())
            .collect::<Vec<Node>>();
        //for each current i64 id, enter euciladan distance from current to target, divided by max speed on that path
        let heuristics =
            graph
                .nodes
                .iter()
                .map(|(id, head)| {
                    (
                        *id,landmark_nodes.iter().map(|landmark|
                        (((i128::pow(((head.lat - landmark.lat) * 111229).into(), 2) as f64
                            / f64::powi(10.0, 14)
                            + i128::pow(((head.lon - landmark.lon) * 71695).into(), 2) as f64
                                / f64::powi(10.0, 14))
                        .sqrt() as u64)
                            / ((110 as f64) * 5.0 / 18.0) as u64) //110 is motorway speed --> max speed possible on road network
                            .abs_diff(((i128::pow(((landmark.lat - target.lat) * 111229).into(), 2) as f64
                        / f64::powi(10.0, 14)
                        + i128::pow(((landmark.lon - target.lon) * 71695).into(), 2) as f64
                            / f64::powi(10.0, 14))
                    .sqrt() as u64)
                        / ((110 as f64) * 5.0 / 18.0) as u64) //110 is motorway speed --> max speed possible on road network
                    ).max().unwrap()
                    )
                })
                .collect::<HashMap<i64, u64>>();
        //println!("to {} is distance est {:?}", target, heuristics.clone().into_values());
        heuristics
    }
    */

    pub fn landmark_heuristic_precompute(
        dijkstra_graph: &mut Dijkstra,
        num_landmarks: usize,
    ) -> HashMap<i64, HashMap<i64, u64>> {
        let roads = dijkstra_graph.graph.clone();
        let empty_hash = HashMap::new();
        let mut landmarks = Vec::new();
        for _ in 0..num_landmarks {
            landmarks.push(dijkstra_graph.get_random_node_id().unwrap());
        }

        landmarks
            .iter()
            .map(|&l| {
                (l, {
                    let mut graph = Dijkstra::new(&roads);
                    graph.dijkstra(l, -1, &empty_hash, false);
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
                        .map(|(_, &ref arr)| {
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
    use crate::graph_construction::*;
    use crate::routing::*;

    pub struct ArcFlags {
        //precomputation stuff for arc flag routing algorithm
        lat_min: i64,
        lat_max: i64, 
        lon_min: i64,
        lon_max: i64
    }

    impl ArcFlags {
        pub fn new(lat_min: f32, lat_max: f32, lon_min: f32, lon_max: f32) -> ArcFlags {
            ArcFlags{
                lat_min: (lat_min * f32::powi(10.0, 7)) as i64,
                lat_max: (lat_max * f32::powi(10.0, 7)) as i64,
                lon_min: (lon_min * f32::powi(10.0, 7)) as i64,
                lon_max: (lon_max * f32::powi(10.0, 7)) as i64,
            }
        }
        pub fn arc_flags_precompute(self, mut routing_graph: Dijkstra) {
            let region_nodes = routing_graph.graph.nodes.iter().filter(|(_,&node)| node.lat < self.lat_max && node.lat> self.lat_min && node.lon < self.lon_max && node.lon > self.lon_min).map(|(id, _)| *id).collect::<Vec<i64>>();
            let mut boundary_node = Vec::new();
            for node in region_nodes.clone() {
                if let Some(ref edge_list) = routing_graph.graph.edges.get(&node) {
                    for edge in **edge_list {
                        if region_nodes.contains(edge.0) {
                            edge.1.1 = true;
                        }
                        else {
                            boundary_node.push(node);

                        }
                    }
                }
            }
            for node in boundary_node {

            }
            //region_nodes.iter().map(|(id, _)| routing_graph.graph.edges.get(*id).unwrap()); 
        }
    }

}
fn main() {}

#[cfg(test)]
mod tests {
    use crate::graph_construction::*;
    use crate::landmark_algo::*;
    use crate::routing::*;

    //use std::collections::HashMap;

    use std::collections::HashMap;
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
        roads = roads.reduce_to_largest_connected_component();
        println!(
            "reduced map nodes {}, and edges {}",
            roads.nodes.len(),
            roads
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
                / 2
        );
        let mut routing_graph = Dijkstra::new(&roads);
        let mut shortest_path_costs = Vec::new();
        let mut query_time = Vec::new();
        let mut settled_nodes = Vec::new();
        let mut heuristics;
        let now = Instant::now();
        //let precompute = landmark_heuristic_precompute(&mut routing_graph, 42); //i think i did it the hard way with greedy the first time
        let mut time = now.elapsed().as_millis() as f32 * 0.001;
        println!("pre done {}", time);
        for _ in 0..100 {
            let source = routing_graph.get_random_node_id().unwrap();
            let target = routing_graph.get_random_node_id().unwrap();
            heuristics = HashMap::new();
            //heuristics = a_star_heuristic(&roads, target); //sets heurstic values for a*, comment out for base Dijkstra
            //heuristics = landmark_heuristic(&roads, target, &landmark_list); //a* with landmarks on greedy pick
            //heuristics = landmark_heuristic(&precompute, &routing_graph, target);
            routing_graph = Dijkstra::new(&roads);
            let now = Instant::now();
            let result = routing_graph.dijkstra(source, target, &heuristics, false);
            time = now.elapsed().as_millis() as f32 * 0.001;
            query_time.push(time);
            shortest_path_costs.push(result.unwrap_or((vec![], 0)).1);
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
        );
    }

    /*

        /*
    #[test]
    fn testing_weird_iter_stuff() {
        let test: Vec<u32> = vec![1, 2, 3, 4, 5, 6, 7, 8, 10, 1, 2, 3, 4, 5, 6, 7, 8, 10, 1, 2, 3, 4, 5, 6, 7, 8, 10, 1, 2, 3, 4, 5, 6, 7, 8, 10, 1, 2, 3, 4, 5, 6, 7, 8, 10];
        let test2: Vec<u32> = vec![10, 10, 10, 10, 10];
        for node in 0..8 {
            let mut index = 0;
            let b = test.iter().filter(|x| **x!= 10).skip(node).step_by(8).map(|a| {
                let dist = a.abs_diff(test2[index]);
                index = index + 1;
                dist}
            ).max().unwrap();
            println!("{:?}", b);
        }
    }
    */
        use std::collections::HashMap;
        fn cost(head: Node, tail: Node, speed: u64) -> u64 {
            let a = i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64 / f64::powi(10.0, 14);
            let b = i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64 / f64::powi(10.0, 14);
            let c = (a + b).sqrt();
            (c / ((speed as f64) * 5.0 / 18.0)) as u64 // km / kmph == h (hours)
        }

        #[test]
        fn djikstra_test() {
            let node1 = Node {
                id: 1,
                lat: 495000000,
                lon: 70000000,
            };
            let node2 = Node {
                id: 2,
                lat: 493500000,
                lon: 71250000,
            };
            let node3 = Node {
                id: 3,
                lat: 492500000,
                lon: 72500000,
            };
            let node4 = Node {
                id: 4,
                lat: 497500000,
                lon: 72500000,
            };
            let roads = RoadNetwork {
                nodes: HashMap::from([(1, node1), (2, node2), (3, node3), (4, node4)]),
                edges: HashMap::from([
                    //(2, HashMap::from([(1, cost(node1, node2, 30))])),
                    (1, HashMap::from([(2, cost(node1, node2, 10)), (3, cost(node1, node3, 70))])),
                    //(3, HashMap::from([(2, cost(node3, node2, 30))])),
                    (2, HashMap::from([(3, cost(node3, node2, 10))])),
                    //(3, HashMap::from([(4, cost(node3, node4, 7))])),
                    //(4, HashMap::from([(3, cost(node4, node3, 7))])),
                    //(3, HashMap::from([(1, cost(node3, node1, 7))])),
                    //(1, HashMap::from([(3, cost(node1, node3, 7))])),
                ]),
            };
            println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
            let mut shortest_path_graph = Dijkstra::new(roads);
            let source = 1;
            let target = 3;
            println!(
                "dijiktra path and cost {:?}",
                shortest_path_graph.dijkstra(source, target)
            );
        }
    */
}
