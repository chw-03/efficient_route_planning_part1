/*  Run Dijkstra’s algorithm for 100 random queries for both of our OSM graphs
        Use the version of the graph reduced to its largest connected component
    Report the average running time and the average shortest path length per query
*/
#[allow(unused)]
mod routing {
    use crate::graph_construction::*;
    use rand::Rng;
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::rc::Rc;
    use std::u64::MAX;
    pub struct Dijkstra {
        pub graph: RoadNetwork,
        pub visited_nodes: HashSet<i64>,
        //settled_nodes: HashMap<i64, u64>,
    }

    #[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord)]
    pub struct PathedNode {
        node_self: Node,
        distance_from_start: u64,
        parent_node: Option<Rc<PathedNode>>,
    }

    impl PathedNode {
        pub fn extract_parent<PathedNode: std::clone::Clone>(
            last_elem: Rc<PathedNode>,
        ) -> PathedNode {
            let inner: PathedNode = Rc::unwrap_or_clone(last_elem);
            inner
        }
    }

    //implementation of dijkstra's shortest path algorithm
    impl Dijkstra {
        pub fn new(graph: &RoadNetwork) -> Self {
            let visited_nodes = HashSet::new();
            //let settled_nodes = HashMap::new();
            Self {
                graph: graph.clone(),
                visited_nodes,
                //settled_nodes,
            }
        }
        //return node id of neighbors
        pub fn get_neighbors(&mut self, current: &PathedNode) -> Vec<(PathedNode, u64)> {
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
                    path.1,
                ));
            }
            paths
        }

        //Uses reference to find the source node with parent_node == None
        pub fn get_path(&mut self, target: PathedNode) -> (Vec<Node>, u64) {
            let mut shortest_path: Vec<Node> = Vec::new();
            let mut total_distance: u64 = 0;
            let mut current = target;
            while let Some(previous_node) = current.parent_node {
                shortest_path.push(current.node_self);
                total_distance = total_distance + current.distance_from_start;
                current = PathedNode::extract_parent(previous_node);
            }
            shortest_path.push(current.node_self);
            (shortest_path, total_distance)
        }

        pub fn dijkstra(&mut self, source_id: i64, target_id: i64) -> Option<(Vec<Node>, u64)> {
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

            let mut counter = 1;
            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                if !(self.visited_nodes.insert(pathed_current_node.node_self.id)) {
                    continue;
                }

                if pathed_current_node.node_self.id.eq(&target_id) {
                    return Some(self.get_path(pathed_current_node));
                }

                for neighbor_node in self.get_neighbors(&pathed_current_node) {
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

        pub fn get_random_unvisted_node_id(&mut self) -> Option<i64> {
            if self.visited_nodes.len() == self.graph.nodes.len() {
                println!("all nodes visted");
                return None;
            }
            let mut full_node_list = Vec::new();
            let mut rng = rand::thread_rng();
            for node in &self.graph.nodes {
                full_node_list.push(node.0);
            }

            let mut random: usize = rng.gen_range(0..full_node_list.len());
            let mut node_id = full_node_list.get(random).unwrap();

            while self.visited_nodes.contains(node_id) {
                print!("b");
                random = rng.gen_range(0..=full_node_list.len());
                node_id = full_node_list.get(random).unwrap();
            }

            Some(**node_id)
        }
    }
}

#[allow(unused)]
mod graph_construction {
    use crate::routing::*;
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
        pub nodes: HashMap<i64, Node>,              // HASH < node.id, node >
        pub edges: HashMap<i64, HashMap<i64, u64>>, // HASH < tail.id, HASH < head.id, cost >
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
            let mut edges: HashMap<i64, HashMap<i64, u64>> = HashMap::new();
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
                        let a = i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64
                            / f64::powi(10.0, 14);
                        let b = i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64
                            / f64::powi(10.0, 14);
                        let c = (a + b).sqrt();
                        let cost = (c / ((way.speed as f64) * 5.0 / 18.0)) as u64; //meters per second
                        edges
                            .entry(tail_id)
                            .and_modify(|inner| {
                                inner.insert(head_id, cost);
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(head_id, cost);
                                a
                            });
                        edges
                            .entry(head.id)
                            .and_modify(|inner| {
                                inner.insert(tail_id, cost);
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(tail_id, cost);
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
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<i64, i32> = HashMap::new();
            let mut shortest_path_graph = Dijkstra::new(&self);
            let mut max_connections = 0;

            while let Some(source_id) = shortest_path_graph.get_random_unvisted_node_id() {
                if number_times_node_visted.len() == self.nodes.len() {
                    break;
                }

                counter = counter + 1;
                let mut shortest_path_graph = Dijkstra::new(&self);
                shortest_path_graph.dijkstra(source_id, -1);
                for node in &shortest_path_graph.visited_nodes {
                    number_times_node_visted.insert(*node, counter);
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
fn main() {}

#[cfg(test)]
mod tests {
    use crate::graph_construction::*;
    use crate::routing::*;
    /*
        #[test]
        fn uci_dijkstra() {
            let data = RoadNetwork::read_from_osm_file("uci.osm.pbf").unwrap();
            let mut roads = RoadNetwork::new(data.0, data.1);
            println!(
                "Nodes: {}, Edges: {}",
                roads.nodes.len(),
                roads
                    .edges
                    .iter()
                    .map(|(_, edges)| edges.len())
                    .sum::<usize>()
            );
            let source = 8925472275;
            let target = 122610516;
            let mut shortest_path_graph = Dijkstra::new(&roads);
            println!(
                "dijiktra path and cost {:?}\n",
                shortest_path_graph.dijkstra(source, target)
            );
            println!("\n{}\n", shortest_path_graph.visited_nodes.len());
            roads = roads.reduce_to_largest_connected_component();
            shortest_path_graph = Dijkstra::new(&roads);
            println!(
                "reduced map nodes {}, and edges {}",
                roads.nodes.len(),
                roads
                    .edges
                    .iter()
                    .map(|(_, edges)| edges.len())
                    .sum::<usize>()
            );
            println!("\n{}\n", shortest_path_graph.visited_nodes.len());
            println!(
                "dijiktra path and cost {:?}\n",
                shortest_path_graph.dijkstra(source, target)
            );
            println!("\n{}\n", shortest_path_graph.visited_nodes.len());
        }
    */

    #[test]
    fn saarland_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        let mut roads = RoadNetwork::new(data.0, data.1);
        println!(
            "Nodes: {}, Edges: {} \n",
            roads.nodes.len(),
            roads.edges.len()
        );
        let mut shortest_path_graph = Dijkstra::new(&roads);
        let source = 1329323405; //1020974368
        let target = 1020974185;
        println!(
            "dijiktra path and cost {:?}\n",
            shortest_path_graph.dijkstra(source, target),
        );
        roads = roads.reduce_to_largest_connected_component();
        let mut shortest_path_graph = Dijkstra::new(&roads);
        println!(
            "reduced map nodes {}, and edges {}\n",
            roads.nodes.len(),
            roads.edges.len()
        );
        println!(
            "dijiktra path and cost {:?}\n",
            shortest_path_graph.dijkstra(source, target)
        );
    }

    /*
    #[test]
    fn bw_roadnet() {
        let data = RoadNetwork::read_from_osm_file("baden-wuerttemberg_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
    }
    */

    /*
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
