mod week1;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::rc::Rc;
use std::u64::MAX;
use week1::*;

/*  Add a method reduceToLargestConnectedComponent to your RoadNetwork class that reduces
        the graph (already read from an OSM file) to its largest connected component.
    Run Dijkstra’s algorithm for 100 random queries for both of our OSM graphs
        Use the version of the graph reduced to its largest connected component
    Report the average running time and the average shortest path length per query
*/
pub struct Dijkstra {
    graph: RoadNetwork,
    visited_nodes: HashMap<i64, (PathedNode, u32)>,
    settled_nodes: HashMap<i64, u64>,
}

#[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord)]
pub struct PathedNode {
    node_self: Node,
    distance_from_start: u64,
    parent_node: Option<Rc<PathedNode>>,
}

impl PathedNode {
    pub fn extract_parent<PathedNode: std::clone::Clone>(last_elem: Rc<PathedNode>) -> PathedNode {
        let inner: PathedNode =
            //Rc::try_unwrap(last_elem).unwrap_or_else(|_| panic!("parent shared, unwrap failed"));
            Rc::unwrap_or_clone(last_elem);
        inner
    }
}

//implementation of dijkstra's shortest path algorithm
impl Dijkstra {
    pub fn new(graph: RoadNetwork) -> Self {
        let visited_nodes = HashMap::new();
        let settled_nodes = HashMap::new();
        Self {
            graph,
            visited_nodes,
            settled_nodes,
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
            //paths.push((path.1, *self.graph.nodes.get(&path.0).unwrap()));
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
            //println!("currently at {:?} and trying to unwrap {:?}", current.node_self, current.parent_node);
            shortest_path.push(current.node_self);
            //let prev_distance: Vec<&(i64, u64)> = stored_distances.iter().filter(|x| x.0 == current.node_self.id).collect();
            total_distance = total_distance + current.distance_from_start;
            //total_distance = total_distance + stored_distances.get(&current.node_self.id).unwrap();
            current = PathedNode::extract_parent(previous_node);
        }
        shortest_path.push(current.node_self);
        (shortest_path, total_distance)
    }

    pub fn dijkstra(&mut self, source_id:i64, target_id: i64) -> Option<(Vec<Node>, u64)> {
        //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
        let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
        //set target (-1) for all-node-settle rather than just target settle or smth
        let source = *self.graph.nodes.get(&source_id).unwrap_or_else(|| panic!("source node not found"));

        let source_node: PathedNode = PathedNode {
            node_self: (source),
            distance_from_start: 0,
            parent_node: (None),
        };        
        priority_queue.push(Reverse((0, source_node.clone())));

        //let _ = self.graph.nodes.iter().map(|node|{
        /*let graph_clone = self.graph.nodes.clone();
        for node in graph_clone {
            if !node.0.eq(&source.id) {
                self.visited_nodes.push(0);
                priority_queue.push(Reverse((MAX, node.1)));
            }
        }*/
        //});

        let mut counter = 1;
        while !priority_queue.is_empty() {
            let mut pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
            if let Some(_) = self.visited_nodes.insert(pathed_current_node.node_self.id, (pathed_current_node.clone(), counter)) {
                continue;
            }

            if pathed_current_node.node_self.id.eq(&target_id) {
                return Some(self.get_path(pathed_current_node));
            }

            for neighbor_node in self.get_neighbors(&pathed_current_node) {
                // something to check if node was already done stuff to and skip this whole thing if yes
                let temp_distance = pathed_current_node.distance_from_start + neighbor_node.1;
                println!("temp {}, edge {}", temp_distance, neighbor_node.1);
                //*self.stored_distance_per_node.get(&current_node.1.id).unwrap() + neighbor_node.0;
                //if let Some(next_distance) = self.stored_distance_per_node.get(&neighbor_node.1.id) {
                let next_distance = neighbor_node.0.distance_from_start;
                if temp_distance < next_distance {
                    println!("change");
                    let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                    let tentative_new_node = PathedNode {
                        node_self: neighbor_node.0.node_self,
                        distance_from_start: temp_distance,
                        parent_node: Some(prev_node),
                    };
                    //if let Some(_) = self.visited_nodes.insert(neighbor_node.0.node_self.id, (tentative_new_node.clone(), counter)){
                        priority_queue.push(Reverse((temp_distance, tentative_new_node)));
                    //}
                }
                  /*if !priority_queue
                    .iter()
                    .any(|member| member.0 .1.node_self.id == neighbor_node.0.node_self.id)
                {
                    //priority_queue.push(Reverse((temp_distance, neighbor_node.0.clone())));
                }*/
            }
            counter = counter + 1;
        }
        println!("not found");
        None
    }
}

fn main() {}

#[cfg(test)]
mod tests {
    use crate::{Dijkstra, Node, RoadNetwork};
    use std::collections::HashMap;

    #[test]
    fn uci_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("uci.osm.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("a");
        println!("a");
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
        let source =8925472275;
        let target = 122610516;
        println!(
            "edges with source {:?}",
            roads.edges.get(&source).clone()
        );
        let mut shortest_path_graph = Dijkstra::new(roads);
        println!(
            "dijiktra path and cost {:?}",
            shortest_path_graph.dijkstra(source, target)
        );
    }
/*
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
    
    #[test]
    fn saarland_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
        let mut shortest_path_graph = Dijkstra::new(roads);
        let source = 1020974368;
        let target = 1020974185;
        println!(
            "dijiktra path and cost {:?}",
            shortest_path_graph.dijkstra(source, target)
        );
    }

    /*
    #[test]
    fn bw_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("baden-wuerttemberg_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
    }
    */
}
