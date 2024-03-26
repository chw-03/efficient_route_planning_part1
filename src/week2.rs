mod week1;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::rc::Rc;
use std::u16::MAX;
use week1::*;

/*  Add a method reduceToLargestConnectedComponent to your RoadNetwork class that reduces
        the graph (already read from an OSM file) to its largest connected component.
    Run Dijkstra’s algorithm for 100 random queries for both of our OSM graphs
        Use the version of the graph reduced to its largest connected component
    Report the average running time and the average shortest path length per query
*/
pub struct Dijkstra {
    graph: RoadNetwork,
    visited_nodes: Vec<u8>,
}

#[derive(Debug, PartialEq)]
pub struct PathedNode {
    node_self: Node,
    parent_node: Option<Rc<PathedNode>>,
}

impl PathedNode {
    pub fn extract_parent<PathedNode>(last_elem: Rc<PathedNode>) -> PathedNode {
        let inner: PathedNode =
            Rc::try_unwrap(last_elem).unwrap_or_else(|_| panic!("last_elem shared, unwrap failed"));
        inner
    }
}

//implementation of dijkstra's shortest path algorithm
impl Dijkstra {
    pub fn new(graph: RoadNetwork) -> Self {
        let visited_nodes = Vec::new();
        Self {
            graph,
            visited_nodes,
        }
    }
    //return node id of neighbors
    pub fn get_neighbors(&mut self, current: Node) -> Vec<(u16, Node)> {
        let mut paths = Vec::new();
        let mut next_node_edges = HashMap::new();
        if let Some(connections) = self.graph.edges.get(&current.id) {
            next_node_edges = connections.clone();
        }
        for path in next_node_edges {
            paths.push((path.1, *self.graph.nodes.get(&path.0).unwrap()));
        }
        paths
    }

    //Uses reference to find the source node with parent_node == None
    pub fn get_path(target: PathedNode, stored_distances: HashMap<i64, u16>) -> (Vec<Node>, u16) {
        let mut shortest_path: Vec<Node> = Vec::new();
        let mut total_distance: u16 = 0;
        let mut current = target;
        while let Some(previous_node) = current.parent_node {
            shortest_path.push(current.node_self);
            total_distance = total_distance + stored_distances.get(&current.node_self.id).unwrap();
            current = PathedNode::extract_parent(previous_node);
        }
        shortest_path.push(current.node_self);
        (shortest_path, total_distance)
    }

    pub fn dijkstra(&mut self, source: Node, target: Node) -> Option<(Vec<Node>, u16)> {
        //set target (-1) for all-node-settle rather than just target settle or smth
        self.visited_nodes.push(0);
        //cost to reach nth node
        let mut stored_distance_per_node = HashMap::new();
        stored_distance_per_node.insert(source.id, 0);

        //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
        let mut priority_queue: BinaryHeap<Reverse<(u16, Node)>> = BinaryHeap::new();
        priority_queue.push(Reverse((0, source)));

        //let _ = self.graph.nodes.iter().map(|node|{
        let graph_clone = self.graph.nodes.clone();
        for node in graph_clone {
            if !node.0.eq(&source.id) {
                self.visited_nodes.push(0);
                stored_distance_per_node.insert(node.0, MAX);
                priority_queue.push(Reverse((MAX, node.1)));
            }
        }
        //});

        let mut pathed_current_node: PathedNode = PathedNode {
            node_self: (source),
            parent_node: (None),
        };
        let mut counter = 0;
        while !priority_queue.is_empty() {
            
            let current_node = priority_queue.pop().unwrap().0; //.0 "unwraps" from Reverse()


            if pathed_current_node.node_self.id.eq(&target.id) {
                return Some(Self::get_path(
                    pathed_current_node,
                    stored_distance_per_node,
                ));
            }
            let neighbors_of_current = Self::get_neighbors(self, current_node.1);
            
            if counter == 2 {
                //println!("current {:?} and target {:?}", pathed_current_node.node_self, target);
            }

            for neighbor_node in neighbors_of_current {
                let temp_distance =
                    *stored_distance_per_node.get(&current_node.1.id).unwrap() + neighbor_node.0;

                if let Some(next_distance) = stored_distance_per_node.get(&neighbor_node.1.id) {
                    if temp_distance < *next_distance {
                        if let Some(x) = stored_distance_per_node.get_mut(&neighbor_node.1.id) {
                            *x = temp_distance;
                            let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node);
                            pathed_current_node = PathedNode {
                                node_self: neighbor_node.1,
                                parent_node: Some(Rc::clone(&prev_node)),
                            };
                            drop(prev_node);
                        }
                        
                    }
                }
                if counter == 0 {
                    //println!("current node {:?}, to distance {:?}", pathed_current_node.node_self, stored_distance_per_node.get(&neighbor_node.1.id));
                }
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
    //use std::collections::HashMap;
    use crate::{Dijkstra, Node, RoadNetwork};

    #[test]
    fn uci_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("uci.osm.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
        let source = Node {
            id: 8925472275,
            lat: 336428178,
            lon: -1178391486,
        };
        let target = Node {
            id: 122610516,
            lat: 336428062,
            lon: -1178390482,
        };
        //println!("edges with source {:?}", roads.edges.get(&source.id).clone());
        let mut shortest_path_graph = Dijkstra::new(roads);
        println!("dijiktra path and cost {:?}", shortest_path_graph.dijkstra(source, target));
    }

    /*
    fn cost(head: Node, tail: Node, speed: u16) -> f64 {
        let a = i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64 / f64::powi(10.0, 14);
        let b = i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64 / f64::powi(10.0, 14);
        let c = (a+b).sqrt();
        (c / ((speed as f64) * 5.0 / 18.0))  // km / kmph == h (hours)
    }
    
    #[test]
    fn djikstra_test() {
        let node1 = Node {id:1, lat: 495000000, lon: 70000000};
        let node2 = Node {id:2, lat: 493500000, lon: 71250000};
        let node3 =  Node {id:3, lat: 492500000, lon: 72500000};
        let node4 =  Node {id:4, lat: 497500000, lon: 72500000};
        let roads = RoadNetwork {
            nodes: HashMap::from([
                (1, node1),
                (2, node2),
                (3, node3),
            ]),
            edges: HashMap::from([
                (2, HashMap::from([(1, cost(node3, node2, 7))])),
                (1, HashMap::from([(2, cost(node1, node2, 7))])),
                (3, HashMap::from([(2, cost(node3, node2, 7))])),
                (2, HashMap::from([(3, cost(node1, node2, 7))])),
                (3, HashMap::from([(4, cost(node3, node4, 7))])),
                (4, HashMap::from([(3, cost(node4, node2, 7))])),
            ])
        };
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
        let mut shortest_path_graph = Dijkstra::new(roads);
        let source = Node {id:1, lat: 495000000, lon: 70000000};
        let target = Node {id:3, lat: 492500000, lon: 72500000};
        println!(
            "dijiktra path and cost {:?}",
            shortest_path_graph.dijkstra(source, target)
        );
    } */

    /*
    #[test]
    fn saarland_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
        let mut shortest_path_graph = Dijkstra::new(roads);
        let source = Node {
            id: 314060626,
            lat: 491468544,
            lon: 71136987,
        };
        let target = Node {
            id: 314060628,
            lat: 491467252,
            lon: 71141107,
        };
        println!(
            "dijiktra path and cost {:?}",
            shortest_path_graph.dijkstra(source, target)
        );
    }
    */
    /*
    #[test]
    fn bw_dijkstra() {
        let data = RoadNetwork::read_from_osm_file("baden-wuerttemberg_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
    }
    */
}
