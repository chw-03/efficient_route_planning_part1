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
            if !node.1.eq(&source) {
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

        while !priority_queue.is_empty() {
            let current_node = priority_queue.pop().unwrap().0; //.0 "unwraps" from Reverse()

            if pathed_current_node.node_self.eq(&target) {
                return Some(Self::get_path(
                    pathed_current_node,
                    stored_distance_per_node,
                ));
            }
            let neighbors_of_current = Self::get_neighbors(self, current_node.1);
            for neighbor_node in neighbors_of_current {
                let temp_distance =
                    *stored_distance_per_node.get(&current_node.1.id).unwrap() + neighbor_node.0;
                if let Some(next_distance) = stored_distance_per_node.get(&neighbor_node.1.id) {
                    if temp_distance < *next_distance {
                        if let Some(x) = stored_distance_per_node.get_mut(&neighbor_node.1.id) {
                            *x = temp_distance;
                        }
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node);
                        pathed_current_node = PathedNode {
                            node_self: neighbor_node.1,
                            parent_node: Some(Rc::clone(&prev_node)),
                        };
                        drop(prev_node);
                    }
                }
            }
        }
        None
    }
}

fn main() {}

#[cfg(test)]
mod tests {
    use crate::{Dijkstra, Node, RoadNetwork};

    #[test]
    fn saarland_dijkstra() {
        println!("starting");
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
            shortest_path_graph.dijkstra(source, target).unwrap().1
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
