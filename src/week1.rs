use osmpbfreader::objects::OsmObj;
use std::collections::HashMap;

#[derive(Debug, PartialEq, Hash, Eq, Clone, Copy)]
pub struct Node {
    id: i64,
    lat: i64,
    lon: i64,
}
#[derive(Debug, PartialEq, Hash, Eq)]
pub struct Way {
    id: i64,
    speed: u16,
    refs: Vec<i64>,
}

#[derive(Debug, PartialEq)]
pub struct RoadNetwork {
    nodes: HashMap<i64, Node>,
    edges: HashMap<i64, HashMap<i64, u32>>, // hash < tail.id hash < head.id, cost>
}

pub fn speed_calc(highway: &str) -> Option<u16> {
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
    pub fn new(nodes: HashMap<i64, Node>, ways: Vec<Way>) -> Self {
        let mut edges: HashMap<i64, HashMap<i64, u32>> = HashMap::new();
        println!("# of ways {}", ways.len());
        for way in ways {
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
                    let cost = ((i64::pow((head.lat - tail.lat)*111229, 2) as f64/ f64::powi(10.0, 7)) 
                                + (i64::pow((head.lon - tail.lon)*71695, 2) as f64/ f64::powi(10.0, 7)).sqrt() 
                                / (way.speed as f64)) as u32;
                    edges
                        .entry(tail_id).and_modify(|inner| {
                            inner.insert(head_id, cost);
                        }).or_insert({
                            let mut a = HashMap::new();
                            a.insert(head_id, cost);
                            a
                        });
                    edges
                        .entry(head.id).and_modify(|inner| {
                            inner.insert(tail_id, cost);
                        }).or_insert({
                            let mut a = HashMap::new();
                            a.insert(tail_id, cost);
                            a
                        });
                    previous_head_node_now_tail = Some(&head);
                    previous_head_node_index = i + 1;
                }
            }
        }
        Self { nodes, edges }
    }

    pub fn read_from_osm_file(path: &str) -> Option<(HashMap<i64, Node>, Vec<Way>)> {
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
                    if let Some(road_type) = e.tags.clone().iter().find(|(k, _)| k.eq(&"highway")) {
                        if let Some (speed) = speed_calc(road_type.1.as_str()) {
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
}

#[cfg(test)]
mod tests {
    use crate::RoadNetwork;
    /*
    #[test]
    fn saarland_roadnet() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
    }*/

    #[test]
    fn bw_roadnet() {
        let data = RoadNetwork::read_from_osm_file("baden-wuerttemberg_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
    }
}

fn main() {}
