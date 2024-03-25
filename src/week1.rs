use osmpbfreader::objects::OsmObj;
use std::{collections::HashMap, time::Instant};

#[derive(Debug, PartialEq, Hash, Eq, Clone, Copy)]
pub struct Node {
    id: i64,
    lat: i64,
    lon: i64,
}
#[derive(Debug, PartialEq, Hash, Eq)]
pub struct Way {
    id: i64,
    road_type: String,
    refs: Vec<i64>,
}

#[derive(Debug, PartialEq)]
pub struct RoadNetwork {
    nodes: HashMap<i64, Node>,
    edges: HashMap<i64, HashMap<i64, u32>>, // hash < tail.id hash < head.id, cost>
}

pub fn cost_calc(head: Node, tail: Node, road_type: &String) -> u32 {
    let kmh = match road_type.as_str() {
        "motorway" => 110,
        "trunk" => 110,
        "primary" => 70,
        "secondary" => 60,
        "tertiary" => 50,
        "motorway_link" => 50,
        "trunk_link" => 50,
        "primary_link" => 50,
        "secondary_link" => 50,
        "road" => 40,
        "unclassified" => 40,
        "residential" => 30,
        "unsurfaced" => 30,
        "living_street" => 10,
        "service" => 5,
        _ => 0,
    };
    let distance =
        f64::sqrt((i64::pow(head.lat - tail.lat, 2) + i64::pow(head.lon - tail.lon, 2)) as f64);
    (distance / (kmh as f64)) as u32
}

impl RoadNetwork {
    pub fn new(nodes: HashMap<i64, Node>, ways: Vec<Way>) -> Self {
        println!("Construct start");
        let now = Instant::now();
        let mut edges: HashMap<i64, HashMap<i64, u32>> = HashMap::new();
        let mut counter = ways.len();
        for way in ways {
            println!("{}", counter);
            counter = counter - 1;
            for i in 0..way.refs.len()-1 {
                let tail_id = way.refs[i];
                let tail = nodes.get(&tail_id);
                let head_id = way.refs[i+1];
                let head = nodes.get(&head_id);
                if let (Some(tail), Some(head)) = (tail, head) {
                    let cost = cost_calc(*head, *tail, &way.road_type);
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
                }
            }
        }
        println!("Construct time: {}", now.elapsed().as_secs_f32());
        Self { nodes, edges }
    }

    pub fn read_from_osm_file(path: &str) -> Option<(HashMap<i64, Node>, Vec<Way>)> {
        println!("Reading start");
        let now = Instant::now();
        let mut nodes = HashMap::new();
        let mut ways = Vec::new();
        let path_cleaned = std::path::Path::new(&path);
        let r = std::fs::File::open(&path_cleaned).unwrap();
        let mut reader = osmpbfreader::OsmPbfReader::new(r);
        println!("Reading time {}", now.elapsed().as_secs_f32());
        for obj in reader.iter().map(Result::unwrap) {
            match obj {
                OsmObj::Node(e) => {
                    nodes.insert(
                        e.id.0,
                        Node {
                            id: e.id.0,
                            lat: (e.lat() * f64::powi(10.0, 7)) as i64, //remember to backconvert when pulling val
                            lon: (e.lon() * f64::powi(10.0, 7)) as i64, //remember to backconvert when pulling val
                        },
                    );
                },
                OsmObj::Way(e) => {
                    let tags = e.tags.clone().into_inner();
                    let road_type = tags.iter().find(|(k, _)| k.eq(&"highway"));
                    let road_type = match road_type {
                        Some((_k, v)) => v.to_string(),
                        None => String::from(""),
                    };
                    ways.push(Way {
                        id: e.id.0,
                        road_type,
                        refs: e.nodes.into_iter().map(|x| x.0).collect(),
                    });
                },
                _ => {}
            }
        }
        Some((nodes, ways))
    }
}

#[cfg(test)]
mod tests {
    use crate::RoadNetwork;

    #[test]
    fn cmon_do_something() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        println!("Nodes: {}, Ways: {}", data.0.len(), data.1.len());
    }

    #[test]
    fn constructing_roadnet() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!("Nodes: {}, Edges: {}", roads.nodes.len(), roads.edges.len());
    }
}

fn main() {}
