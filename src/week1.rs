use osmpbf::*;
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
    tags: Vec<(String, String)>,
    refs: Vec<i64>,
}

#[derive(Debug, PartialEq)]
pub struct RoadNetwork {
    nodes: HashMap<i64, Node>,
    outgoing_arcs: Vec<Vec<Option<Edge>>>,
}

#[derive(Debug, PartialEq)]
pub struct Edge {
    head: Node, //destination
    cost: f64,
}

pub fn cost_calc(head: Node, tail: Node, way: &Way) -> f64 {
    let road_type = way.tags.iter().find(|(k, _v)| k.eq(&"highway"));
    let road_type = match road_type {
        Some((_k, v)) => v.as_str(),
        None => "",
    };

    let kmh = match road_type {
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
    distance / (kmh as f64)
}

impl RoadNetwork {
    pub fn new(nodes: HashMap<i64, Node>, ways: Vec<Way>) -> Self {
        println!("Construct start");
        let now = Instant::now();
        let mut outgoing_arcs: Vec<Vec<Option<Edge>>> = Vec::new();
        let node_iter = nodes.iter();
        //println!("{}",);
        for k in node_iter {
            let adjacent_edges: Vec<Option<Edge>> = ways
                .iter()
                .map(|way| {
                    let pos = way.refs.iter().position(|m| m == k.0);
                    let index = pos.unwrap() + 1;
                    let node_ref = way.refs.iter().nth(index);
                    match node_ref {
                        Some(node_ref) => {
                            let head = nodes.get(&node_ref).unwrap();
                            let cost = cost_calc(*head, *k.1, way);
                            Some(Edge { head: *head, cost })
                        }
                        None => None,
                    }
                })
                .collect();
            outgoing_arcs.push(adjacent_edges);
        }
        println!("Construct time: {}", now.elapsed().as_secs());
        Self {
            nodes,
            outgoing_arcs,
        }
    }

    pub fn read_from_osm_file(path: &str) -> Option<(HashMap<i64, Node>, Vec<Way>)> {
        println!("Reading start");
        let now = Instant::now();
        let reader = ElementReader::from_path(path).ok()?;
        let mut nodes = HashMap::new();
        //nodes.insert(3, Node{id:11, lat:11, lon:11});
        let mut ways = Vec::new();
        let _ = reader.for_each(|e: Element<'_>| match e {
            Element::Way(e) => ways.push(Way {
                id: e.id(),
                tags: e
                    .tags()
                    .map(|(a, b)| (a.to_string(), b.to_string()))
                    .collect(),
                refs: e.raw_refs().to_vec(),
            }),
            _ => (),
        });
        let reader = ElementReader::from_path(path).ok()?;
        let _ = reader.for_each(|e: Element<'_>| match e {
            Element::DenseNode(e) => {
                nodes.insert(
                    e.id(),
                    Node {
                        id: e.id(),
                        lat: (e.lat() * f64::powi(10.0, 7)) as i64,
                        lon: (e.lon() * f64::powi(10.0, 7)) as i64,
                    },
                );
                return ();
            }
            _ => (),
        });
        println!("Reading time {}", now.elapsed().as_secs());
        Some((nodes, ways))
    }
}

#[cfg(test)]
mod tests {
    use crate::RoadNetwork;

    #[test]
    fn cmon_do_something() {
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        println!(
            "Nodes: {}, Ways: {}",
            data.0.len(),
            data.1.len()
        );
    }

    #[test]
    fn constructing_roadnet(){
        let data = RoadNetwork::read_from_osm_file("saarland_01.pbf").unwrap();
        let roads = RoadNetwork::new(data.0, data.1);
        println!(
            "Nodes: {}, Edges: {}",
            roads.nodes.len(),
            roads.outgoing_arcs.len()
        );
    }
}

fn main() {}
