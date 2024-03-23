use vadeen_osm::geo::Coordinate;
/*
Implement a simple class RoadNetwork for an undirected graph with arc costs.
Add a method readFromOsmFile to your class that reads an OSM file (in XML format) and
constructs the corresponding road network. Translate the road types to speeds.
Read the OSM files for Saarland and Baden-Wurttemberg.
Add a line to the table linked to from the course Wiki, stating your name, the number of arcs
and nodes in your graphs, and any other information asked for there.
*/
use std::time::Instant;
use vadeen_osm::osm_io::{error::Error as osm_err, read};
use vadeen_osm::*;

#[derive(Debug, PartialEq)]
pub struct RoadNetwork {
    nodes: Vec<Node>,
    outgoing_arcs: Vec<Vec<Option<Edge>>>,
}

#[derive(Debug, PartialEq)]
pub struct Edge {
    head: Node, //destination
    cost: f64,
}

pub fn cost_calc(head: Coordinate, tail: Coordinate, way_tags: Vec<Tag>) -> f64 {
    let road_type = way_tags.into_iter().find(|k| k.key.eq(&"highway"));
    let road_type = match road_type.clone() {
        Some(tag) => tag.value,
        None => "".to_owned(),
    };

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
        f64::sqrt(f64::powi(head.lat() - tail.lat(), 2) + f64::powi(head.lon() - tail.lon(), 2));
    distance / (kmh as f64)
}

impl RoadNetwork {
    pub fn new(osm: Osm) -> Self {
        let now = Instant::now();
        let nodes = osm.nodes;
        let ways = osm.ways;
        let mut outgoing_arcs: Vec<Vec<Option<Edge>>> = Vec::new();
        for n in 1..nodes.len() {
            let adjacent_edges: Vec<Option<Edge>> = ways
                .iter()
                .map(|way| {
                    let pos = way.refs.iter().position(|&m| m == nodes[n-1].id);
                    if pos.is_some() {
                        let index = pos.unwrap() + 1;
                        if index > way.refs.len() {
                            return None;
                        }
                        let target = way.refs[index];
                        let head = nodes
                            .iter()
                            .find(|node: &&Node| node.id == target)
                            .unwrap()
                            .clone();
                        let cost =
                            cost_calc(head.coordinate, nodes[n].coordinate, way.meta.tags.clone());
                        Some(Edge { head, cost })
                    } else {
                        None
                    }
                })
                .collect();
            outgoing_arcs.push(adjacent_edges);
        }
        println!("Construct time: {}", now.elapsed().as_secs());
        RoadNetwork {
            nodes,
            outgoing_arcs,
            //osm.ways transformed into the edge thing i just wrote up there it should get the head node (the destination) and the costs caluclatd using cost_calc
            //need to figure out where to get edclid distance from like start node to this one???
        }
    }
    pub fn read_from_osm_file(path: &str) -> Result<Osm, osm_err> {
        let now = Instant::now();
        let osm = read(path)?;
        println!("Reading time {}", now.elapsed().as_secs());
        Ok(osm)
    }
}

#[cfg(test)]
mod tests {
    use crate::RoadNetwork;

    #[test]
    fn run_the_thing() {
        let osm = RoadNetwork::read_from_osm_file("saarland.osm").unwrap();
        let result = RoadNetwork::new(osm);
        print!(
            "#nodes: {}, #arcs: {}",
            result.nodes.len(),
            result.outgoing_arcs.len()
        );
    }
}

fn main() {}
