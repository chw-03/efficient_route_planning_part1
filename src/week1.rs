/* 
Implement a simple class RoadNetwork for an undirected graph with arc costs.
Add a method readFromOsmFile to your class that reads an OSM file (in XML format) and
constructs the corresponding road network. Translate the road types to speeds.
Read the OSM files for Saarland and Baden-Wurttemberg.
Add a line to the table linked to from the course Wiki, stating your name, the number of arcs
and nodes in your graphs, and any other information asked for there. 
*/



use vadeen_osm::osm_io::{read, error::Error as osm_err};
use vadeen_osm::*;
pub struct RoadNetwork {
    data: Osm,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
}

pub enum Speed {
    motorway,
    trunk,
    primary,
    secondary,
    tertiary,
    motorway_link, 
    trunk_link,
    primary_link,
    secondary_link,
    road,
    unclassified,
    residential,
    unsurfaced,   
    living_street,
    service,
}


pub struct Edge {
    head: Node,
    cost: f64,
}

pub fn cost_calc(distance: f64, road_type: Speed) -> f64 { //path: Way
    //let road_type = path.meta.tags; //tag k = highway, v = ......
    let kmh= match(road_type) {
        Speed::motorway => 110,
        Speed::trunk => 110,
        Speed::primary=> 70,
        Speed::secondary=> 60,
        Speed::tertiary=> 50,
        Speed::motorway_link=> 50, 
        Speed::trunk_link=> 50,
        Speed::primary_link=> 50,
        Speed::secondary_link=> 50,
        Speed::road=> 40,
        Speed::unclassified=> 40,
        Speed::residential=>30,
        Speed::unsurfaced=>30,   
        Speed::living_street=>10,
        Speed::service  => 5,
    };
    distance / (kmh as f64)
}


impl RoadNetwork {
    pub fn new(path: &str) -> Self {
        let osm = Self::read_from_osm_file(path);
        RoadNetwork {
            data,
            nodes,
            edges,
        }
    }
    pub fn read_from_osm_file(path: &str) -> Result<Osm, osm_err> {
        let osm = read(path)?;
        Ok(osm)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}

fn main(){

}