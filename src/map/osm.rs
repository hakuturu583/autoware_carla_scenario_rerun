//! Reading the lines of a Lanelet2 map (`.osm`) with quick-xml.
//!
//! Only what the recording uses is kept: the left and right bounds of every
//! lanelet (each linestring once, in the order lanelets first name them),
//! every `stop_line` linestring, and every traffic-light regulatory element
//! (its id, its lights and its stop lines: what names a CARLA light by its
//! Lanelet2 id). Points are in the map frame of the [`Projection`] they were
//! read with.

use std::collections::{HashMap, HashSet};

use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;

use super::projection::Projection;

/// How a lane boundary is painted, from its linestring's `type`/`subtype`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BoundaryKind {
    /// A painted solid line (`line_thin`/`line_thick` but `dashed`, ...).
    Solid,
    /// A dashed line.
    Dashed,
    /// A curb or a road border.
    Curb,
    /// No marking on the ground (`virtual`).
    Virtual,
}

/// A lane boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct Boundary {
    pub way: i64,
    pub kind: BoundaryKind,
    pub points: Vec<[f64; 3]>,
}

/// A traffic-light regulatory element (`type=regulatory_element`,
/// `subtype=traffic_light`): the id Autoware knows the light by.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficLightElement {
    pub id: i64,
    /// Its `refers` linestrings: the lights.
    pub lights: Vec<Vec<[f64; 3]>>,
    /// Its `ref_line` linestrings: where to stop.
    pub stop_lines: Vec<Vec<[f64; 3]>>,
}

/// The lines of a map, in its map frame.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapGeometry {
    pub lane_boundaries: Vec<Boundary>,
    pub stop_lines: Vec<Vec<[f64; 3]>>,
    /// The traffic-light regulatory elements, by id.
    pub traffic_lights: Vec<TrafficLightElement>,
    /// Nodes in the file.
    pub nodes: usize,
    /// Lanelets in the file.
    pub lanelets: usize,
}

#[derive(Default)]
struct Node {
    lat: f64,
    lon: f64,
    ele: f64,
}

#[derive(Default)]
struct Way {
    nodes: Vec<i64>,
    ty: String,
    subtype: String,
}

#[derive(Default)]
struct Relation {
    id: i64,
    ty: String,
    subtype: String,
    left: Vec<i64>,
    right: Vec<i64>,
    refers: Vec<i64>,
    ref_line: Vec<i64>,
}

enum Open {
    None,
    Node(i64, Node),
    Way(i64, Way),
    Relation(Relation),
}

fn number<T: std::str::FromStr>(text: &str) -> Option<T> {
    text.trim().parse().ok()
}

fn attr(e: &BytesStart<'_>, name: &str) -> Option<String> {
    e.attributes()
        .flatten()
        .find(|a| a.key.as_ref() == name)
        .map(|a| a.value.into_owned())
}

fn close(
    open: Open,
    projection: &Projection,
    nodes: &mut HashMap<i64, [f64; 3]>,
    ways: &mut HashMap<i64, Way>,
    lanelets: &mut Vec<Relation>,
) {
    match open {
        Open::Node(id, n) => {
            nodes.insert(id, projection.forward(n.lat, n.lon, n.ele));
        }
        Open::Way(id, w) => {
            ways.insert(id, w);
        }
        Open::Relation(r) => {
            if r.ty == "lanelet" || (r.ty == "regulatory_element" && r.subtype == "traffic_light") {
                lanelets.push(r);
            }
        }
        Open::None => {}
    }
}

fn kind(way: &Way) -> BoundaryKind {
    match way.ty.as_str() {
        "virtual" => BoundaryKind::Virtual,
        "curbstone" | "road_border" => BoundaryKind::Curb,
        _ if way.subtype.contains("dashed") => BoundaryKind::Dashed,
        _ => BoundaryKind::Solid,
    }
}

/// Reads the lines of the map in `xml`, projecting its nodes with `projection`.
pub fn parse(xml: &[u8], projection: &Projection) -> Result<MapGeometry, String> {
    let xml = std::str::from_utf8(xml).map_err(|e| format!("not UTF-8: {e}"))?;
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    let mut nodes: HashMap<i64, [f64; 3]> = HashMap::new();
    let mut ways: HashMap<i64, Way> = HashMap::new();
    let mut lanelets: Vec<Relation> = Vec::new();
    let mut open = Open::None;
    let mut count_nodes = 0usize;

    loop {
        let event = reader
            .read_event()
            .map_err(|e| format!("at byte {}: {e}", reader.error_position()))?;
        let (e, empty) = match &event {
            Event::Start(e) => (e, false),
            Event::Empty(e) => (e, true),
            Event::End(e) => {
                if matches!(e.name().as_ref(), "node" | "way" | "relation") {
                    close(
                        std::mem::replace(&mut open, Open::None),
                        projection,
                        &mut nodes,
                        &mut ways,
                        &mut lanelets,
                    );
                }
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        match e.name().as_ref() {
            "node" => {
                count_nodes += 1;
                let mut id = None;
                let mut node = Node::default();
                for a in e.attributes().flatten() {
                    match a.key.as_ref() {
                        "id" => id = number(&a.value),
                        "lat" => node.lat = number(&a.value).unwrap_or(f64::NAN),
                        "lon" => node.lon = number(&a.value).unwrap_or(f64::NAN),
                        _ => {}
                    }
                }
                let id = id.ok_or("a node without an id")?;
                open = Open::Node(id, node);
                if empty {
                    close(
                        std::mem::replace(&mut open, Open::None),
                        projection,
                        &mut nodes,
                        &mut ways,
                        &mut lanelets,
                    );
                }
            }
            "way" => {
                let id = attr(e, "id")
                    .and_then(|v| number(&v))
                    .ok_or("a way without an id")?;
                open = Open::Way(id, Way::default());
                if empty {
                    close(
                        std::mem::replace(&mut open, Open::None),
                        projection,
                        &mut nodes,
                        &mut ways,
                        &mut lanelets,
                    );
                }
            }
            "relation" => {
                let id = attr(e, "id").and_then(|v| number(&v)).unwrap_or(0);
                open = Open::Relation(Relation {
                    id,
                    ..Relation::default()
                });
                if empty {
                    close(
                        std::mem::replace(&mut open, Open::None),
                        projection,
                        &mut nodes,
                        &mut ways,
                        &mut lanelets,
                    );
                }
            }
            "nd" => {
                if let Open::Way(_, way) = &mut open {
                    if let Some(r) = attr(e, "ref").and_then(|v| number(&v)) {
                        way.nodes.push(r);
                    }
                }
            }
            "member" => {
                if let Open::Relation(rel) = &mut open {
                    let (mut ty, mut r, mut role) = (String::new(), None, String::new());
                    for a in e.attributes().flatten() {
                        match a.key.as_ref() {
                            "type" => ty = a.value.into_owned(),
                            "ref" => r = number(&a.value),
                            "role" => role = a.value.into_owned(),
                            _ => {}
                        }
                    }
                    if let ("way", Some(r)) = (ty.as_str(), r) {
                        match role.as_str() {
                            "left" => rel.left.push(r),
                            "right" => rel.right.push(r),
                            "refers" => rel.refers.push(r),
                            "ref_line" => rel.ref_line.push(r),
                            _ => {}
                        }
                    }
                }
            }
            "tag" => {
                let (mut k, mut v) = (String::new(), String::new());
                for a in e.attributes().flatten() {
                    match a.key.as_ref() {
                        "k" => k = a.value.into_owned(),
                        "v" => v = a.value.into_owned(),
                        _ => {}
                    }
                }
                match &mut open {
                    Open::Node(_, n) => {
                        if k == "ele" {
                            n.ele = number(&v).unwrap_or(0.0);
                        }
                    }
                    Open::Way(_, w) => match k.as_str() {
                        "type" => w.ty = v,
                        "subtype" => w.subtype = v,
                        _ => {}
                    },
                    Open::Relation(r) => match k.as_str() {
                        "type" => r.ty = v,
                        "subtype" => r.subtype = v,
                        _ => {}
                    },
                    Open::None => {}
                }
            }
            _ => {}
        }
    }

    let points = |way: &Way| -> Vec<[f64; 3]> {
        way.nodes
            .iter()
            .filter_map(|id| nodes.get(id).copied())
            .collect()
    };
    let (lanelets, elements): (Vec<Relation>, Vec<Relation>) =
        lanelets.into_iter().partition(|r| r.ty == "lanelet");
    let lines = |ids: &[i64]| -> Vec<Vec<[f64; 3]>> {
        ids.iter()
            .filter_map(|id| ways.get(id))
            .map(points)
            .collect()
    };
    let mut traffic_lights: Vec<TrafficLightElement> = elements
        .iter()
        .map(|r| TrafficLightElement {
            id: r.id,
            lights: lines(&r.refers),
            stop_lines: lines(&r.ref_line),
        })
        .collect();
    traffic_lights.sort_by_key(|t| t.id);
    let mut seen = HashSet::new();
    let mut lane_boundaries = Vec::new();
    for lanelet in &lanelets {
        for id in lanelet.left.iter().chain(&lanelet.right) {
            if !seen.insert(*id) {
                continue;
            }
            if let Some(way) = ways.get(id) {
                lane_boundaries.push(Boundary {
                    way: *id,
                    kind: kind(way),
                    points: points(way),
                });
            }
        }
    }
    let mut stop_ids: Vec<i64> = ways
        .iter()
        .filter(|(_, w)| w.ty == "stop_line")
        .map(|(id, _)| *id)
        .collect();
    stop_ids.sort_unstable();
    let stop_lines = stop_ids.iter().map(|id| points(&ways[id])).collect();
    Ok(MapGeometry {
        lane_boundaries,
        stop_lines,
        traffic_lights,
        nodes: count_nodes,
        lanelets: lanelets.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::projection::Projector;

    const OSM: &str = r#"<?xml version="1.0"?>
<osm version="0.6" generator="lanelet2">
  <node id="1" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
  <node id="2" lat="0" lon="0.0001"><tag k="ele" v="1.5"/></node>
  <node id="3" lat="0.00003" lon="0"/>
  <node id="4" lat="0.00003" lon="0.0001"></node>
  <node id="5" lat="0" lon="0"/>
  <way id="10"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="dashed"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/><tag k="type" v="curbstone"/></way>
  <way id="12"><nd ref="2"/><nd ref="4"/><tag k="type" v="stop_line"/></way>
  <relation id="20">
    <member type="way" ref="11" role="left"/><member type="way" ref="10" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
  <relation id="21"><member type="way" ref="10" role="left"/><tag k="type" v="regulatory_element"/></relation>
  <way id="14"><nd ref="3"/><nd ref="5"/><tag k="type" v="traffic_light"/><tag k="height" v="1.2"/></way>
  <relation id="30">
    <member type="way" ref="14" role="refers"/><member type="way" ref="12" role="ref_line"/>
    <tag k="type" v="regulatory_element"/><tag k="subtype" v="traffic_light"/>
  </relation>
</osm>"#;

    #[test]
    fn reads_bounds_and_stop_lines() {
        let projection = Projection {
            projector: Projector::Local,
            origin: (0.0, 0.0, 0.0),
        };
        let map = parse(OSM.as_bytes(), &projection).unwrap();
        assert_eq!(map.nodes, 5);
        assert_eq!(map.lanelets, 1);
        assert_eq!(map.lane_boundaries.len(), 2);
        assert_eq!(map.lane_boundaries[0].way, 11);
        assert_eq!(map.lane_boundaries[0].kind, BoundaryKind::Curb);
        assert_eq!(map.lane_boundaries[1].kind, BoundaryKind::Dashed);
        let at = |lat, lon, ele| projection.forward(lat, lon, ele);
        assert_eq!(
            map.lane_boundaries[1].points,
            vec![[0.0, 0.0, 0.0], at(0.0, 0.0001, 1.5)]
        );
        assert_eq!(
            map.stop_lines,
            vec![vec![at(0.0, 0.0001, 1.5), at(0.00003, 0.0001, 0.0)]]
        );
        assert!(
            (map.stop_lines[0][0][0] - 11.13).abs() < 0.05,
            "{:?}",
            map.stop_lines
        );
    }

    #[test]
    fn reads_traffic_light_elements() {
        let projection = Projection {
            projector: Projector::Local,
            origin: (0.0, 0.0, 0.0),
        };
        let map = parse(OSM.as_bytes(), &projection).unwrap();
        let at = |lat, lon, ele| projection.forward(lat, lon, ele);
        // The other regulatory element (21) is not a light; no lanelet counts it.
        assert_eq!(map.lanelets, 1);
        assert_eq!(
            map.traffic_lights,
            vec![TrafficLightElement {
                id: 30,
                lights: vec![vec![at(0.00003, 0.0, 0.0), [0.0, 0.0, 0.0]]],
                stop_lines: vec![vec![at(0.0, 0.0001, 1.5), at(0.00003, 0.0001, 0.0)]],
            }]
        );
    }

    #[test]
    fn bad_xml_is_an_error() {
        let projection = Projection {
            projector: Projector::Local,
            origin: (0.0, 0.0, 0.0),
        };
        assert!(parse(
            b"<osm><node id=\"1\" lat=\"0\" lon=\"0\"></way></osm>",
            &projection
        )
        .is_err());
    }
}
