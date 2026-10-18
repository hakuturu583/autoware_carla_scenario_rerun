//! The Lanelet2 map of a run: finding it, reading it once per process, and
//! placing it in the recording's frame.
//!
//! ACS relates a Lanelet2 map to CARLA's world through an offset
//! (`MapManager.mgrs_offset`, `z_offset`): `x_map = x + ox`,
//! `y_map = -y + oy`, `z_map = z + z_offset`. The recording's frame is
//! CARLA's with `y` flipped, so a map point lands at
//! `(x_map - ox, y_map - oy, z_map - z_offset)`. Version 2 files carry the
//! offsets in their `map` record. Without them (version 1) the offset is the
//! projection of the map origin, which is what ACS computes (zero for the
//! CARLA towns' `Local` maps), and `z_offset` is taken as 0.

pub mod osm;
pub mod projection;

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};

pub use osm::{Boundary, BoundaryKind, MapGeometry};
pub use projection::{Projection, Projector};

/// The file Autoware ships beside a map to say how to project it.
pub const PROJECTOR_INFO_FILENAME: &str = "map_projector_info.yaml";
/// The Lanelet2 file name ACS prefers in a map directory.
pub const LANELET2_FILENAME: &str = "lanelet2_map.osm";

/// The parts of `map_projector_info.yaml` (or of a `map` record's
/// `projector_info`) this crate reads.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ProjectorInfo {
    pub projector_type: Option<String>,
    /// `map_origin`: (latitude, longitude, altitude).
    pub map_origin: Option<(f64, f64, f64)>,
}

impl ProjectorInfo {
    /// Reads the flat YAML Autoware writes (`projector_type: MGRS`, and a
    /// `map_origin:` block of `latitude`/`longitude`/`altitude`).
    pub fn from_yaml(text: &str) -> Self {
        let mut info = Self::default();
        let (mut lat, mut lon, mut alt) = (None, None, None);
        let mut in_origin = false;
        for raw in text.lines() {
            let line = raw.split('#').next().unwrap_or("");
            if line.trim().is_empty() {
                continue;
            }
            let indented = line.starts_with([' ', '\t']);
            let Some((key, value)) = line.trim().split_once(':') else {
                continue;
            };
            let value = value.trim().trim_matches(['"', '\'']);
            if !indented {
                in_origin = key == "map_origin";
                if key == "projector_type" && !value.is_empty() {
                    info.projector_type = Some(value.to_string());
                }
            } else if in_origin {
                let v = value.parse::<f64>().ok();
                match key {
                    "latitude" => lat = v,
                    "longitude" => lon = v,
                    "altitude" => alt = v,
                    _ => {}
                }
            }
        }
        if let (Some(lat), Some(lon)) = (lat, lon) {
            info.map_origin = Some((lat, lon, alt.unwrap_or(0.0)));
        }
        info
    }

    /// Reads a `map` record's `projector_info` object.
    pub fn from_json(value: &serde_json::Map<String, serde_json::Value>) -> Self {
        let origin = value.get("map_origin").and_then(|o| {
            let num = |k: &str| {
                o.get(k)
                    .and_then(|v| v.as_f64().or_else(|| v.as_str()?.parse().ok()))
            };
            Some((
                num("latitude")?,
                num("longitude")?,
                num("altitude").unwrap_or(0.0),
            ))
        });
        Self {
            projector_type: value
                .get("projector_type")
                .and_then(|v| v.as_str())
                .map(str::to_string),
            map_origin: origin,
        }
    }

    /// Reads the descriptor beside a Lanelet2 file, if there is one.
    pub fn beside(lanelet2: &Path) -> Option<Self> {
        let path = lanelet2.parent()?.join(PROJECTOR_INFO_FILENAME);
        std::fs::read_to_string(path)
            .ok()
            .map(|t| Self::from_yaml(&t))
    }
}

/// The `+lat_0`/`+lon_0`/`+h_0` of an OpenDRIVE `geoReference`, which ACS
/// anchors a map without a stated origin at.
pub fn geo_reference(xodr: &str) -> Option<(f64, f64, f64)> {
    let start = xodr.find("<geoReference>")?;
    let end = xodr[start..].find("</geoReference>")? + start;
    let text = &xodr[start..end];
    let value = |key: &str| -> Option<f64> {
        let at = text.find(key)? + key.len();
        let rest = &text[at..];
        let stop = rest
            .find(|c: char| {
                !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E')
            })
            .unwrap_or(rest.len());
        rest[..stop].parse().ok()
    };
    Some((
        value("+lat_0=")?,
        value("+lon_0=")?,
        value("+h_0=").unwrap_or(0.0),
    ))
}

/// Decides the projection of a map the way ACS does: a forced projector type
/// first, then the descriptor (`projector_info`, or the file beside the
/// map), then MGRS; the origin from the descriptor, then the OpenDRIVE
/// `geoReference`, then (0, 0, 0).
pub fn resolve_projection(
    lanelet2: &Path,
    projector_type: Option<&str>,
    projector_info: Option<ProjectorInfo>,
    opendrive: Option<&Path>,
) -> Projection {
    let info = projector_info
        .or_else(|| ProjectorInfo::beside(lanelet2))
        .unwrap_or_default();
    let projector = projector_type
        .and_then(Projector::from_name)
        .or_else(|| {
            info.projector_type
                .as_deref()
                .and_then(Projector::from_autoware)
        })
        .unwrap_or(Projector::Mgrs);
    let origin = info
        .map_origin
        .or_else(|| {
            let xodr = opendrive.map(Path::to_path_buf).or_else(|| {
                let dir = lanelet2.parent()?;
                let mut found: Vec<PathBuf> = std::fs::read_dir(dir)
                    .ok()?
                    .flatten()
                    .map(|e| e.path())
                    .filter(|p| p.extension().is_some_and(|e| e == "xodr"))
                    .collect();
                found.sort();
                found.into_iter().next()
            })?;
            geo_reference(&std::fs::read_to_string(xodr).ok()?)
        })
        .unwrap_or((0.0, 0.0, 0.0));
    Projection { projector, origin }
}

/// Finds the Lanelet2 map of CARLA map `name` (e.g. `Town10HD_Opt`) under a
/// map directory: `<dir>/<name>/lanelet2_map.osm`, one level deeper
/// (`<dir>/<repo>/<name>/...`, the layout of ACS's map root), or the
/// directory's own `lanelet2_map.osm`.
pub fn find_in_dir(dir: &Path, name: &str) -> Option<PathBuf> {
    let direct = dir.join(name).join(LANELET2_FILENAME);
    if direct.is_file() {
        return Some(direct);
    }
    let mut deeper: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .flatten()
        .map(|e| e.path().join(name).join(LANELET2_FILENAME))
        .filter(|p| p.is_file())
        .collect();
    deeper.sort();
    if let Some(found) = deeper.into_iter().next() {
        return Some(found);
    }
    let own = dir.join(LANELET2_FILENAME);
    own.is_file().then_some(own)
}

type Slot = Arc<OnceLock<Result<Arc<MapGeometry>, String>>>;

fn cache() -> &'static Mutex<HashMap<String, Slot>> {
    static CACHE: OnceLock<Mutex<HashMap<String, Slot>>> = OnceLock::new();
    CACHE.get_or_init(Default::default)
}

/// A map read from the per-process cache.
#[derive(Clone, Debug)]
pub struct LoadedMap {
    pub geometry: Arc<MapGeometry>,
    /// Whether another conversion read it (this one found it read, or
    /// waited for that read).
    pub cached: bool,
}

/// Reads the Lanelet2 map at `path` with `projection`, once per process: the
/// map is keyed by `sha256` when the caller knows it (the `map` record), else
/// by the file's path, size and modification time, and by the projection.
/// Concurrent callers wait for the first one's read.
pub fn load(
    path: &Path,
    projection: &Projection,
    sha256: Option<&str>,
) -> Result<LoadedMap, String> {
    let meta = std::fs::metadata(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let identity = match sha256 {
        Some(hash) => format!("sha256:{hash}"),
        None => {
            let mtime = meta
                .modified()
                .ok()
                .and_then(|m| m.duration_since(std::time::UNIX_EPOCH).ok())
                .map_or(0, |d| d.as_nanos());
            format!("{}|{}|{mtime}", path.display(), meta.len())
        }
    };
    let key = format!("{identity}|{projection:?}");
    let slot = {
        let mut map = cache().lock().unwrap_or_else(|e| e.into_inner());
        map.entry(key).or_default().clone()
    };
    let mut read_here = false;
    let result = slot.get_or_init(|| {
        read_here = true;
        let file = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        // SAFETY: the map is only read; a file truncated underneath would
        // fault, as reading any file being rewritten would misbehave.
        let bytes =
            unsafe { memmap2::Mmap::map(&file) }.map_err(|e| format!("{}: {e}", path.display()))?;
        osm::parse(&bytes, projection)
            .map(Arc::new)
            .map_err(|e| format!("{}: {e}", path.display()))
    });
    result.clone().map(|geometry| LoadedMap {
        geometry,
        cached: !read_here,
    })
}

/// Empties the per-process map cache.
pub fn clear_cache() {
    cache().lock().unwrap_or_else(|e| e.into_inner()).clear();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_projector_info_yaml() {
        let info = ProjectorInfo::from_yaml(
            "# c\nprojector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n",
        );
        assert_eq!(info.projector_type.as_deref(), Some("MGRS"));
        assert_eq!(info.map_origin, None);
        let info = ProjectorInfo::from_yaml(
            "projector_type: TransverseMercator\nvertical_datum: WGS84\nmap_origin:\n  latitude: 35.5\n  longitude: 139.25\n  altitude: 3.0\n",
        );
        assert_eq!(info.map_origin, Some((35.5, 139.25, 3.0)));
    }

    #[test]
    fn reads_geo_reference() {
        let xodr = "<header><geoReference><![CDATA[+proj=tmerc +lat_0=35.25 +lon_0=-139.5 +k=1 +h_0=2.5]]></geoReference></header>";
        assert_eq!(geo_reference(xodr), Some((35.25, -139.5, 2.5)));
        assert_eq!(
            geo_reference("<geoReference>+lat_0=0 +lon_0=0</geoReference>"),
            Some((0.0, 0.0, 0.0))
        );
    }
}
