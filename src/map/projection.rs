//! Projecting a Lanelet2 map's lat/lon into its metric map frame, the way
//! autoware_carla_scenario (and Autoware) read it.
//!
//! ACS chooses the projector from the map's `map_projector_info.yaml`
//! (`coordinate/projection.py`): `Local` maps are read with Lanelet2's
//! `UtmProjector` at the map origin (which reproduces the `local_x`/`local_y`
//! tags of the CARLA towns' maps; the tags themselves are not read, as
//! Lanelet2 does not read them); `MGRS` with Autoware's `MGRSProjector` (UTM, then the
//! position inside the 100 km grid square); `TransverseMercator` and
//! `LocalCartesian` with theirs. No descriptor means `MGRS`.
//!
//! The transverse Mercator is Krüger's series to order n^6 (what
//! GeographicLib's `TransverseMercator` evaluates), accurate to a few
//! nanometres within a UTM zone.

/// The projection a map is read with.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Projector {
    /// `projector_type: Local`: as ACS reads it, Lanelet2's `UtmProjector`
    /// at the origin (the same as [`Projector::Utm`]).
    Local,
    /// `projector_type: MGRS`: UTM in each point's own zone, then the
    /// position inside its 100 km MGRS square.
    Mgrs,
    /// Lanelet2's `UtmProjector` at the origin: UTM in the origin's zone,
    /// relative to the origin.
    Utm,
    /// `projector_type: TransverseMercator`: a transverse Mercator (k0 0.9996)
    /// centred on the origin's meridian, relative to the origin.
    TransverseMercator,
    /// `projector_type: LocalCartesian`: east-north-up at the origin.
    LocalCartesian,
}

impl Projector {
    /// The projector for an Autoware `projector_type`, as ACS maps them.
    pub fn from_autoware(name: &str) -> Option<Self> {
        Some(match name {
            "Local" => Self::Local,
            "MGRS" => Self::Mgrs,
            "TransverseMercator" => Self::TransverseMercator,
            "LocalCartesian" | "LocalCartesianUTM" => Self::LocalCartesian,
            _ => return None,
        })
    }

    /// The projector for ACS's own names (`map.projector_type`: `mgrs`,
    /// `utm`, `local_cartesian`, `transverse_mercator`), or an Autoware one.
    pub fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "mgrs" => Self::Mgrs,
            "utm" => Self::Utm,
            "local_cartesian" => Self::LocalCartesian,
            "transverse_mercator" => Self::TransverseMercator,
            "local" => Self::Local,
            other => return Self::from_autoware(other),
        })
    }
}

/// A projector and the origin it is anchored at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Projection {
    pub projector: Projector,
    /// (latitude, longitude, altitude), degrees and metres.
    pub origin: (f64, f64, f64),
}

impl Projection {
    /// The map-frame point of a node at lat/lon (degrees) and elevation `ele`.
    pub fn forward(&self, lat: f64, lon: f64, ele: f64) -> [f64; 3] {
        let (olat, olon, oalt) = self.origin;
        match self.projector {
            Projector::Mgrs => {
                let [x, y] = mgrs(lat, lon);
                [x, y, ele]
            }
            Projector::Local | Projector::Utm => {
                let [x, y] = utm_relative(lat, lon, olat, olon);
                [x, y, ele]
            }
            Projector::TransverseMercator => {
                let [x, y] = tm(lat, lon, olon, UTM_K0);
                let [_, oy] = tm(olat, olon, olon, UTM_K0);
                [x, y - oy, ele]
            }
            Projector::LocalCartesian => enu(lat, lon, ele, olat, olon, oalt),
        }
    }
}

/// WGS84.
const A: f64 = 6_378_137.0;
const F: f64 = 1.0 / 298.257_223_563;
/// UTM's scale on the central meridian.
pub const UTM_K0: f64 = 0.9996;

/// A transverse Mercator projection of (lat, lon) about the meridian `lon0`:
/// (easting, northing) in metres, without false easting or northing.
pub fn tm(lat: f64, lon: f64, lon0: f64, k0: f64) -> [f64; 2] {
    let n = F / (2.0 - F);
    let e = (F * (2.0 - F)).sqrt();
    let (n2, n3) = (n * n, n * n * n);
    let (n4, n5, n6) = (n2 * n2, n2 * n3, n3 * n3);
    let big_a = A / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);
    let alpha = [
        n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0
            + 7891.0 * n6 / 37800.0,
        13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0
            - 1_983_433.0 * n6 / 1_935_360.0,
        61.0 * n3 / 240.0 - 103.0 * n4 / 140.0
            + 15061.0 * n5 / 26880.0
            + 167_603.0 * n6 / 181_440.0,
        49561.0 * n4 / 161_280.0 - 179.0 * n5 / 168.0 + 6_601_661.0 * n6 / 7_257_600.0,
        34729.0 * n5 / 80640.0 - 3_418_889.0 * n6 / 1_995_840.0,
        212_378_941.0 * n6 / 319_334_400.0,
    ];
    let phi = lat.to_radians();
    let lambda = (lon - lon0).to_radians();
    let sin_phi = phi.sin();
    let t = (sin_phi.atanh() - e * (e * sin_phi).atanh()).sinh();
    let xi_p = t.atan2(lambda.cos());
    let eta_p = (lambda.sin() / (1.0 + t * t).sqrt()).atanh();
    let (mut xi, mut eta) = (xi_p, eta_p);
    for (j, a) in alpha.iter().enumerate() {
        let k = 2.0 * (j + 1) as f64;
        xi += a * (k * xi_p).sin() * (k * eta_p).cosh();
        eta += a * (k * xi_p).cos() * (k * eta_p).sinh();
    }
    [k0 * big_a * eta, k0 * big_a * xi]
}

/// The standard UTM zone of a point (GeographicLib `UTMUPS::StandardZone`,
/// with the Norway and Svalbard exceptions).
pub fn utm_zone(lat: f64, lon: f64) -> i32 {
    let lon = ((lon + 180.0).rem_euclid(360.0)) - 180.0;
    let mut zone = ((lon + 180.0) / 6.0).floor() as i32 + 1;
    if zone > 60 {
        zone = 60;
    }
    let band = (lat.floor().clamp(-80.0, 79.0) as i32 + 80) / 8 - 10;
    if band == 7 && zone == 31 && lon >= 3.0 {
        zone = 32; // Norway
    } else if band == 9 && (0.0..42.0).contains(&lon) {
        // Svalbard
        zone = 2 * (((lon + 183.0) / 12.0).floor() as i32) + 1;
    }
    zone
}

/// UTM (easting, northing) of a point in `zone`, false easting and northing
/// included.
pub fn utm_in_zone(lat: f64, lon: f64, zone: i32, north: bool) -> [f64; 2] {
    let lon0 = f64::from(zone) * 6.0 - 183.0;
    let [x, y] = tm(lat, lon, lon0, UTM_K0);
    [x + 500_000.0, y + if north { 0.0 } else { 10_000_000.0 }]
}

/// UTM of a point in its own standard zone.
pub fn utm(lat: f64, lon: f64) -> [f64; 2] {
    utm_in_zone(lat, lon, utm_zone(lat, lon), lat >= 0.0)
}

/// Autoware's `MGRSProjector::forward`: the UTM position modulo the 100 km
/// grid square.
pub fn mgrs(lat: f64, lon: f64) -> [f64; 2] {
    let [x, y] = utm(lat, lon);
    [x.rem_euclid(1e5), y.rem_euclid(1e5)]
}

/// Lanelet2's `UtmProjector` with an offset: UTM in the origin's zone,
/// relative to the origin.
pub fn utm_relative(lat: f64, lon: f64, olat: f64, olon: f64) -> [f64; 2] {
    let zone = utm_zone(olat, olon);
    let north = olat >= 0.0;
    let [x, y] = utm_in_zone(lat, lon, zone, north);
    let [ox, oy] = utm_in_zone(olat, olon, zone, north);
    [x - ox, y - oy]
}

fn ecef(lat: f64, lon: f64, h: f64) -> [f64; 3] {
    let e2 = F * (2.0 - F);
    let (phi, lambda) = (lat.to_radians(), lon.to_radians());
    let n = A / (1.0 - e2 * phi.sin().powi(2)).sqrt();
    [
        (n + h) * phi.cos() * lambda.cos(),
        (n + h) * phi.cos() * lambda.sin(),
        (n * (1.0 - e2) + h) * phi.sin(),
    ]
}

/// East-north-up of a point at an origin (GeographicLib `LocalCartesian`).
pub fn enu(lat: f64, lon: f64, h: f64, olat: f64, olon: f64, oh: f64) -> [f64; 3] {
    let p = ecef(lat, lon, h);
    let o = ecef(olat, olon, oh);
    let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
    let (phi, lambda) = (olat.to_radians(), olon.to_radians());
    let (sp, cp, sl, cl) = (phi.sin(), phi.cos(), lambda.sin(), lambda.cos());
    [
        -sl * d[0] + cl * d[1],
        -sp * cl * d[0] - sp * sl * d[1] + cp * d[2],
        cp * cl * d[0] + cp * sl * d[1] + sp * d[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn utm_matches_known_points() {
        // GeoConvert: 35.62318359651 139.77840697094 -> 54S 389376.63 3942842.26
        // (the Odaiba map's MGRS 54SUE 89376 42842, its local_x/local_y).
        let [e, n] = utm(35.623_183_596_51, 139.778_406_970_94);
        assert_eq!(utm_zone(35.6, 139.7), 54);
        let [x, y] = mgrs(35.623_183_596_51, 139.778_406_970_94);
        assert!(close(x, 89_376.626_1, 0.01), "{x}");
        assert!(close(y, 42_842.260_8, 0.01), "{y}");
        assert!(close(e.rem_euclid(1e5), x, 1e-9) && close(n.rem_euclid(1e5), y, 1e-9));
    }

    #[test]
    fn equator_central_meridian_is_false_easting() {
        let [e, n] = utm(0.0, 3.0);
        assert!(close(e, 500_000.0, 1e-6) && close(n, 0.0, 1e-6));
    }

    #[test]
    fn zone_exceptions() {
        assert_eq!(utm_zone(60.0, 5.0), 32);
        assert_eq!(utm_zone(78.0, 10.0), 33);
        assert_eq!(utm_zone(0.0, 180.0), 1);
        assert_eq!(utm_zone(0.0, 179.9), 60);
    }

    // The references below are what simple-lanelet2 1.2.x's projectors
    // (Lanelet2's UtmProjector and LocalCartesianProjector, Autoware's
    // MGRSProjector and TransverseMercatorProjector) return for the same
    // points.
    const ODAIBA: (f64, f64) = (35.623_183_596_51, 139.778_406_970_94);

    #[test]
    fn local_and_utm_are_lanelet2s_utm_projector() {
        for projector in [Projector::Local, Projector::Utm] {
            let p = Projection {
                projector,
                origin: (0.0, 0.0, 0.0),
            };
            let [x, y, z] = p.forward(0.000_085_703_72, 0.000_799_908_11, 1.0);
            assert!(
                close(x, 89.132_690_011_814_7, 1e-6) && close(y, 9.485_916_941_464_126, 1e-6),
                "{x} {y}"
            );
            assert_eq!(z, 1.0);
        }
        let p = Projection {
            projector: Projector::Utm,
            origin: (35.6, 139.7, 0.0),
        };
        let [x, y, _] = p.forward(ODAIBA.0, ODAIBA.1, 5.0);
        assert!(
            close(x, 7_134.635_039_718_647, 1e-6) && close(y, 2_480.425_059_856_847, 1e-6),
            "{x} {y}"
        );
    }

    #[test]
    fn mgrs_is_autowares_mgrs_projector() {
        let p = Projection {
            projector: Projector::Mgrs,
            origin: (0.0, 0.0, 0.0),
        };
        let [x, y, _] = p.forward(ODAIBA.0, ODAIBA.1, 0.0);
        assert!(
            close(x, 89_376.626_069_512_33, 1e-6) && close(y, 42_842.260_853_851_214, 1e-6),
            "{x} {y}"
        );
    }

    #[test]
    fn transverse_mercator_is_autowares() {
        let p = Projection {
            projector: Projector::TransverseMercator,
            origin: (35.6, 139.7, 0.0),
        };
        let [x, y, _] = p.forward(ODAIBA.0, ODAIBA.1, 0.0);
        assert!(
            close(x, 7_100.098_940_314_067, 1e-6) && close(y, 2_574.062_863_058_876, 1e-6),
            "{x} {y}"
        );
    }

    #[test]
    fn local_cartesian_is_lanelet2s() {
        let p = Projection {
            projector: Projector::LocalCartesian,
            origin: (35.6, 139.7, 0.0),
        };
        let [x, y, z] = p.forward(ODAIBA.0, ODAIBA.1, 5.0);
        assert!(
            close(x, 7_102.942_742_037_821, 1e-6) && close(y, 2_575.093_254_884_134, 1e-6),
            "{x} {y}"
        );
        assert!(close(z, 0.527_884_371_541_858_9, 1e-6), "{z}");
    }

    #[test]
    fn enu_at_origin_is_zero() {
        let [e, n, u] = enu(35.0, 139.0, 10.0, 35.0, 139.0, 10.0);
        assert!(close(e, 0.0, 1e-6) && close(n, 0.0, 1e-6) && close(u, 0.0, 1e-6));
        let [e, n, _] = enu(35.0, 139.001, 0.0, 35.0, 139.0, 0.0);
        assert!(close(n, 0.0, 0.01) && close(e, 91.29, 0.1), "{e} {n}");
    }
}
