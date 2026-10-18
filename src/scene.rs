//! CARLA's world frame into the recording's.
//!
//! CARLA's world frame is left-handed (x forward/east, y right/south, z up);
//! the recording's is the same with `y` flipped -- right-handed, `z` up, the
//! frame ACS hands a driver -- because the viewer is right-handed and would
//! otherwise mirror the scene.

/// A CARLA world point (or vector) in the recording's frame.
#[inline]
pub fn point(p: [f64; 3]) -> [f64; 3] {
    [p[0], -p[1], p[2]]
}

/// [`point`], as the viewer's `f32`.
#[inline]
pub fn point32(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, -p[1] as f32, p[2] as f32]
}

/// The `xyzw` quaternion of a CARLA rotation (roll, pitch, yaw, degrees) in
/// the recording's frame.
///
/// CARLA's rotation matrix is `Rz(yaw) Ry(-pitch) Rx(-roll)` in its
/// left-handed frame; mirrored in `y` that is `Rz(-yaw) Ry(-pitch) Rx(roll)`
/// (the same as ACS's in-process recorder).
#[inline]
pub fn quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let hz = (-yaw).to_radians() / 2.0;
    let hy = (-pitch).to_radians() / 2.0;
    let hx = roll.to_radians() / 2.0;
    let (sz, cz) = hz.sin_cos();
    let (sy, cy) = hy.sin_cos();
    let (sx, cx) = hx.sin_cos();
    [
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
        cx * cy * cz + sx * sy * sz,
    ]
}

/// Rotates `v` by the unit quaternion `q` (`xyzw`).
#[inline]
pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    // t = 2 * cross(q.xyz, v); v' = v + w * t + cross(q.xyz, t)
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

/// The centre of an actor's bounding box in the recording's frame: its origin
/// plus the box offset (`center`, in the actor's own frame) turned with it.
#[inline]
pub fn box_center(position: [f64; 3], q: [f64; 4], offset: [f64; 3]) -> [f64; 3] {
    let p = point(position);
    let o = rotate(q, point(offset));
    [p[0] + o[0], p[1] + o[1], p[2] + o[2]]
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn yaw_turns_clockwise_seen_from_above_in_carla() {
        // CARLA yaw +90 faces +y (south); flipped, that is -y.
        let q = quaternion(0.0, 0.0, 90.0);
        assert!(close(rotate(q, [1.0, 0.0, 0.0]), [0.0, -1.0, 0.0]));
    }

    #[test]
    fn pitch_up_raises_the_nose() {
        let q = quaternion(0.0, 30.0, 0.0);
        let f = rotate(q, [1.0, 0.0, 0.0]);
        assert!(f[2] > 0.49 && f[2] < 0.51, "{f:?}");
    }

    proptest! {
        #[test]
        fn quaternions_are_unit(roll in -180.0f64..180.0, pitch in -90.0f64..90.0, yaw in -360.0f64..360.0) {
            let q = quaternion(roll, pitch, yaw);
            let n = q.iter().map(|c| c * c).sum::<f64>();
            prop_assert!((n - 1.0).abs() < 1e-12);
        }

        #[test]
        fn rotation_keeps_length(yaw in -360.0f64..360.0, pitch in -90.0f64..90.0, x in -10.0f64..10.0, y in -10.0f64..10.0) {
            let v = rotate(quaternion(0.0, pitch, yaw), [x, y, 1.0]);
            let n0 = x * x + y * y + 1.0;
            let n1 = v.iter().map(|c| c * c).sum::<f64>();
            prop_assert!((n0 - n1).abs() < 1e-9);
        }

        #[test]
        fn yaw_matches_the_flipped_planar_rotation(yaw in -360.0f64..360.0, x in -10.0f64..10.0, y in -10.0f64..10.0) {
            // In CARLA's frame: (x cos - y sin, x sin + y cos); then flip y.
            let r = yaw.to_radians();
            let carla = [x * r.cos() - y * r.sin(), x * r.sin() + y * r.cos(), 0.0];
            let v = rotate(quaternion(0.0, 0.0, yaw), point([x, y, 0.0]));
            prop_assert!(close(v, point(carla)));
        }
    }
}
