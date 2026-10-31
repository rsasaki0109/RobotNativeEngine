//! Conservative box-pair sweep. Lists retain the original collider order.
use rne_math::Vec3;

/// `radius` bounds a box about its collider origin. Unknown shapes fall back
/// to the original all-pairs path, including both directions of mixed pairs.
pub(crate) fn candidates(bounds: &[Option<(Vec3, f64)>], margin_m: f64) -> Vec<Option<Vec<usize>>> {
    // For an accepted SAT pair, each first-box face projection is at most
    // half_i + radius_b(axis_i) + margin (+ the 1e-9 face-selection tie).
    // Thus center distance <= radius_a + sqrt(3)*radius_b + sqrt(3)*margin.
    // The symmetric bound also holds. Their average gives
    // (1 + sqrt(3))/2 * (radius_a + radius_b) + sqrt(3)*margin.
    // This is independent of box rotation and deliberately looser than an AABB.
    let boxes: Vec<_> = bounds
        .iter()
        .map(|bound| {
            bound.and_then(|(center, radius)| {
                if !center.is_finite()
                    || !radius.is_finite()
                    || radius < 0.0
                    || !margin_m.is_finite()
                {
                    return None;
                }
                let scale = center.abs().max_element() + radius + margin_m.abs() + 1.0;
                let extent = (1.0 + 3.0_f64.sqrt()) * 0.5 * radius
                    + 3.0_f64.sqrt() * margin_m.max(0.0) * 0.5
                    + 1e-8
                    + 64.0 * f64::EPSILON * scale;
                (extent.is_finite() && (center.abs().max_element() + extent).is_finite())
                    .then_some((center, extent))
            })
        })
        .collect();
    let unknown: Vec<_> = boxes
        .iter()
        .enumerate()
        .filter_map(|(i, b)| b.is_none().then_some(i))
        .collect();
    let mut result: Vec<_> = boxes.iter().map(|b| b.map(|_| unknown.clone())).collect();
    let mut order: Vec<_> = boxes
        .iter()
        .enumerate()
        .filter_map(|(i, b)| b.map(|_| i))
        .collect();
    order.sort_by(|&i, &j| {
        let (a, ra) = boxes[i].unwrap();
        let (b, rb) = boxes[j].unwrap();
        (a.x - ra).total_cmp(&(b.x - rb)).then(i.cmp(&j))
    });
    for (position, &i) in order.iter().enumerate() {
        let (a, ra) = boxes[i].unwrap();
        for &j in &order[position + 1..] {
            let (b, rb) = boxes[j].unwrap();
            if b.x - rb > a.x + ra {
                break;
            }
            let reach = ra + rb;
            if (a.y - b.y).abs() <= reach && (a.z - b.z).abs() <= reach {
                result[i].as_mut().unwrap().push(j);
                result[j].as_mut().unwrap().push(i);
            }
        }
    }
    for indices in result.iter_mut().flatten() {
        indices.sort_unstable();
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collide::box_box;
    use rne_math::Quat;
    use rne_world::Transform3;

    #[test]
    fn rotated_box_contacts_survive_sweep_in_original_order() {
        let mut poses = Vec::new();
        let mut halves = Vec::new();
        for i in 0..80 {
            halves.push(Vec3::new(0.02 + (i % 3) as f64 * 0.03, 0.04, 0.09));
            poses.push(Transform3::from_translation_rotation(
                Vec3::new(
                    (i % 10) as f64 * 0.07,
                    ((i / 10) % 4) as f64 * 0.08,
                    (i / 40) as f64 * 0.13,
                ),
                Quat::from_rotation_x(i as f64 * 0.13) * Quat::from_rotation_z(i as f64 * 0.31),
            ));
        }
        let bounds: Vec<_> = poses
            .iter()
            .zip(&halves)
            .map(|(p, h)| Some((p.translation, h.length())))
            .collect();
        for margin in [0.0, 0.02, 0.125] {
            let pairs = candidates(&bounds, margin);
            let mut contacts = 0;
            for i in 0..poses.len() {
                let indices = pairs[i].as_ref().unwrap();
                assert!(indices.windows(2).all(|w| w[0] < w[1]));
                for j in 0..poses.len() {
                    if i != j
                        && !box_box(halves[i], &poses[i], halves[j], &poses[j], margin).is_empty()
                    {
                        contacts += 1;
                        assert!(indices.binary_search(&j).is_ok(), "missing {i}, {j}");
                    }
                }
            }
            assert!(contacts > 100);
        }
    }

    #[test]
    fn sparse_boxes_are_pruned_but_unknown_and_invalid_bounds_fall_back() {
        let bounds = [
            Some((Vec3::ZERO, 0.1)),
            None,
            Some((Vec3::X * 100.0, 0.1)),
            Some((Vec3::ZERO, f64::NAN)),
        ];
        let pairs = candidates(&bounds, 0.02);
        assert_eq!(pairs, vec![Some(vec![1, 3]), None, Some(vec![1, 3]), None]);
        assert!(candidates(&bounds, f64::NAN).iter().all(Option::is_none));
    }

    #[test]
    fn margin_boundary_and_large_coordinates_are_retained() {
        for origin in [0.0, 1e8] {
            let half = Vec3::splat(0.5);
            let bounds = [
                Some((Vec3::X * origin, half.length())),
                Some((Vec3::X * (origin + 1.125), half.length())),
            ];
            assert_eq!(
                candidates(&bounds, 0.125),
                vec![Some(vec![1]), Some(vec![0])]
            );
        }
    }
}
