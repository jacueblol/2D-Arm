use glam::DVec3;

/// Joint angles for one arm configuration in the 2-link planar solve.
/// `theta1`: link1 angle from horizontal; `theta2`: link2 angle relative to
/// link1.
#[derive(Clone, Copy, Debug)]
pub struct IkSolution {
    pub theta1: f64,
    pub theta2: f64,
}

pub struct IkSolutions {
    /// theta2 > 0 — elbow bends upward relative to the reach line.
    pub elbow_pos: IkSolution,
    /// theta2 < 0 — elbow bends downward relative to the reach line.
    pub elbow_neg: IkSolution,
}

/// Which of the two IK solutions to use — a 2-link arm can generally reach
/// a given point with the elbow bent either up or down.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ElbowConfig {
    /// Elbow bends downward. The only configuration the old prototype ever
    /// used — kept as the default so existing behavior is unchanged.
    #[default]
    Down,
    /// Elbow bends upward.
    Up,
}

impl IkSolutions {
    pub fn pick(&self, config: ElbowConfig) -> IkSolution {
        match config {
            ElbowConfig::Down => self.elbow_neg,
            ElbowConfig::Up => self.elbow_pos,
        }
    }
}

/// Closed-form 2-link planar IK via the law of cosines.
///
/// Returns both elbow configurations, or `None` if `(px, py)` is outside
/// the reachable annulus `[|l1 - l2|, l1 + l2]`.
pub fn solve(l1: f64, l2: f64, px: f64, py: f64) -> Option<IkSolutions> {
    let cos_t2 = (px * px + py * py - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);

    // Values outside [-1, 1] mean the target is unreachable.
    if !(-1.0..=1.0).contains(&cos_t2) {
        return None;
    }

    let t2_pos = cos_t2.acos();
    let t2_neg = -t2_pos;

    let phi = py.atan2(px);

    let theta1_for = |t2: f64| -> f64 {
        let alpha = (l2 * t2.sin()).atan2(l1 + l2 * t2.cos());
        phi - alpha
    };

    Some(IkSolutions {
        elbow_pos: IkSolution {
            theta1: theta1_for(t2_pos),
            theta2: t2_pos,
        },
        elbow_neg: IkSolution {
            theta1: theta1_for(t2_neg),
            theta2: t2_neg,
        },
    })
}

/// Decompose a 3D target into a pan angle plus a planar `(r, h)` problem
/// (pan faces the target horizontally; the 2-link solve happens in the
/// vertical plane the pan direction defines), then pick a solution per
/// `elbow`. Returns `(pan_rad, theta1, theta2)`, or `None` if unreachable.
pub fn solve_3d(l1: f64, l2: f64, target: DVec3, elbow: ElbowConfig) -> Option<(f64, f64, f64)> {
    let pan = target.z.atan2(target.x);
    let r = (target.x * target.x + target.z * target.z).sqrt();
    let h = target.y;

    let solutions = solve(l1, l2, r, h)?;
    let sol = solutions.pick(elbow);
    Some((pan, sol.theta1, sol.theta2))
}

#[cfg(test)]
mod tests {
    use super::super::fk::forward_kinematics;
    use super::*;

    #[test]
    fn unreachable_target_returns_none() {
        // l1=1, l2=1: max reach 2.0, target at distance 3.0.
        assert!(solve(1.0, 1.0, 3.0, 0.0).is_none());
    }

    #[test]
    fn full_extension_matches_known_solution() {
        // Both links straight along +x: target at (l1+l2, 0). theta2 should
        // be 0 (fully extended), theta1 should be 0.
        let solutions = solve(1.0, 0.8, 1.8, 0.0).unwrap();
        assert!(solutions.elbow_neg.theta2.abs() < 1e-9);
        assert!(solutions.elbow_neg.theta1.abs() < 1e-9);
    }

    #[test]
    fn elbow_pos_and_elbow_neg_are_mirror_solutions() {
        let solutions = solve(1.0, 0.8, 1.2, 0.5).unwrap();
        assert!((solutions.elbow_pos.theta2 + solutions.elbow_neg.theta2).abs() < 1e-12);
    }

    /// The property the elbow-up feature exists to prove: FK(IK(target))
    /// reproduces `target`, for both elbow configurations, across a sample
    /// of the reachable planar workspace.
    #[test]
    fn planar_fk_ik_round_trip_both_elbow_configs() {
        let l1: f64 = 1.0;
        let l2: f64 = 0.8;
        let max_reach = l1 + l2;
        let min_reach = (l1 - l2).abs();

        for i in 0..12 {
            let angle = i as f64 * std::f64::consts::TAU / 12.0;
            let r = (min_reach + max_reach) / 2.0; // safely inside the annulus
            let px = r * angle.cos();
            let py = r * angle.sin();

            let solutions = solve(l1, l2, px, py).expect("point should be reachable");

            for (name, sol) in [
                ("elbow_pos", solutions.elbow_pos),
                ("elbow_neg", solutions.elbow_neg),
            ] {
                // Reuse forward_kinematics with pan=0 so it degenerates to
                // the same planar geometry solve() was derived for: x=px, z=0.
                let [_, _, _, ee] = forward_kinematics(0.0, sol.theta1, sol.theta2, l1, l2);
                let err = ((ee.x - px).powi(2) + (ee.z - 0.0).powi(2)).sqrt();
                assert!(
                    err < 1e-9,
                    "{name} round-trip failed at angle {angle:.2}: target=({px:.4},{py:.4}), got ee.x={:.4}, err={err:.2e}",
                    ee.x
                );
                assert!(
                    (ee.y - py).abs() < 1e-9,
                    "{name} height mismatch: expected {py:.4}, got {:.4}",
                    ee.y
                );
            }
        }
    }

    /// Same round-trip property through the full 3D dispatch (pan + planar).
    #[test]
    fn solve_3d_fk_ik_round_trip() {
        let l1 = 1.0;
        let l2 = 0.8;
        let targets = [
            DVec3::new(1.2, 0.5, 0.3),
            DVec3::new(0.6, 1.0, -0.4),
            DVec3::new(-0.5, 0.2, 0.9),
            DVec3::new(0.9, -0.3, 0.2),
        ];

        for target in targets {
            for elbow in [ElbowConfig::Down, ElbowConfig::Up] {
                let (pan, t1, t2) = solve_3d(l1, l2, target, elbow)
                    .unwrap_or_else(|| panic!("{target:?} should be reachable"));
                let [_, _, _, ee] = forward_kinematics(pan, t1, t2, l1, l2);
                let err = (ee - target).length();
                assert!(
                    err < 1e-9,
                    "{elbow:?} round-trip failed for {target:?}: got {ee:?}, err={err:.2e}"
                );
            }
        }
    }

    #[test]
    fn solve_3d_unreachable_returns_none() {
        assert!(solve_3d(1.0, 0.8, DVec3::new(10.0, 10.0, 10.0), ElbowConfig::Down).is_none());
    }
}
