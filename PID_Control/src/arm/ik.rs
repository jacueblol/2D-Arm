/// Joint angles (theta1_rad, theta2_rad) for one arm configuration.
/// theta1: link1 angle from horizontal; theta2: link2 angle relative to link1.
pub struct IkSolution {
    pub theta1: f64,
    pub theta2: f64,
}

#[allow(dead_code)]
pub struct IkSolutions {
    /// theta2 > 0 — elbow bends upward relative to the reach line
    pub elbow_pos: IkSolution,
    /// theta2 < 0 — elbow bends downward relative to the reach line
    pub elbow_neg: IkSolution,
}

/// Closed-form 2-link planar IK via law of cosines.
///
/// Returns both elbow configurations, or None if (px, py) is outside
/// the reachable annulus [|l1 - l2|, l1 + l2].
pub fn solve(l1: f64, l2: f64, px: f64, py: f64) -> Option<IkSolutions> {
    let cos_t2 = (px * px + py * py - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);

    // Values outside [-1, 1] mean the target is unreachable
    if cos_t2 < -1.0 || cos_t2 > 1.0 {
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
        elbow_pos: IkSolution { theta1: theta1_for(t2_pos), theta2: t2_pos },
        elbow_neg: IkSolution { theta1: theta1_for(t2_neg), theta2: t2_neg },
    })
}
