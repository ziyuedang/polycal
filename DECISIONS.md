# DECISIONS.md

Architectural Decision Records (ADRs) for polycal.

When you encounter a decision point during a session, do NOT make it silently. Either follow an existing resolved ADR here, or add a new "Open" entry and flag it in `PROGRESS.md` for user resolution.

## Entry format

```
## ADR-NNN: <Title>

**Status**: Open | Resolved (YYYY-MM-DD)

**Context**: <why this needs deciding>

**Options**:
1. <Option A> — <tradeoffs>
2. <Option B> — <tradeoffs>

**Decision**: <if resolved>
**Rationale**: <if resolved>
**Consequences**: <what this enables / forecloses>
```

---

## ADR-001: Recursive estimator architecture

**Status**: Resolved (2026-05-23)

**Context**: Polycal needs a stateful estimator over the 6-DOF extrinsic. Choice has downstream consequences for library selection, code structure, and how covariance is handled.

**Options**:
1. **Continuous EKF** — lightweight, naturally recursive, covariance directly available, deployable on embedded hardware. Downsides: linearization error accumulates with no re-linearization of past states; classical overconfidence in reported covariance.
2. **Sliding-window factor graph** — re-linearizes within window, naturally extends to batch refinement, well-studied in SLAM. Downsides: heavier compute, covariance via marginalization is non-trivial, requires GTSAM or custom Ceres setup.

**Decision**: EKF (Extended Kalman Filter).
**Rationale**: Sufficient for slow thermal/mechanical drift operating regime. Lower complexity than factor graph. Linearization error is not a meaningful limitation for polycal's use case. Precedent: Mishra & Saripalli 2022, Wang et al. 2025.

**Consequences**: Enables ADR-002 resolution. Factor graph upgrade path remains open if empirical results show linearization error is the bottleneck.

---

## ADR-002: Optimization library

**Status**: Resolved (2026-05-23)

**Context**: Need a nonlinear least squares / smoothing library.

**Options**:
1. **Ceres Solver** (Google) — general-purpose NLS, large user base, manual or autodiff Jacobians.
2. **GTSAM** (Georgia Tech) — factor graph native, ISAM2 for incremental smoothing, natural fit for sliding-window.
3. **g2o** — popular in older SLAM stacks, less actively maintained.

**Decision**: Ceres Solver.
**Rationale**: GTSAM's main advantage is native factor graph support, which is no longer needed given ADR-001. Ceres is sufficient for the optimization substeps polycal requires and has broader familiarity.
**Consequences**: C++ estimator layer will depend on Ceres. Core EKF layer uses Eigen + Sophus only. Ceres is an optional dependency for initialization and front-end refinement, introduced when ADR-003 defines a concrete nonlinear subproblem requiring it. Do not add Ceres as a required dependency until that subproblem is specified.

---

## ADR-003: Primary observation model for streaming estimation

**Status**: Resolved (2026-05-23)

**Context**: The recursive estimator needs at least one observation type at runtime. Choice affects data requirements and failure modes.

**Options**:
1. **Motion-based hand-eye** — independently estimated ego-motion from each sensor, align trajectories. Well-conditioned with diverse motion, no scene assumptions. Degenerate under constant velocity, requires reliable per-sensor odometry.
2. **Edge-based** — align image edges with LiDAR depth discontinuities (Levinson & Thrun 2013). Dense, fast. Scene-content sensitive, fails on textureless scenes.
3. **Mutual information** (Pandey et al. 2012) — image gradients vs LiDAR reflectivity. Robust to scene type. Slow to converge, more complex.
4. **Hybrid** — multiple observation types combined via factor weights.

**Decision**: Motion-based hand-eye calibration for Phase 1. Edge-based added in Phase 2.
**Rationale**:
- Current synthetic generator already emits per-sensor trajectories and correspondences. Motion-based slots in directly without generator changes.
- Cleaner measurement Jacobian for first EKF implementation — easier to validate correctness on synthetic data.
- Edge-based requires a real image edge detector and LiDAR discontinuity front-end, which cannot be validated on the current synthetic output. Premature before estimator is validated.
- Precedent: Mishra & Saripalli 2022, Wang et al. 2025 both use motion-based for EKF online calibration.
- Degenerate cases (constant velocity) are handled by motion excitation requirements documented in data collection guidelines, not by switching observation models.

**Consequences**:
- Phase 1 synthetic generator extension needed: add per-sensor odometry trajectory output (camera visual odometry, LiDAR ICP odometry) with configurable noise and degeneracy injection.
- Measurement Jacobian is the relative motion constraint H: SE(3) x SE(3) -> SE(3), linearized around current estimate.
- Edge-based remains the Phase 2 target. Hybrid is the long-term architecture.
- ADR-003 resolution unblocks Phase 1 estimator implementation.

---

## ADR-004: Benchmark dataset selection

**Status**: Open

**Context**: For direct comparison vs Cocheteux et al., dataset choice matters.

**Options**:
1. **KITTI only** — matches Cocheteux's primary dataset, classical reference.
2. **KITTI + nuScenes** — broader validation, more diverse scenes.
3. **KITTI + DSEC** — matches Cocheteux's secondary dataset (event camera + LiDAR; less relevant unless event-camera support is in scope).

**Decision**: Pending. Recommendation: start KITTI, add others if time permits.

---

## ADR-005: Drift detection statistic

**Status**: Open

**Context**: For Layer 2 (drift detection), need a change-point statistic on innovations.

**Options**:
1. **CUSUM** — classical, controllable false-alarm rate, assumes known pre/post-change distributions.
2. **SPRT** — sequential probability ratio test, optimal under simple hypotheses.
3. **χ² gating on innovations** — simplest, more an outlier filter than a change-point detector.
4. **Combined**: χ² gating for outlier rejection + CUSUM on standardized innovations for drift.

**Decision**: Pending. Recommendation: Option 4.

---

## ADR-006: Coverage recalibration method

**Status**: Open

**Context**: If empirical PICP deviates from target, polycal needs a recalibration step (the conformal-prediction-flavored novelty of the project).

**Options**:
1. **Scalar inflation factor** — multiply nominal σ by empirical factor s.t. coverage matches target. Simple, may overcorrect on tails.
2. **Per-DOF inflation** — independent factor per X/Y/Z/Roll/Pitch/Yaw. More flexible, more params.
3. **Distribution-free quantile** — fit empirical quantile of normalized residuals, conformal-style.
4. **Variance recalibration via auxiliary regression** — learn σ correction as function of state.

**Decision**: Pending. Recommendation: start with Option 3 (conformal-style on residuals), Option 1 as ablation baseline.

---

## ADR-007: SE(3) frame conventions and perturbation side

**Status**: Resolved (2026-05-23)

**Context**: Before any C++ estimator code is written, the exact geometric conventions must be documented and used consistently everywhere. Ambiguity here causes silent sign errors that are extremely hard to debug.

**Decisions needed**:
- T_lc direction: is T_lc LiDAR-to-camera or camera-to-LiDAR? Must be consistent across synthetic.py, metrics.py, and estimator.
- Perturbation side: left perturbation (T_lc * exp(δ)) or right perturbation (exp(δ) * T_lc)?
- Euler angle order for reporting: matches Cocheteux Fig. 2 — X/Y/Z translation, then Roll/Pitch/Yaw rotation in LiDAR frame.
- Rotation representation in EKF state: rotation vector (so(3)) or quaternion?

**Decision**:
1. T_lc direction: LiDAR-to-camera. p_c = T_lc * p_l. Consistent everywhere: synthetic.py, metrics.py, estimator.

2. Perturbation side: right/local perturbation.
   T_lc <- T_lc * Exp(δ)
   δ is a 6D tangent vector in the LiDAR body frame.
   Do NOT use left perturbation. Do NOT use the term "left perturbation" anywhere in the codebase.

3. Reporting order: [X_cm, Y_cm, Z_cm, Roll_deg, Pitch_deg, Yaw_deg]. Matches Cocheteux Fig. 2 and existing metrics.py DOF order.

4. EKF state error: 6D tangent vector [rho ∈ R³, phi ∈ R³].
   rho: translation perturbation in meters.
   phi: rotation perturbation as so(3) rotation vector in radians.
   Internal units are always meters and radians.
   Convert to cm and degrees ONLY at the reporting layer (when calling picp, mpiw, interval_score).

5. Exp/Log maps: use Sophus SE3d::exp() and SE3d::log() exclusively. No custom Lie group implementations anywhere in the codebase.

**Rationale**: Right/local perturbation is preferred over left because the hand-eye measurement constraint is naturally expressed in the sensor body frame, yielding a cleaner Jacobian. Convention follows Sola et al. "A micro Lie theory" (2018).

**Consequences**:
- synthetic.py must be audited to confirm T_lc convention matches p_c = T_lc * p_l before Phase 1 estimator begins.
- All C++ estimator code uses Sophus SE3d. No raw rotation matrices or quaternions in the EKF state.
- Unit conversion happens at API boundary only.

---
