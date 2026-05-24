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
**Consequences**: C++ estimator layer will depend on Ceres.

---

## ADR-003: Primary observation model for streaming estimation

**Status**: Open

**Context**: The recursive estimator needs at least one observation type at runtime. Choice affects data requirements and failure modes.

**Options**:
1. **Motion-based hand-eye** — independently estimated ego-motion from each sensor, align trajectories. Well-conditioned with diverse motion, no scene assumptions. Degenerate under constant velocity, requires reliable per-sensor odometry.
2. **Edge-based** — align image edges with LiDAR depth discontinuities (Levinson & Thrun 2013). Dense, fast. Scene-content sensitive, fails on textureless scenes.
3. **Mutual information** (Pandey et al. 2012) — image gradients vs LiDAR reflectivity. Robust to scene type. Slow to converge, more complex.
4. **Hybrid** — multiple observation types combined via factor weights.

**Decision**: Pending. Recommendation for V1: motion-based, then add edge-based in V2.

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
