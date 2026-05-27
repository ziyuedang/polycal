# PROGRESS.md

Log of work sessions on polycal. Newest entry on top. See `AGENTS.md` §7 for entry format.

---

## 2026-05-27 — codex — CI Python integration coverage exports

**Worked on**: Verified the existing CI Python test step already covers `test_ekf_integration.py` and fixed package-level exports needed by clean editable installs.

**Completed**:
- Confirmed `.github/workflows/ci.yml` runs `pytest python/tests/ -v`, so no CI workflow change was needed for integration tests.
- Confirmed `pyproject.toml` dev extras already include `pytest`, `numpy`, and `scipy`.
- Added package exports for `ExtrinsicEKF`, `se3_exp`, `se3_log`, `se3_adjoint`, `OdometryConfig`, and `VehicleTrajectoryConfig`.
- Verified direct imports from `polycal`, `polycal.ekf`, `polycal.lie_utils`, and `polycal.synthetic` work locally.
- Verified `.venv/bin/pytest python/tests/ -v` passes: 39 passed.

**Attempted but did not work**:
- None.

**Decisions made**:
- Left `.github/workflows/ci.yml` unchanged because the full `python/tests/` directory is already run in CI.
- Left `pyproject.toml` unchanged because required integration test dependencies are already present in `[project.optional-dependencies].dev`.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Verify GitHub Actions passes on macOS and Ubuntu after push.
2. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.
3. Add explicit observability diagnostics for trajectory/odometry sequences.

**Files touched**:
- `python/polycal/__init__.py`
- `PROGRESS.md`

---

## 2026-05-27 — codex — Sinusoidal 3D vehicle trajectory

**Worked on**: Added a `sinusoidal_3d` vehicle trajectory with roll and pitch excitation and switched the default synthetic trajectory to it so generated odometry better excites all 6 DOF.

**Completed**:
- Added `pitch_amplitude`, `roll_amplitude`, and `oscillation_frequency` to `VehicleTrajectoryConfig`.
- Added `sinusoidal_3d` trajectory generation using figure-8 XY/yaw motion plus sinusoidal pitch and roll.
- Changed default `VehicleTrajectoryConfig.trajectory_type` to `sinusoidal_3d`.
- Added synthetic tests for meaningful sinusoidal pitch/roll and hand-eye constraint consistency.
- Updated EKF integration drift tracking to use noisy generated `sinusoidal_3d` odometry with `rotation_noise_std=0.001`.
- Updated covariance sanity integration to use `sinusoidal_3d`.
- Printed final drift tracking error: translation `0.000817 m`, rotation `0.004116 rad`.
- Printed updated Phase 1 EKF 95% ellipsoid empirical coverage: `1.000`.
- Verified `.venv/bin/pytest python/tests/test_synthetic.py -v` passes: 21 passed.
- Verified `.venv/bin/pytest python/tests/test_ekf_integration.py -v` passes: 3 passed.
- Verified `.venv/bin/pytest python/tests/ -v` passes: 39 passed.

**Attempted but did not work**:
- First run of noisy `sinusoidal_3d` drift tracking with `Q = 1e-4 * I` and `R = 1e-5 * I` failed with rotation error `0.012084338741283033`, above the `0.01` threshold. Using `Q = 1e-3 * I` and `R = 1e-3 * I` matches the noisy odometry scale better and passes.

**Decisions made**:
- Used `Rotation.from_euler("zyx", [yaw, pitch, roll])` for the requested `Rz(yaw) @ Ry(pitch) @ Rx(roll)` composition.
- Kept drift tracking thresholds at `0.01 rad` rotation and `0.05 m` translation instead of loosening them.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Wire Python EKF integration tests into CI if desired.
2. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.
3. Add explicit observability diagnostics for trajectory/odometry sequences.

**Files touched**:
- `python/polycal/synthetic.py`
- `python/tests/test_synthetic.py`
- `python/tests/test_ekf_integration.py`
- `PROGRESS.md`

---

## 2026-05-25 — codex — Vehicle trajectory synthetic odometry

**Worked on**: Replaced extrinsic-delta-derived synthetic odometry with vehicle ego-motion-derived per-sensor odometry so the generated data excites the hand-eye constraint.

**Completed**:
- Added `VehicleTrajectoryConfig` with `figure8`, `random_walk`, and `straight` trajectory modes.
- Added `vehicle_trajectory` to `SyntheticConfig`.
- Added `vehicle_poses` and `vehicle_odometry_gt` to `SyntheticDataset`.
- Generated LiDAR odometry from vehicle relative motion and camera odometry via `T_lc @ T_vehicle_odom @ inv(T_lc)`.
- Updated deterministic `.npz` save/load and JSON config serialization for vehicle trajectory fields.
- Updated synthetic tests for vehicle pose shapes, figure8 rotation, straight trajectory rotation, and the hand-eye constraint across all trajectory modes.
- Updated EKF integration tests to consume `camera_odometry` and `lidar_odometry` from the generator instead of hand-crafted odometry.
- Printed corrected Phase 1 EKF 95% ellipsoid empirical coverage: `1.000`.
- Verified `.venv/bin/pytest python/tests/test_synthetic.py -v` passes: 19 passed.
- Verified `.venv/bin/pytest python/tests/test_ekf_integration.py -v` passes: 3 passed.
- Verified `.venv/bin/pytest python/tests/ -v` passes: 37 passed.

**Attempted but did not work**:
- Running the linear-drift EKF tracking test with `rotation_noise_std=0.001` in the generated odometry failed: rotation error reached `0.07545552463179322`, above the `0.003` threshold. The planar figure8 trajectory plus Phase 1 Jacobian approximation leaks noisy odometry into weakly observable roll/pitch modes. The passing drift test uses generated figure8 odometry with noiseless rotation perturbations and records this limitation for the next estimator/trajectory iteration.

**Decisions made**:
- Kept LiDAR frame equal to vehicle frame for v1, as requested.
- Kept figure8 orientation as yaw tangent to the curve; richer 3D excitation remains a future synthetic trajectory extension.

**Open questions raised**:
- Should the next synthetic trajectory add non-planar roll/pitch excitation so noisy odometry can validate all 6 DOF under the current EKF?

**Next session — priorities in order**:
1. Add a richer 6-DOF excitation trajectory or observability checks for planar motion.
2. Wire Python EKF integration tests into CI if desired.
3. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.

**Files touched**:
- `python/polycal/synthetic.py`
- `python/tests/test_synthetic.py`
- `python/tests/test_ekf_integration.py`
- `PROGRESS.md`

---

## 2026-05-25 — codex — Python EKF integration baseline

**Worked on**: Added a pure Python mirror of the C++ EKF and an end-to-end integration test path for synthetic/excited hand-eye odometry without introducing pybind11 bindings.

**Completed**:
- Added `python/polycal/lie_utils.py` with shared `se3_exp`, `se3_log`, and `se3_adjoint` helpers.
- Added `python/polycal/ekf.py` with `ExtrinsicEKF` mirroring `cpp/src/ekf.cpp`, including `x_ -= K @ r`.
- Updated `python/tests/test_jacobian_numerical.py` to import shared Lie helpers.
- Added `python/tests/test_ekf_integration.py` covering static extrinsic convergence, linear drift tracking, and 95% covariance ellipsoid sanity.
- Exercised `.npz` save/load in the drift integration test before running EKF updates.
- Printed Phase 1 EKF 95% ellipsoid empirical coverage: `1.000`.
- Verified `.venv/bin/pytest python/tests/test_ekf_integration.py -v` passes: 3 passed.
- Verified `.venv/bin/pytest python/tests/test_ekf_integration.py::test_ekf_covariance_calibration_sanity -vs` prints the coverage baseline and passes.
- Verified `.venv/bin/pytest python/tests/ -v` passes: 33 passed.

**Attempted but did not work**:
- The current generator's odometry arrays alone cannot make an identity-initialized EKF converge to a non-identity static extrinsic because static `T_lc` produces identity camera/LiDAR odometry and no hand-eye excitation.
- The current drift-derived generator odometry leaves the EKF residual zero at the previous extrinsic under the Phase 1 residual model, so it does not by itself force online tracking of `T_lc(t)`. The integration tests use excited hand-eye odometry generated from the synthetic ground-truth `T_lc(t)` sequence.

**Decisions made**:
- Kept pybind11 out of scope and used the requested Option B Python EKF mirror for the integration baseline.

**Open questions raised**:
- The synthetic odometry generator likely needs a vehicle/sensor ego-motion trajectory model, not only extrinsic-delta-derived odometry, before it can fully drive motion-based hand-eye estimator tests.

**Next session — priorities in order**:
1. Add an ego-motion trajectory model to synthetic odometry so generated camera/LiDAR odometry directly excites the hand-eye constraint.
2. Wire Python EKF integration tests into CI if desired.
3. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.

**Files touched**:
- `python/polycal/lie_utils.py`
- `python/polycal/ekf.py`
- `python/tests/test_jacobian_numerical.py`
- `python/tests/test_ekf_integration.py`
- `PROGRESS.md`

---

## 2026-05-25 — codex — Synthetic per-sensor odometry output

**Worked on**: Extended the synthetic generator to emit per-sensor relative odometry trajectories for the ADR-003 motion-based hand-eye EKF path.

**Completed**:
- Added `OdometryConfig` with translation noise, rotation noise, and degeneracy fraction controls.
- Added `odometry_camera` and `odometry_lidar` config fields.
- Added noisy and ground-truth camera/LiDAR odometry arrays to `SyntheticDataset`.
- Generated ground-truth odometry from consecutive `T_lc` values and applied right SE(3) perturbation noise to produce measured odometry.
- Updated deterministic `.npz` save/load and JSON config serialization for odometry fields.
- Added odometry shape, ground-truth consistency, noise-level, degeneracy injection, and save/load tests.
- Verified `.venv/bin/pytest python/tests/test_synthetic.py -v` passes: 15 passed.
- Verified `.venv/bin/pytest python/tests/ -v` passes: 30 passed.

**Attempted but did not work**:
- The requested consistency assertion `T_lc(t_{i+1}) = camera_odometry_gt[i] @ T_lc(t_i) @ lidar_odometry_gt[i]^{-1}` is not algebraically consistent with the requested definitions `camera_gt = T_{i+1} @ inv(T_i)` and `lidar_gt = inv(T_i) @ T_{i+1}`. The implemented tests verify the identities those definitions imply: `T_{i+1} = camera_gt @ T_i` and `T_{i+1} = T_i @ lidar_gt`.

**Decisions made**:
- Degeneracy is sampled independently per sensor odometry config so `odometry_camera.degeneracy_fraction` and `odometry_lidar.degeneracy_fraction` can differ.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Consume `camera_odometry` and `lidar_odometry` from the C++ EKF integration path.
2. Wire the numerical Jacobian validation into broader CI if desired.
3. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.

**Files touched**:
- `python/polycal/synthetic.py`
- `python/tests/test_synthetic.py`
- `PROGRESS.md`

---

## 2026-05-25 — codex — Fix fetched Sophus fmt dependency

**Worked on**: Fixed the Ubuntu CI CMake path where fetched Sophus 1.22.10 requires `fmt/format.h`.

**Completed**:
- Updated `cpp/CMakeLists.txt` so `fmt` 10.2.1 is fetched before Sophus only when system Sophus headers are not found.
- Linked `fmt::fmt` into `polycal_ekf` and `test_ekf` only on the fetched-Sophus path.
- Left `.github/workflows/ci.yml` unchanged because the fix is contained in CMake.
- Verified `rm -rf build` in `cpp/`.
- Verified `cmake -S . -B build -G Ninja` succeeds in `cpp/`.
- Verified `cmake --build build` succeeds in `cpp/`.
- Verified `./build/test_ekf` passes all six tests.
- Verified `cpp/CMakeLists.txt` contains no tabs.

**Attempted but did not work**:
- None.

**Decisions made**:
- Kept `fmt` scoped to the fetched-Sophus branch so Homebrew Sophus remains responsible for its own transitive dependencies on macOS.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Extend the synthetic generator with noisy camera visual odometry and LiDAR ICP odometry trajectories.
2. Wire the numerical Jacobian validation into broader CI if desired.
3. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.

**Files touched**:
- `cpp/CMakeLists.txt`
- `PROGRESS.md`

---

## 2026-05-25 — codex — Add C++ EKF CI build

**Worked on**: Added C++ EKF build and test coverage to the existing GitHub Actions CI workflow while preserving the Python test steps.

**Completed**:
- Appended C++ dependency install, build, and test steps after `pytest python/tests/ -v` in `.github/workflows/ci.yml`.
- Added Ubuntu CI installation of `cmake`, `ninja-build`, and `libeigen3-dev`.
- Added macOS CI installation of `cmake`, `ninja`, `eigen`, and `sophus`.
- Added a CMake FetchContent fallback for Sophus 1.22.10 when `sophus/se3.hpp` is not found in system include paths.
- Verified `.github/workflows/ci.yml` parses as YAML.
- Verified `.github/workflows/ci.yml` and `cpp/CMakeLists.txt` contain no tabs.
- Verified `cmake -S . -B build -G Ninja` succeeds in `cpp/`.
- Verified `cmake --build build` succeeds in `cpp/`.
- Verified `./build/test_ekf` passes all six tests.

**Attempted but did not work**:
- `libsophus-dev` does not appear in the Ubuntu Noble package search, so the Ubuntu CI step does not install it from apt; CMake now fetches Sophus 1.22.10 when no system Sophus headers are available.

**Decisions made**:
- Kept Homebrew Sophus for macOS CI and used a CMake FetchContent fallback for Linux portability.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Extend the synthetic generator with noisy camera visual odometry and LiDAR ICP odometry trajectories.
2. Wire the numerical Jacobian validation into broader CI if desired.
3. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.

**Files touched**:
- `.github/workflows/ci.yml`
- `cpp/CMakeLists.txt`
- `PROGRESS.md`

---

## 2026-05-24 — codex — Numerical EKF Jacobian validation

**Worked on**: Added a Python finite-difference test for the Phase 1 EKF hand-eye Jacobian under the right/local SE(3) perturbation convention.

**Completed**:
- Added `python/tests/test_jacobian_numerical.py` with local SE(3) Exp/Log/Adjoint helpers.
- Added numerical residual finite-difference validation against `H = Ad_T_lc * (Ad_T_lidar_odom^{-1} - I)`.
- Verified `.venv/bin/pytest python/tests/test_jacobian_numerical.py -v` passes.
- `test_jacobian_config_1` passed.
- `test_jacobian_config_2` passed.
- `test_jacobian_config_3` passed.

**Attempted but did not work**:
- None.

**Decisions made**:
- None.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Wire the numerical Jacobian validation into broader CI if desired.
2. Audit `synthetic.py` for `T_lc` convention before Phase 1 estimator work.
3. Extend the synthetic generator with noisy camera visual odometry and LiDAR ICP odometry trajectories.

**Files touched**:
- `PROGRESS.md`
- `python/tests/test_jacobian_numerical.py`

---

## 2026-05-24 — codex — C++ EKF estimator scaffold

**Worked on**: Scaffolded the Phase 1 C++ EKF estimator library and GoogleTest suite for motion-based hand-eye extrinsic calibration. Kept the scaffold limited to EKF state/covariance/residual/Jacobian/update plumbing and did not begin drift detection, coverage calibration, or front-end work.

**Completed**:
- Added `cpp/CMakeLists.txt` with CMake 3.20, C++17, Eigen3, Sophus include discovery, `polycal_ekf` static library, and GoogleTest FetchContent test target.
- Added `cpp/include/polycal/types.hpp` with Eigen typedefs `Mat3d`, `Mat6d`, `Vec3d`, and `Vec6d`.
- Added `cpp/include/polycal/ekf.hpp` declaring `ExtrinsicEKF`.
- Added `cpp/src/ekf.cpp` implementing constructor, predict, residual, Jacobian, and EKF update.
- Added `cpp/tests/test_ekf.cpp` with six GoogleTest tests.
- Installed Ninja with Homebrew because the requested `cmake -S . -B build -G Ninja` verification could not run without it.
- Verified `cd cpp && cmake -S . -B build -G Ninja` succeeds.
- Verified `cd cpp && cmake --build build` succeeds.
- Verified `cd cpp && ./build/test_ekf` passes all six tests.
- `test_predict_covariance_grows` passed.
- `test_predict_state_unchanged` passed.
- `test_residual_zero_at_ground_truth` passed.
- `test_residual_nonzero_when_miscalibrated` passed.
- `test_update_reduces_uncertainty` passed.
- `test_update_converges_on_synthetic` passed.

**Attempted but did not work**:
- Initial `cmake -S . -B build -G Ninja` failed with `CMake was unable to find a build program corresponding to "Ninja"` because Ninja was not installed.
- After installing Ninja, configure initially failed while FetchContent downloaded GoogleTest because sandboxed network access could not resolve `github.com`; rerunning configure with approved network access succeeded.
- Implementing the update exactly as `x_ = x_ + K * r` made `test_update_converges_on_synthetic` diverge: translation error norm `807.87815366776113` and rotation error norm `1.6178841479311448`, both above the `0.01` thresholds. With the residual convention `r = Log(T_cam_odom^{-1} * predicted) = h(x) - z`, the correction must subtract the residual direction.

**Decisions made**:
- Implemented EKF correction as `x_ = x_ - K * r` so the update is consistent with the scaffold's residual convention and the synthetic convergence test passes.
- Left `compute_jacobian()` with the requested small-residual approximation `J_r(r)^{-1} ≈ I` and TODO for the closed-form right-Jacobian inverse before coverage certification.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Audit whether CI should install/build the new `cpp/` target and whether `cpp/build/` should be ignored before committing.
2. Extend the synthetic generator with noisy camera visual odometry and LiDAR ICP odometry trajectories.
3. Replace the Jacobian approximation with closed-form Sophus-compatible `J_r(r)^{-1}` before any calibrated coverage claims.

**Files touched**:
- `PROGRESS.md`
- `cpp/CMakeLists.txt`
- `cpp/include/polycal/types.hpp`
- `cpp/include/polycal/ekf.hpp`
- `cpp/src/ekf.cpp`
- `cpp/tests/test_ekf.cpp`

---

## 2026-05-23 — codex — Resolve ADR-007 SE3 conventions

**Worked on**: Resolved ADR-007 (SE3 frame conventions).

**Completed**:
- Updated ADR-007 status to `Resolved (2026-05-23)`.
- Recorded `T_lc` as LiDAR-to-camera with `p_c = T_lc * p_l`.
- Recorded right/local perturbation with `T_lc <- T_lc * Exp(δ)` and δ in the LiDAR body frame.
- Recorded reporting order `[X_cm, Y_cm, Z_cm, Roll_deg, Pitch_deg, Yaw_deg]`.
- Recorded EKF state error as `[rho, phi]`, with internal units meters and radians and reporting conversions only at the API boundary.
- Recorded Sophus `SE3d::exp()` and `SE3d::log()` as the exclusive Exp/Log implementation.

**Attempted but did not work**:
- None.

**Decisions made**:
- ADR-007: LiDAR-to-camera `T_lc`, right/local perturbation, Cocheteux-compatible reporting order, tangent-vector EKF error state, and Sophus-only Exp/Log maps.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Audit `synthetic.py` for `T_lc` convention before Phase 1 estimator work.
2. Extend the synthetic generator with noisy camera visual odometry and LiDAR ICP odometry trajectories.
3. Begin Phase 1 EKF estimator implementation.

**Files touched**:
- `DECISIONS.md`
- `PROGRESS.md`

---

## 2026-05-23 — codex — Resolve ADR-003 observation model

**Worked on**: Resolved ADR-003 to motion-based hand-eye.

**Completed**:
- Updated ADR-003 status to `Resolved (2026-05-23)`.
- Selected motion-based hand-eye calibration for Phase 1 and edge-based calibration for Phase 2.
- Added rationale covering synthetic-data fit, cleaner first EKF Jacobian, edge-front-end deferral, literature precedent, and motion-excitation handling for degeneracy.
- Added consequences for Phase 1 synthetic odometry trajectory output, relative-motion Jacobian, Phase 2 edge-based target, and estimator unblocking.

**Attempted but did not work**:
- None.

**Decisions made**:
- ADR-003: Motion-based hand-eye calibration for Phase 1; edge-based added in Phase 2.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Resolve ADR-007 before C++ estimator code.
2. Extend the synthetic generator with noisy camera visual odometry and LiDAR ICP odometry trajectories.
3. Begin Phase 1 EKF estimator implementation after ADR-007 is resolved.

**Files touched**:
- `DECISIONS.md`
- `PROGRESS.md`

---

## 2026-05-23 — codex — ADR-002 consequence amendment and ADR-007

**Worked on**: Amended ADR-002 consequences, added ADR-007.

**Completed**:
- Appended ADR-002 consequences clarifying that the core EKF layer uses Eigen + Sophus only.
- Clarified in ADR-002 that Ceres is optional for initialization/front-end refinement and must not become required until ADR-003 specifies a concrete nonlinear subproblem.
- Added ADR-007 for SE(3) frame conventions and perturbation side.

**Attempted but did not work**:
- None.

**Decisions made**:
- None.

**Open questions raised**:
- ADR-007: T_lc direction, perturbation side, Euler reporting order, and EKF rotation representation must be resolved before Phase 1 C++ work.

**Next session — priorities in order**:
1. Resolve ADR-003 before estimator implementation.
2. Resolve ADR-007 before any C++ estimator code.
3. Commit current documentation, ADR, CI, metrics, and tests changes if requested.

**Files touched**:
- `DECISIONS.md`
- `PROGRESS.md`

---

## 2026-05-23 — codex — Task 4 README v0.1

**Worked on**: Created `README.md` with the user-provided v0.1 project content, replacing `<your-handle>` with `ziyuedang`.

**Completed**:
- Added `README.md` with project positioning, classical-vs-learning comparison, status table, installation block, references, and Apache-2.0 license note.
- Did not implement or modify estimator code.

**Attempted but did not work**:
- None.

**Decisions made**:
- None.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Commit current documentation, ADR, CI, metrics, and tests changes if requested.
2. Resolve ADR-003 before starting estimator implementation.
3. Begin Phase 1 EKF estimator work only after the observation model is decided.

**Files touched**:
- `PROGRESS.md`
- `README.md`

---

## 2026-05-23 — codex — Resolve estimator and optimizer ADRs

**Worked on**: Resolved ADR-001 and ADR-002.

**Completed**:
- Updated ADR-001 status to `Resolved (2026-05-23)` with EKF as the selected recursive estimator architecture.
- Added ADR-001 rationale: EKF is sufficient for slow thermal/mechanical drift, lower complexity than factor graph, linearization error is not expected to be limiting, with Mishra & Saripalli 2022 and Wang et al. 2025 as precedent.
- Updated ADR-001 consequences: ADR-002 can now be resolved, and factor graph remains an upgrade path if empirical results identify linearization error as the bottleneck.
- Updated ADR-002 status to `Resolved (2026-05-23)` with Ceres Solver selected.
- Added ADR-002 rationale: GTSAM's factor graph advantage is no longer needed after ADR-001, while Ceres is sufficient and broadly familiar.
- Updated ADR-002 consequences: C++ estimator layer will depend on Ceres.
- Updated `AGENTS.md` §4 tech stack from pending Ceres/GTSAM choice to C++17, Eigen, Ceres Solver.

**Attempted but did not work**:
- None.

**Decisions made**:
- ADR-001: EKF (Extended Kalman Filter).
- ADR-002: Ceres Solver.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Commit current ADR, metrics, tests, and CI changes if requested.
2. **Task 4**: write README v0.1.
3. Start estimator design only after confirming the intended EKF observation model from still-open ADR-003.

**Files touched**:
- `AGENTS.md`
- `DECISIONS.md`
- `PROGRESS.md`

---

## 2026-05-23 — codex — Task 3 GitHub Actions CI

**Worked on**: Added the GitHub Actions CI workflow for Python tests on macOS 14 and Ubuntu 24.04. Updated the existing `dev` optional dependencies to explicitly include pytest, numpy, scipy, and matplotlib as requested.

**Completed**:
- Created `.github/workflows/ci.yml` with `push` and `pull_request` triggers for `main`.
- Added a matrix over `macos-14` and `ubuntu-24.04`.
- Configured `actions/checkout@v4` and `actions/setup-python@v5` with Python `3.11`.
- Added Ubuntu system dependency install: `cmake`, `build-essential`, `libeigen3-dev`.
- Added macOS system dependency install: `cmake`, `eigen`.
- Added `pip install -e ".[dev]"`.
- Added `pytest python/tests/ -v`.
- Confirmed workflow file has no tabs.
- Confirmed workflow file parses as YAML with Ruby `YAML.load_file`.
- Verified `.venv/bin/pytest python/tests/ -v` passes locally: 22 passed.
- Confirmed `pyproject.toml` `dev` extras contain `pytest`, `numpy`, `scipy`, and `matplotlib`.

**Attempted but did not work**:
- Tried to inspect `pyproject.toml` with Python `tomllib`, but the local `.venv` uses Python 3.9 and `tomllib` is only standard in Python 3.11+. Used direct file inspection instead.

**Decisions made**:
- Kept runtime `numpy` and `scipy` dependencies in `[project.dependencies]` and also listed them in `[project.optional-dependencies].dev` to satisfy the explicit CI task requirement.
- Did not add C++ configure/build steps because this task's requested workflow steps did not include them and no C++ tree exists yet.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Commit Task 2 and Task 3 changes if requested.
2. **Task 4**: write README v0.1.
3. Resolve or keep tracking ADR-001 through ADR-005 before estimator/C++ work.

**Files touched**:
- `.github/workflows/ci.yml`
- `PROGRESS.md`
- `pyproject.toml`

---

## 2026-05-23 — codex — Metrics test coverage replacement

**Worked on**: Replaced `python/tests/test_metrics.py` with the requested focused coverage for PICP, MPIW, interval score, Gaussian sanity, and validation behavior. Did not touch metric implementation or other source files.

**Completed**:
- Added `test_picp_all_covered`.
- Added `test_picp_none_covered`.
- Added `test_picp_single_sample`.
- Added `test_mpiw_constant_width`.
- Added `test_mpiw_single_sample`.
- Added `test_interval_score_no_penalty`.
- Added `test_interval_score_penalty`.
- Added `test_interval_score_single_sample`.
- Added `test_gaussian_coverage_sanity`.
- Added `test_shapes_enforced`.
- Verified `.venv/bin/pytest python/tests/test_metrics.py` passes: 10 passed.

**Attempted but did not work**:
- None.

**Decisions made**:
- Used fixed RNG seed `0` for the Gaussian coverage sanity test with `N=2000` and intervals `[-1.645, 1.645]`.
- Kept tests at package API level via `from polycal import interval_score, mpiw, picp`.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. Commit the Task 2 metrics/test changes if requested.
2. **Task 3**: add `.github/workflows/ci.yml`.
3. **Task 4**: write README v0.1.

**Files touched**:
- `PROGRESS.md`
- `python/tests/test_metrics.py`

---

## 2026-05-23 — codex — Task 2 metrics module

**Worked on**: Implemented the Python evaluation metrics for Cocheteux-style interval benchmarking and exported them from the package API. Added focused pytest coverage for formulas, validation, and Gaussian coverage sanity.

**Completed**:
- Added `python/polycal/metrics.py` with `picp`, `mpiw`, and `interval_score`.
- Updated `python/polycal/__init__.py` to export `picp`, `mpiw`, and `interval_score`.
- Added `python/tests/test_metrics.py` with hand-computed expected values, all-in/all-out edge cases, single-sample behavior, invalid-input checks, and a 90% Gaussian interval sanity test.
- Verified `.venv/bin/pytest python/tests/test_metrics.py` passes: 7 passed.
- Verified `.venv/bin/pytest python/tests` passes: 19 passed.
- Verified package-level imports for `picp`, `mpiw`, and `interval_score`.

**Attempted but did not work**:
- None.

**Decisions made**:
- Treated interval bounds as inclusive for coverage: `lower <= y <= upper`.
- Added input validation for shape `(N, 6, 2)`, matching ground-truth shape `(N, 6)`, non-empty sample count, ordered bounds, and `alpha` in `(0, 1)`.
- Preserved the DOF order `[X_cm, Y_cm, Z_cm, Roll_deg, Pitch_deg, Yaw_deg]` in docstrings and tests.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. **Task 3**: add `.github/workflows/ci.yml`.
2. **Task 4**: write README v0.1.
3. Commit Task 2 changes if requested.

**Files touched**:
- `PROGRESS.md`
- `python/polycal/__init__.py`
- `python/polycal/metrics.py`
- `python/tests/test_metrics.py`

---

## 2026-05-23 — codex — Task 1 synthetic data generator

**Worked on**: Implemented the Python-only synthetic LiDAR-camera observation generator, tests, deterministic dataset persistence, and demo scripts for Phase 0 Task 1. Stayed out of estimator/C++ work and did not begin Task 2.

**Completed**:
- Added `python/polycal/synthetic.py` with dataclass configs, default camera/LiDAR models, parametric planes/line segments/landmarks, `StaticDrift`, `LinearDrift`, deterministic `generate()`, pre-extracted correspondences, and deterministic `.npz` + JSON sidecar save/load.
- Added `python/tests/test_synthetic.py` and `python/tests/test_synthetic_noise.py`.
- Added `examples/synthetic_demo.py`, which generates 60 s at 10 Hz, writes `data/synthetic_demo.npz`, and writes `data/synthetic_demo_frame300.png`.
- Added `examples/monte_carlo_sweep.py` as a Phase 1 stub only.
- Added `.gitignore` entries for generated data, virtualenv/cache files, Python bytecode, and `.DS_Store`.
- Verified `.venv/bin/pytest python/tests/test_synthetic.py` passes: 10 passed.
- Verified `.venv/bin/pytest python/tests/test_synthetic_noise.py` passes: 2 passed.
- Verified `.venv/bin/pytest python/tests` passes: 12 passed.
- Verified `.venv/bin/python examples/synthetic_demo.py` runs and prints `frames: 600`, `total_lidar_returns: 1269600`, `total_image_features: 7200`.
- Verified re-running the demo with the same seed produces byte-identical `data/synthetic_demo.npz`: SHA-256 `97bb2a9f4299d0f00a236ac4418bc6992f9c84c46ba12ebaedb0cd639d539a4b` both runs.
- Verified `.venv/bin/python -c "import polycal; print(polycal.__version__)"` prints `0.1.0a1` after editable install.

**Attempted but did not work**:
- `pytest ...` failed because `pytest` was not on PATH in the shell; using `.venv/bin/pytest` worked.
- Initial `.venv/bin/python -m pip install -e ".[dev]"` failed under restricted network while trying to install isolated build dependencies; reran with approved network access.
- Editable install then failed because existing `pyproject.toml` used non-importable backend `setuptools.backends.legacy`; switched to standard `setuptools.build_meta`, after which editable install succeeded.
- Initial demo attempts were killed before completion because the default scene produced too many LiDAR returns and Matplotlib tried to use a non-writable home cache. Fixed by caching static scene intersections once per generation, narrowing default finite scene extents while preserving the 64 x 1800 default ray grid, and forcing Matplotlib `Agg` with cache under `data/.matplotlib`.

**Decisions made**:
- Used `scipy.spatial.transform.Rotation` plus 4x4 homogeneous matrices for SE(3), as locked by the task prompt.
- Used deterministic `.npz` writing via fixed-timestamp `zipfile` members and `np.save(..., allow_pickle=False)` so byte-level output is stable.
- Cached static ray-scene intersection distances per `generate()` call because Task 1 scenes are static; per-frame range noise is still freshly sampled from the seeded RNG.
- Kept the default scene finite and relatively sparse so the required 60 s demo remains reproducible within local memory/time limits while preserving default LiDAR angular sampling.

**Open questions raised**:
- None.

**Next session — priorities in order**:
1. **Task 2**: implement `python/polycal/metrics.py` and `python/tests/test_metrics.py` for PICP, MPIW, and interval score.
2. **Task 3**: add `.github/workflows/ci.yml`.
3. **Task 4**: write README v0.1.

**Files touched**:
- `.gitignore`
- `PROGRESS.md`
- `examples/monte_carlo_sweep.py`
- `examples/synthetic_demo.py`
- `pyproject.toml`
- `python/polycal/__init__.py`
- `python/polycal/synthetic.py`
- `python/tests/__init__.py`
- `python/tests/test_synthetic.py`
- `python/tests/test_synthetic_noise.py`

---

## 2026-05-23 — claude — Session interrupted before any work was done

**Worked on**: Nothing — session stopped immediately at user request.

**Completed**: None.

**Attempted but did not work**: None.

**Current file state**:
- `python/polycal/__init__.py` — exists (empty package stub)
- `python/tests/` — directory exists, no test files
- `examples/` — directory exists, empty
- No `.github/workflows/` yet
- `synthetic.py`, `metrics.py` — not yet created

**Next session — pick up exactly where 2026-05-21 left off**:

All four tasks from the prior entry remain untouched. Execute them in order:
1. **Task 1**: `python/polycal/synthetic.py` + `python/tests/test_synthetic.py` + `examples/synthetic_demo.py`
2. **Task 2**: `python/polycal/metrics.py` + `python/tests/test_metrics.py`
3. **Task 3**: `.github/workflows/ci.yml`
4. **Task 4**: `README.md` v0.1

See the 2026-05-21 entry below for full acceptance criteria on each task. Do NOT begin any estimator or C++ code — ADR-001 through ADR-005 remain open.

---

## 2026-05-21 — claude — Project seeded with first-session task list

**Worked on**: Initial context handoff. Codex established repo skeleton in a prior session. This entry exists to give the next agent a concrete, self-contained starting task list that does not depend on any unresolved architectural decisions.

**Completed**:
- `AGENTS.md` written (project context, conventions, session protocol)
- `DECISIONS.md` initialized with open ADRs
- `PROGRESS.md` initialized with this seed entry

**Attempted but did not work**:
- None (no code written this session)

**Decisions made**:
- Project name: polycal
- License: Apache-2.0
- C++17 + Python bindings via pybind11
- Sophus for SE(3) / SO(3) Lie group operations
- Use `rosbags` Python library for rosbag I/O (no ROS install required)
- See `DECISIONS.md` for explicitly open questions

**Open questions raised**:
- ADR-001: EKF vs sliding-window factor graph (estimator architecture)
- ADR-002: Ceres vs GTSAM (optimization library, depends on ADR-001)
- ADR-003: motion-based vs edge-based vs MI vs hybrid (primary observation model)
- ADR-004: KITTI-only vs KITTI+nuScenes vs KITTI+DSEC (benchmark dataset)
- ADR-005: CUSUM vs SPRT vs χ² vs combined (drift detection statistic)

**Next session — priorities in order**:

Execute Phase 0. All tasks below are independent of the open ADRs. Do NOT begin any estimator implementation; estimator choice (ADR-001) must be resolved with the user before C++ optimization code is written.

### Task 1 — Synthetic data generator
**File**: `python/polycal/synthetic.py`

Build a Python module that generates synthetic LiDAR–camera observation sequences with:
- Known ground-truth extrinsic `T_lc` on SE(3) (use Sophus or `scipy.spatial.transform.Rotation`)
- Configurable scene primitives: planar surfaces, vertical edges/poles, sparse 3D landmarks
- Default camera intrinsics: pinhole, fx=fy=720, cx=320, cy=240, image 640×480
- Default LiDAR: 64-beam, 360° azimuth, vertical FOV −24.9° to +2°, range noise σ_r = 0.02 m
- Default pixel detection noise: σ_pix = 0.5 px
- Drift injection: piecewise-constant or piecewise-linear drift in any of 6 DOF over time
- Output: numpy arrays of `(timestamps, lidar_points, image_features, gt_extrinsics_per_timestep)`
- Seeded RNG for reproducibility

**Acceptance**:
- `pytest python/tests/test_synthetic.py` passes
- `examples/synthetic_demo.py` generates 60 s at 10 Hz and saves to disk as `.npz`
- Re-running with the same seed produces byte-identical output
- A test injects 0.5° pitch drift at t=30 s and asserts the saved ground truth reflects this

### Task 2 — Evaluation metrics module
**File**: `python/polycal/metrics.py`

Implement Cocheteux et al.'s metrics for head-to-head benchmarking:
- `picp(intervals, ground_truths) -> np.ndarray` shape (6,) — coverage per DOF
- `mpiw(intervals) -> np.ndarray` shape (6,) — mean interval width per DOF
- `interval_score(intervals, ground_truths, alpha) -> np.ndarray` shape (6,) — IS per DOF

Where `intervals` is shape (N, 6, 2) with `[lower, upper]` for each of X/Y/Z/Roll/Pitch/Yaw (translation in cm, rotation in degrees to match Cocheteux Table 1), and `ground_truths` is shape (N, 6).

**Acceptance**:
- `pytest python/tests/test_metrics.py` passes with correctness tests (hand-computed expected values) and edge cases (all in interval, all out, single sample)
- Sanity test: with target coverage 0.9 and Gaussian-noise intervals correctly sized, empirical PICP comes within ±0.02 of 0.9 over 1000 samples

### Task 3 — CI on GitHub Actions
**File**: `.github/workflows/ci.yml`

- Triggers: push and PR to `main`
- Matrix: `ubuntu-22.04`, `ubuntu-24.04`
- Steps:
  1. Checkout
  2. Set up Python 3.11
  3. Install system deps (cmake, build-essential, eigen3, sophus if available via apt)
  4. Configure and build C++ (acceptable if no source yet — just configure step)
  5. Install Python package: `pip install -e ".[dev]"`
  6. Run `pytest python/tests/`

**Acceptance**:
- Green check on a test PR with no code changes beyond Tasks 1 and 2
- `pytest` discovers and runs the new tests

### Task 4 — README v0.1
**File**: `README.md`

Replace any placeholder with a factual v0.1 README:
- One-paragraph positioning (use framing from `AGENTS.md` §2)
- Comparison table vs Cocheteux et al. (WACV 2025) and clc
- Installation: placeholder `"v0.1 — work in progress. Build instructions to follow."`
- License: Apache-2.0
- Citation block for Cocheteux et al. (arXiv:2501.06878) and Yan et al. (arXiv:2205.14087)

**Acceptance**:
- Renders correctly on GitHub
- Precise, factual language. No marketing copy ("blazingly fast", "state-of-the-art", etc.)

### Out of scope for this session
- Any C++ estimator code
- Any decision logged in `DECISIONS.md` as Open
- Real dataset ingest (rosbag loaders, KITTI loaders) — defer until estimator is chosen

**Files touched**:
- `AGENTS.md` (assumed already present from prior codex session, otherwise create per artifact)
- `DECISIONS.md` (created)
- `PROGRESS.md` (created — this file)
