# AGENTS.md — polycal

Context file for AI coding agents (Claude Code, Codex, Cursor, and similar) working on polycal. Read this file in full before starting any session. Read `PROGRESS.md` immediately after.

---

## 1. Project identity

**polycal** is an open-source library for online LiDAR–camera extrinsic calibration with empirically calibrated uncertainty and drift detection, targeted at autonomous vehicle fleets.

Three technical layers:

1. **Recursive estimator** — continuous EKF or sliding-window factor graph over SE(3) extrinsics, fed by streaming sensor observations.
2. **Drift detection** — change-point statistics (CUSUM, SPRT) on innovations with controlled false-alarm rate and characterized detection delay.
3. **Calibrated coverage** — the stated x% confidence interval empirically contains the true extrinsic x% of the time, validated against drift-injected ground truth.

---

## 2. Positioning vs prior art

Polycal is **the classical model-based analog** to Cocheteux et al.'s learning-based conformal prediction approach. Differentiating dimensions:

| Dimension | Cocheteux et al. (WACV 2025) | polycal |
|---|---|---|
| Uncertainty source | MC Dropout over neural network | Physical noise propagation via Fisher information |
| Training data | KITTI/DSEC labeled, 60/15/15/10 split | None — datasheet noise models |
| Coverage semantics | Marginal (conformal) | Conditional (asymptotic, Fisher) |
| State | Stateless per-frame inference | Recursive over time |
| Drift detection | Aspirational only (§3) | Implemented and characterized |
| Compute | GPU at inference (25 fwd passes) | CPU only, matrix ops |
| Operating range | ±1° / ±10cm (training distribution) | Bounded by convergence basin |
| Interpretability | σ is a scalar per parameter | Covariance decomposable by noise source |

**Reference papers and code:**
- Cocheteux, Moreau, Davoine. *Uncertainty-Aware Online Extrinsic Calibration: A Conformal Prediction Approach.* WACV 2025. arXiv:2501.06878. **The direct benchmark target.**
- Kogan, `clc` (github.com/dkogan/camera-lidar-calibration). **Offline** target-based calibration with rigorous covariance. Polycal does NOT replicate this — clc owns the offline rigorous niche.
- Yan et al. *OpenCalib.* arXiv:2205.14087. The breadth baseline. Polycal does NOT compete on breadth.

---

## 3. Anti-scope (things polycal explicitly does NOT do)

Do NOT extend the project to include any of the following without explicit user approval:

- **Offline target-based calibration.** Kogan's clc owns this.
- **Multi-modal calibration beyond LiDAR–camera** (no IMU, radar, multi-LiDAR in v1).
- **Learning-based components in the core estimator.** Defeats the classical-analog positioning.
- **A factory calibration workflow.** Different problem, different users.
- **A GUI.** CLI + Python API is sufficient.

If a session naturally drifts toward any of these, stop and flag it in `PROGRESS.md` under "Open questions raised" rather than building it.

---

## 4. Technical stack

- **Core**: C++17, Eigen, optimization library (Ceres or GTSAM — decision pending; check `PROGRESS.md`).
- **Geometry**: Sophus for SE(3) / SO(3). Do NOT roll custom Lie group code.
- **Python interop**: pybind11.
- **Build**: CMake (modern target-based, `FetchContent` for non-system deps).
- **Testing**: GoogleTest for C++, pytest for Python.
- **Data formats**: rosbag for real-data ingest (via `rosbags` library, NOT requiring ROS install). Synthetic datasets generated in-repo.
- **CI**: GitHub Actions, macOS 14+, Linux CI for portability.
- **License**: Apache-2.0 (matches clc, OpenCalib).

Do not introduce new dependencies without justification logged in `PROGRESS.md`.

---

## 5. Code conventions

- **C++**: Google style with these deviations: 4-space indent, `snake_case` for functions and variables, `PascalCase` for types.
- **Python**: PEP 8, type hints required for public API, Google-style docstrings.
- Public headers in `cpp/include/polycal/`. Implementation in `cpp/src/`.
- No `using namespace` in headers.
- Prefer `Eigen` typedefs (`Mat6d`, `Vec6d`) over raw template syntax.
- No silent `auto` for matrix expressions — Eigen lazy evaluation traps are a real failure mode.

---

## 6. Session protocol

### Start of session

1. Read this file (`AGENTS.md`) in full.
2. Read `PROGRESS.md` from the top until you have full context on the current state.
3. Read any open items flagged in the latest progress entry.
4. Confirm understanding of the immediate next task before writing code. If the next priority is unclear, ask the user.

### During session

- Make small, atomic commits with clear messages.
- If you encounter an unresolved technical decision, do NOT make it silently. Log it as an open question in `PROGRESS.md` and ask the user.
- If experiments fail, record what was tried and why it didn't work. **Negative knowledge is the most important thing to preserve across sessions** — the next agent should not repeat dead ends.

### End of session

Append a new entry to the **top** of `PROGRESS.md` using the template in §7. Do not omit any section; if a section is empty, write "None."

---

## 7. PROGRESS.md entry template

````markdown
## YYYY-MM-DD — <agent: claude | codex | other> — <session topic>

**Worked on**: <1–2 sentences of what this session covered>

**Completed**:
- <concrete artifacts: files created, tests passing, benchmarks run>

**Attempted but did not work**:
- <experiments that failed and why — be specific so the next agent doesn't repeat>

**Decisions made**:
- <technical or scope decisions, with rationale>

**Open questions raised**:
- <unresolved items requiring user input>

**Next session — priorities in order**:
1. <most important next step>
2. <second>
3. <third>

**Files touched**:
- `path/to/file.cpp`
- `path/to/file.py`
````

---

## 8. Communication with the user

The user is **Zoey**. Preferences:

- Factual, precise, concise. No emotional language, praise, or patronization.
- No grammar or formulation feedback on her chat messages.
- Copy-paste-ready outputs when relevant.
- Push back when wrong. Do not capitulate to incorrect technical claims.

When ending a session, summarize in 3–5 lines: what shipped, what is blocked, what is next. Full detail lives in `PROGRESS.md`.

---

## 9. Things to flag immediately (do not proceed silently)

- Any prior-art finding that suggests polycal's wedge is occupied or weaker than believed.
- Any methodological dead-end that requires a scope pivot.
- Any dependency or environment issue that blocks reproducibility.
- Any benchmark result substantially worse than Cocheteux et al.'s reported numbers on KITTI:
  - 90% coverage: X 2.81 cm, Y 1.64 cm, Z 2.43 cm, Roll 0.14°, Pitch 0.31°, Yaw 0.15°
  - PICP within 1–2 points of target (89.4 / 88.7 / 89.2 / 89.3 / 90.5 / 88.8 at 90% target)

---

## 10. Cross-agent trust

When you read a `PROGRESS.md` entry written by a different agent (Claude, Codex, another), trust it. Do not re-verify completed work unless there is concrete evidence of an error. Re-doing prior work is the largest waste mode in multi-agent projects.

If you suspect a prior entry is wrong, flag it as a new open question rather than silently redoing the work.
