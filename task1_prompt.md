# Task 1 — Synthetic Data Generator

You are working on polycal. Before starting, read `AGENTS.md`, `DECISIONS.md`, and `PROGRESS.md` in that order. This task implements PROGRESS.md → "Next session — priorities in order" → Task 1.

## Scope

Implement a parametric synthetic LiDAR–camera observation generator with continuous-time drift modeling. Python-only. No C++ in this task.

## Locked design decisions

- **Scene representation**: parametric (planes, line segments, point landmarks). No rendering.
- **Output layering**: emit BOTH raw sensor data (LiDAR sweeps, image features) AND pre-extracted correspondences. Estimator front-ends can consume either layer.
- **Drift profiles**: implement `StaticDrift` and `LinearDrift` only. `StepDrift` and `RandomWalkDrift` deferred.
- **SE(3) representation**: `scipy.spatial.transform.Rotation` + translation vector (4×4 homogeneous matrix as canonical exchange format). No Sophus, no custom Lie group code in Python layer.

## Deliverables

### Files to create

```
python/polycal/__init__.py
python/polycal/synthetic.py
python/polycal/__init__.py exposes: SyntheticConfig, SyntheticDataset, generate, CameraModel, LidarModel, Plane, LineSegment, Landmark, SceneConfig, StaticDrift, LinearDrift
python/tests/__init__.py
python/tests/test_synthetic.py
python/tests/test_synthetic_noise.py
examples/synthetic_demo.py
examples/monte_carlo_sweep.py     # stub only
pyproject.toml                     # if not already present from prior session
```

### Module structure (`python/polycal/synthetic.py`)

Use `@dataclass` for all config types. Use `numpy` arrays for bulk data, `scipy.spatial.transform.Rotation` for rotations.

```python
@dataclass
class CameraModel:
    fx: float = 720.0
    fy: float = 720.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    pixel_noise_std: float = 0.5  # pixels

    def project(self, points_cam: np.ndarray) -> np.ndarray:
        """Project Nx3 camera-frame points to Nx2 pixel coords. No distortion in v1."""

@dataclass
class LidarModel:
    n_beams: int = 64
    vertical_fov_deg: tuple[float, float] = (-24.9, 2.0)
    horizontal_resolution_deg: float = 0.2
    range_noise_std: float = 0.02  # meters
    max_range: float = 80.0

    def sweep_directions(self) -> np.ndarray:
        """Return unit ray directions in LiDAR frame, shape (n_beams * n_azimuth, 3)."""

@dataclass
class Plane:
    normal: np.ndarray              # (3,) unit vector
    offset: float                   # n · x = offset
    extent: tuple[float, float]     # half-widths in plane-local x, y
    center: np.ndarray              # (3,) point on plane defining origin

@dataclass
class LineSegment:
    start: np.ndarray               # (3,)
    end: np.ndarray                 # (3,)

@dataclass
class Landmark:
    position: np.ndarray            # (3,)

@dataclass
class SceneConfig:
    planes: list[Plane]
    lines: list[LineSegment]
    landmarks: list[Landmark]

    @staticmethod
    def default_urban() -> "SceneConfig":
        """A reasonable default scene: 2 ground planes, 3 vertical walls, 6 poles, 12 landmarks."""

# Drift profiles
class DriftProfile(Protocol):
    def __call__(self, t: float) -> np.ndarray:  # 4x4 SE(3)
        ...

@dataclass
class StaticDrift:
    T: np.ndarray  # 4x4

    def __call__(self, t: float) -> np.ndarray:
        return self.T.copy()

@dataclass
class LinearDrift:
    T_start: np.ndarray             # 4x4
    angular_velocity_rad_s: np.ndarray   # (3,) axis-angle rate
    linear_velocity_m_s: np.ndarray      # (3,)

    def __call__(self, t: float) -> np.ndarray:
        """Apply exponential map of (omega * t, v * t) to T_start."""

@dataclass
class SyntheticConfig:
    duration_s: float = 60.0
    rate_hz: float = 10.0
    seed: int = 0
    camera: CameraModel = field(default_factory=CameraModel)
    lidar: LidarModel = field(default_factory=LidarModel)
    scene: SceneConfig = field(default_factory=SceneConfig.default_urban)
    initial_extrinsic: np.ndarray = field(default_factory=lambda: np.eye(4))
    drift_profile: DriftProfile = field(default_factory=lambda: StaticDrift(np.eye(4)))

@dataclass
class Correspondence:
    """Pre-extracted matched LiDAR-camera feature at one timestep."""
    lidar_point: np.ndarray         # (3,) in LiDAR frame
    pixel: np.ndarray               # (2,) in image
    landmark_id: int                # which Landmark this came from

@dataclass
class SyntheticDataset:
    timestamps: np.ndarray                          # (N,)
    lidar_sweeps: list[np.ndarray]                  # length N, each (M_i, 3) in LiDAR frame
    image_features: list[np.ndarray]                # length N, each (K_i, 2) in pixels
    correspondences: list[list[Correspondence]]     # length N
    gt_extrinsics: np.ndarray                       # (N, 4, 4) LiDAR->camera SE(3) per timestep
    config: SyntheticConfig                         # for reproducibility

    def save(self, path: Path) -> None:
        """Save to .npz. Config is serialized as a JSON sidecar (path.with_suffix('.json'))."""

    @classmethod
    def load(cls, path: Path) -> "SyntheticDataset":
        ...

def generate(config: SyntheticConfig) -> SyntheticDataset:
    """Main entry point. Produces a deterministic dataset given the seed."""
```

### Generation logic (in `generate`)

For each timestep `t_i`:

1. Compute ground-truth extrinsic `T_lc(t_i) = drift_profile(t_i)` (LiDAR → camera transform).
2. **LiDAR sweep**: cast all rays from `LidarModel.sweep_directions()`. For each ray, find nearest intersection with scene primitives (plane, line). Add Gaussian range noise σ = `range_noise_std`. Drop returns beyond `max_range`. Store points in LiDAR frame.
3. **Image features**: project all `Landmark` positions into the camera using `T_lc` and `CameraModel.project()`. Keep only those within image bounds. Add Gaussian pixel noise σ = `pixel_noise_std`.
4. **Correspondences**: for each in-frame landmark, also generate a LiDAR observation by intersecting a ray to the landmark (treat landmark as small sphere or just use ground-truth position with range noise). Pair the noisy LiDAR point with the noisy pixel. Store with `landmark_id`.

Determinism: derive all RNG from `np.random.default_rng(config.seed)`. Pass child RNGs to subroutines via `rng.spawn()` (Python 3.12+) or by stream slicing.

### Test requirements (`test_synthetic.py`)

```python
def test_static_drift_produces_constant_ground_truth():
    # StaticDrift → gt_extrinsics[i] == gt_extrinsics[0] for all i (within float tol)

def test_linear_drift_accumulates_over_time():
    # LinearDrift with known angular_velocity → gt rotation at t=30s matches analytic

def test_seed_determinism():
    # Two calls with same seed produce byte-identical arrays
    # Save and reload preserves equality

def test_lidar_sweep_size():
    # For default 64-beam, 0.2° azimuth resolution → expect ~64 * 1800 = 115200 rays max
    # Actual return count <= ray count

def test_image_feature_counts_reasonable():
    # Default urban scene → 5–50 features per frame
    # All features within image bounds

def test_correspondences_consistent():
    # For each Correspondence: pixel ≈ project(landmark) under T_lc
    # lidar_point distance from landmark < 5 * range_noise_std (i.e., 5σ outlier check)

def test_save_load_roundtrip():
    # Generate → save → load → fields equal
```

### Test requirements (`test_synthetic_noise.py`)

```python
def test_pixel_noise_matches_config():
    # Generate 1000 samples of a single landmark observation
    # Empirical pixel σ within 5% of configured pixel_noise_std

def test_range_noise_matches_config():
    # Generate 1000 LiDAR returns from a single ray-plane intersection
    # Empirical range σ within 5% of configured range_noise_std
```

### Demo (`examples/synthetic_demo.py`)

```
1. Create default config (60s @ 10Hz, default urban scene, LinearDrift with small rates)
2. Run generate()
3. Save to data/synthetic_demo.npz (gitignored)
4. Visualize frame 300 (t=30s):
   - Plot image with projected LiDAR points overlaid, colored by depth
   - Save to data/synthetic_demo_frame300.png
5. Print summary: frame count, total LiDAR returns, total image features, drift magnitude over duration
```

### Monte Carlo stub (`examples/monte_carlo_sweep.py`)

```python
"""Stub for Phase 1 Monte Carlo evaluation.

Will run N trials with varying seeds and aggregate estimator outputs.
Not implemented in Task 1 — estimator does not yet exist.
"""
if __name__ == "__main__":
    raise NotImplementedError("Monte Carlo evaluation deferred to Phase 1.")
```

## Constraints

- **No C++ in this task.** Pure Python.
- **No external simulators.** Do not import CARLA, AirSim, Open3D, ROS, or any rendering library.
- **No new dependencies beyond**: numpy, scipy, matplotlib (for the demo plot only), pytest. If you think you need anything else, stop and ask.
- **No type-check-then-cast patterns.** Use proper static types (`np.ndarray`, `list[X]`).
- **Match the polycal Python conventions in `AGENTS.md` §5.**
- **All public functions and classes need Google-style docstrings.**

## Open questions handling

If you hit a design ambiguity not covered above, **do not guess**. Add an entry to `PROGRESS.md` under "Open questions raised" and stop the relevant subtask. Continue with other subtasks if possible.

## Acceptance — must all pass before ending session

1. `pytest python/tests/test_synthetic.py` passes
2. `pytest python/tests/test_synthetic_noise.py` passes
3. `python examples/synthetic_demo.py` runs, produces `.npz` and `.png` in `data/`
4. Re-running demo with same seed produces byte-identical `.npz` (verify via `cmp` or `hashlib`)
5. `python -c "import polycal; print(polycal.__version__)"` works (set `__version__ = "0.1.0a1"` in `__init__.py`)

## Session end protocol

Append entry to top of `PROGRESS.md` per `AGENTS.md` §7. Required sections:
- Worked on
- Completed (list each acceptance criterion you verified)
- Attempted but did not work (negative knowledge — be specific)
- Decisions made
- Open questions raised
- Next session — priorities in order
- Files touched

Do not begin Task 2 in this session. After Task 1 lands and PROGRESS.md is updated, end the session.
