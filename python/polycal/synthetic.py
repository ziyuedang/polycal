"""Synthetic LiDAR-camera data generation for polycal.

The generator is intentionally parametric: it emits simple geometric sensor
observations and landmark correspondences without depending on a renderer.
"""

from __future__ import annotations

import dataclasses
import io
import json
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from scipy.spatial.transform import Rotation


Array = np.ndarray


@dataclass
class CameraModel:
    """Pinhole camera model with Gaussian pixel detection noise."""

    fx: float = 720.0
    fy: float = 720.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    pixel_noise_std: float = 0.5

    def project(self, points_cam: Array) -> Array:
        """Project camera-frame points to pixel coordinates.

        Args:
            points_cam: Array with shape ``(N, 3)`` in the camera frame.

        Returns:
            Pixel coordinates with shape ``(N, 2)``. Points with non-positive
            depth are projected algebraically and should be filtered by caller.
        """
        points = np.asarray(points_cam, dtype=float)
        z = points[:, 2]
        pixels = np.empty((points.shape[0], 2), dtype=float)
        pixels[:, 0] = self.fx * points[:, 0] / z + self.cx
        pixels[:, 1] = self.fy * points[:, 1] / z + self.cy
        return pixels


@dataclass
class LidarModel:
    """Spinning LiDAR model with deterministic angular sampling."""

    n_beams: int = 64
    vertical_fov_deg: tuple[float, float] = (-24.9, 2.0)
    horizontal_resolution_deg: float = 0.2
    range_noise_std: float = 0.02
    max_range: float = 80.0

    def sweep_directions(self) -> Array:
        """Return LiDAR-frame unit ray directions.

        The LiDAR frame uses +Z as forward, +X as right, and +Y as down to
        match the default pinhole camera convention used by this Python layer.
        """
        vertical = np.deg2rad(
            np.linspace(
                self.vertical_fov_deg[0],
                self.vertical_fov_deg[1],
                self.n_beams,
            )
        )
        n_azimuth = int(round(360.0 / self.horizontal_resolution_deg))
        azimuth = np.deg2rad(np.arange(n_azimuth) * self.horizontal_resolution_deg)
        az, el = np.meshgrid(azimuth, vertical)
        dirs = np.column_stack(
            [
                np.cos(el).ravel() * np.sin(az).ravel(),
                -np.sin(el).ravel(),
                np.cos(el).ravel() * np.cos(az).ravel(),
            ]
        )
        norms = np.linalg.norm(dirs, axis=1, keepdims=True)
        return dirs / norms


@dataclass
class Plane:
    """Finite plane primitive described by ``normal · x = offset``."""

    normal: Array
    offset: float
    extent: tuple[float, float]
    center: Array


@dataclass
class LineSegment:
    """Finite 3D line segment primitive."""

    start: Array
    end: Array


@dataclass
class Landmark:
    """Sparse 3D landmark position in the LiDAR/world frame."""

    position: Array


@dataclass
class SceneConfig:
    """Collection of parametric scene primitives."""

    planes: list[Plane]
    lines: list[LineSegment]
    landmarks: list[Landmark]

    @staticmethod
    def default_urban() -> "SceneConfig":
        """Create a small urban-like scene with planes, poles, and landmarks."""
        planes = [
            Plane(
                normal=np.array([0.0, 1.0, 0.0]),
                offset=1.6,
                extent=(2.0, 14.0),
                center=np.array([0.0, 1.6, 25.0]),
            ),
            Plane(
                normal=np.array([0.0, -1.0, 0.0]),
                offset=-4.5,
                extent=(2.0, 14.0),
                center=np.array([0.0, 4.5, 25.0]),
            ),
            Plane(
                normal=np.array([1.0, 0.0, 0.0]),
                offset=7.0,
                extent=(0.8, 10.0),
                center=np.array([7.0, 1.0, 24.0]),
            ),
            Plane(
                normal=np.array([-1.0, 0.0, 0.0]),
                offset=7.0,
                extent=(0.8, 10.0),
                center=np.array([-7.0, 1.0, 24.0]),
            ),
            Plane(
                normal=np.array([0.0, 0.0, -1.0]),
                offset=-42.0,
                extent=(1.5, 1.0),
                center=np.array([0.0, 0.5, 42.0]),
            ),
        ]
        pole_x = [-5.5, -3.0, -1.0, 1.0, 3.0, 5.5]
        lines = [
            LineSegment(
                start=np.array([x, 1.5, 10.0 + 3.5 * i]),
                end=np.array([x, -2.5, 10.0 + 3.5 * i]),
            )
            for i, x in enumerate(pole_x)
        ]
        landmarks = [
            Landmark(np.array([x, y, z]))
            for x, y, z in [
                (-2.4, -0.9, 12.0),
                (0.5, -0.7, 13.5),
                (2.8, -0.6, 15.0),
                (-3.6, 0.2, 17.0),
                (3.8, 0.1, 18.5),
                (-1.0, -1.1, 21.0),
                (1.8, -0.8, 23.5),
                (-4.5, 0.5, 26.0),
                (4.5, 0.4, 29.0),
                (-2.0, -1.2, 32.0),
                (2.4, -1.0, 36.0),
                (0.0, -0.5, 40.0),
            ]
        ]
        return SceneConfig(planes=planes, lines=lines, landmarks=landmarks)


class DriftProfile(Protocol):
    """Callable object returning the LiDAR-to-camera transform at time ``t``."""

    def __call__(self, t: float) -> Array:
        """Return a 4x4 homogeneous SE(3) matrix."""


@dataclass
class StaticDrift:
    """Time-invariant LiDAR-to-camera extrinsic."""

    T: Array

    def __call__(self, t: float) -> Array:
        """Return the same transform for every timestamp."""
        return self.T.copy()


@dataclass
class LinearDrift:
    """Constant angular and translational drift applied after ``T_start``."""

    T_start: Array
    angular_velocity_rad_s: Array
    linear_velocity_m_s: Array

    def __call__(self, t: float) -> Array:
        """Apply an axis-angle rotation and linear translation over time."""
        T = np.asarray(self.T_start, dtype=float).copy()
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_rotvec(
            np.asarray(self.angular_velocity_rad_s, dtype=float) * t
        ).as_matrix()
        delta[:3, 3] = np.asarray(self.linear_velocity_m_s, dtype=float) * t
        return delta @ T


@dataclass
class SyntheticConfig:
    """Configuration for generating a synthetic sequence."""

    duration_s: float = 60.0
    rate_hz: float = 10.0
    seed: int = 0
    camera: CameraModel = field(default_factory=CameraModel)
    lidar: LidarModel = field(default_factory=LidarModel)
    scene: SceneConfig = field(default_factory=SceneConfig.default_urban)
    initial_extrinsic: Array = field(default_factory=lambda: np.eye(4))
    drift_profile: DriftProfile = field(default_factory=lambda: StaticDrift(np.eye(4)))


@dataclass
class Correspondence:
    """Pre-extracted matched LiDAR-camera feature at one timestep."""

    lidar_point: Array
    pixel: Array
    landmark_id: int


@dataclass
class SyntheticDataset:
    """Generated synthetic sequence and reproducibility metadata."""

    timestamps: Array
    lidar_sweeps: list[Array]
    image_features: list[Array]
    correspondences: list[list[Correspondence]]
    gt_extrinsics: Array
    config: SyntheticConfig

    def save(self, path: Path) -> None:
        """Save dataset arrays to ``.npz`` and config metadata to JSON.

        Args:
            path: Destination ``.npz`` path. A deterministic JSON sidecar is
                written at ``path.with_suffix(".json")``.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        arrays = _dataset_to_arrays(self)
        _save_npz_deterministic(path, arrays)
        path.with_suffix(".json").write_text(
            json.dumps(_config_to_jsonable(self.config), sort_keys=True, indent=2)
            + "\n",
            encoding="utf-8",
        )

    @classmethod
    def load(cls, path: Path) -> "SyntheticDataset":
        """Load a dataset saved by :meth:`SyntheticDataset.save`."""
        path = Path(path)
        with np.load(path, allow_pickle=False) as data:
            timestamps = data["timestamps"]
            gt_extrinsics = data["gt_extrinsics"]
            lidar_counts = data["lidar_counts"]
            lidar_points = data["lidar_points"]
            feature_counts = data["feature_counts"]
            image_feature_points = data["image_feature_points"]
            corr_counts = data["corr_counts"]
            corr_lidar_points = data["corr_lidar_points"]
            corr_pixels = data["corr_pixels"]
            corr_landmark_ids = data["corr_landmark_ids"]

        lidar_sweeps = _split_rows(lidar_points, lidar_counts)
        image_features = _split_rows(image_feature_points, feature_counts)
        correspondences: list[list[Correspondence]] = []
        cursor = 0
        for count in corr_counts:
            frame_corrs = []
            for _ in range(int(count)):
                frame_corrs.append(
                    Correspondence(
                        lidar_point=corr_lidar_points[cursor].copy(),
                        pixel=corr_pixels[cursor].copy(),
                        landmark_id=int(corr_landmark_ids[cursor]),
                    )
                )
                cursor += 1
            correspondences.append(frame_corrs)
        config_json = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
        return cls(
            timestamps=timestamps,
            lidar_sweeps=lidar_sweeps,
            image_features=image_features,
            correspondences=correspondences,
            gt_extrinsics=gt_extrinsics,
            config=_config_from_jsonable(config_json),
        )


def generate(config: SyntheticConfig) -> SyntheticDataset:
    """Generate a deterministic synthetic LiDAR-camera sequence.

    Args:
        config: Generator configuration. Reusing the same seed and config
            produces identical numeric outputs.

    Returns:
        Synthetic dataset with raw observations, correspondences, and ground
        truth extrinsics.
    """
    rng = np.random.default_rng(config.seed)
    frame_count = int(round(config.duration_s * config.rate_hz))
    timestamps = np.arange(frame_count, dtype=float) / config.rate_hz
    directions = config.lidar.sweep_directions()
    base_lidar_distances = _scene_intersection_distances(config.scene, directions)
    lidar_sweeps: list[Array] = []
    image_features: list[Array] = []
    correspondences: list[list[Correspondence]] = []
    gt_extrinsics = np.empty((frame_count, 4, 4), dtype=float)

    for index, timestamp in enumerate(timestamps):
        T_lc = np.asarray(config.drift_profile(float(timestamp)), dtype=float)
        gt_extrinsics[index] = T_lc
        lidar_sweeps.append(
            _simulate_lidar_sweep(config.lidar, directions, base_lidar_distances, rng)
        )
        features, frame_corrs = _simulate_camera_and_correspondences(
            config.scene,
            config.camera,
            config.lidar,
            T_lc,
            rng,
        )
        image_features.append(features)
        correspondences.append(frame_corrs)

    return SyntheticDataset(
        timestamps=timestamps,
        lidar_sweeps=lidar_sweeps,
        image_features=image_features,
        correspondences=correspondences,
        gt_extrinsics=gt_extrinsics,
        config=config,
    )


def _simulate_lidar_sweep(
    lidar: LidarModel,
    directions: Array,
    distances: Array,
    rng: np.random.Generator,
) -> Array:
    finite = np.isfinite(distances)
    if not np.any(finite):
        return np.empty((0, 3), dtype=float)
    noisy_distances = distances[finite] + rng.normal(
        0.0,
        lidar.range_noise_std,
        size=int(np.count_nonzero(finite)),
    )
    valid = (noisy_distances > 0.0) & (noisy_distances <= lidar.max_range)
    return directions[finite][valid] * noisy_distances[valid, None]


def _scene_intersection_distances(scene: SceneConfig, directions: Array) -> Array:
    distances = np.full(directions.shape[0], np.inf, dtype=float)
    for plane in scene.planes:
        plane_distances = _intersect_plane_many(directions, plane)
        distances = np.minimum(distances, plane_distances)
    for line in scene.lines:
        line_distances = _intersect_line_many(directions, line)
        distances = np.minimum(distances, line_distances)
    return distances


def _simulate_camera_and_correspondences(
    scene: SceneConfig,
    camera: CameraModel,
    lidar: LidarModel,
    T_lc: Array,
    rng: np.random.Generator,
) -> tuple[Array, list[Correspondence]]:
    positions = np.array([landmark.position for landmark in scene.landmarks], dtype=float)
    if positions.size == 0:
        return np.empty((0, 2), dtype=float), []
    points_cam = _transform_points(T_lc, positions)
    pixels = camera.project(points_cam)
    in_frame = (
        (points_cam[:, 2] > 0.0)
        & (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < camera.width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < camera.height)
    )
    features = []
    correspondences = []
    for landmark_id in np.flatnonzero(in_frame):
        pixel = pixels[landmark_id] + rng.normal(0.0, camera.pixel_noise_std, size=2)
        if not (0.0 <= pixel[0] < camera.width and 0.0 <= pixel[1] < camera.height):
            continue
        position = positions[landmark_id]
        direction = position / np.linalg.norm(position)
        noisy_point = position + direction * rng.normal(0.0, lidar.range_noise_std)
        features.append(pixel)
        correspondences.append(
            Correspondence(
                lidar_point=noisy_point,
                pixel=pixel,
                landmark_id=int(landmark_id),
            )
        )
    if not features:
        return np.empty((0, 2), dtype=float), correspondences
    return np.vstack(features), correspondences


def _nearest_intersection(direction: Array, scene: SceneConfig) -> float | None:
    distances = []
    for plane in scene.planes:
        distance = _intersect_plane(direction, plane)
        if distance is not None:
            distances.append(distance)
    for line in scene.lines:
        distance = _intersect_line(direction, line)
        if distance is not None:
            distances.append(distance)
    if not distances:
        return None
    return min(distances)


def _intersect_plane_many(directions: Array, plane: Plane) -> Array:
    normal = _unit(np.asarray(plane.normal, dtype=float))
    denom = directions @ normal
    valid = np.abs(denom) >= 1e-9
    distances = np.full(directions.shape[0], np.inf, dtype=float)
    distances[valid] = float(plane.offset) / denom[valid]
    valid &= distances > 0.0
    points = np.zeros_like(directions)
    points[valid] = directions[valid] * distances[valid, None]
    u_axis, v_axis = _plane_basis(normal)
    relative = points - np.asarray(plane.center, dtype=float)
    inside = (
        (np.abs(relative @ u_axis) <= plane.extent[0])
        & (np.abs(relative @ v_axis) <= plane.extent[1])
    )
    distances[~(valid & inside)] = np.inf
    return distances


def _intersect_line_many(
    directions: Array,
    line: LineSegment,
    radius: float = 0.08,
) -> Array:
    start = np.asarray(line.start, dtype=float)
    end = np.asarray(line.end, dtype=float)
    segment = end - start
    seg_len_sq = float(segment @ segment)
    distances = np.full(directions.shape[0], np.inf, dtype=float)
    if seg_len_sq <= 0.0:
        return distances
    b = directions @ segment
    d = directions @ start
    e = float(segment @ start)
    denom = seg_len_sq - b * b
    valid = np.abs(denom) >= 1e-12
    ray_t = np.full(directions.shape[0], np.inf, dtype=float)
    seg_t = np.zeros(directions.shape[0], dtype=float)
    ray_t[valid] = (b[valid] * e - seg_len_sq * d[valid]) / denom[valid]
    seg_t[valid] = (e - b[valid] * d[valid]) / denom[valid]
    seg_t = np.clip(seg_t, 0.0, 1.0)
    closest_ray = directions * ray_t[:, None]
    closest_segment = start + seg_t[:, None] * segment
    close = np.linalg.norm(closest_ray - closest_segment, axis=1) <= radius
    distances[valid & close & (ray_t > 0.0)] = ray_t[valid & close & (ray_t > 0.0)]
    return distances


def _intersect_plane(direction: Array, plane: Plane) -> float | None:
    normal = _unit(np.asarray(plane.normal, dtype=float))
    denom = float(normal @ direction)
    if abs(denom) < 1e-9:
        return None
    distance = float(plane.offset / denom)
    if distance <= 0.0:
        return None
    point = direction * distance
    u_axis, v_axis = _plane_basis(normal)
    relative = point - np.asarray(plane.center, dtype=float)
    if (
        abs(float(relative @ u_axis)) <= plane.extent[0]
        and abs(float(relative @ v_axis)) <= plane.extent[1]
    ):
        return distance
    return None


def _intersect_line(direction: Array, line: LineSegment, radius: float = 0.08) -> float | None:
    start = np.asarray(line.start, dtype=float)
    end = np.asarray(line.end, dtype=float)
    segment = end - start
    seg_len_sq = float(segment @ segment)
    if seg_len_sq <= 0.0:
        return None
    a = float(direction @ direction)
    b = float(direction @ segment)
    c = seg_len_sq
    d = float(direction @ start)
    e = float(segment @ start)
    denom = a * c - b * b
    if abs(denom) < 1e-12:
        return None
    ray_t = (b * e - c * d) / denom
    seg_t = (a * e - b * d) / denom
    seg_t = min(1.0, max(0.0, seg_t))
    closest_ray = direction * ray_t
    closest_segment = start + seg_t * segment
    if ray_t > 0.0 and np.linalg.norm(closest_ray - closest_segment) <= radius:
        return float(ray_t)
    return None


def _plane_basis(normal: Array) -> tuple[Array, Array]:
    helper = np.array([0.0, 1.0, 0.0])
    if abs(float(helper @ normal)) > 0.9:
        helper = np.array([1.0, 0.0, 0.0])
    u_axis = _unit(np.cross(normal, helper))
    v_axis = _unit(np.cross(normal, u_axis))
    return u_axis, v_axis


def _unit(vector: Array) -> Array:
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        raise ValueError("zero-length vector cannot be normalized")
    return vector / norm


def _transform_points(T: Array, points: Array) -> Array:
    rotation = T[:3, :3]
    translation = T[:3, 3]
    return (rotation @ points.T).T + translation


def _split_rows(values: Array, counts: Array) -> list[Array]:
    rows = []
    cursor = 0
    for count in counts:
        next_cursor = cursor + int(count)
        rows.append(values[cursor:next_cursor].copy())
        cursor = next_cursor
    return rows


def _dataset_to_arrays(dataset: SyntheticDataset) -> dict[str, Array]:
    lidar_counts = np.array([points.shape[0] for points in dataset.lidar_sweeps], dtype=np.int64)
    feature_counts = np.array(
        [features.shape[0] for features in dataset.image_features],
        dtype=np.int64,
    )
    corr_counts = np.array([len(corrs) for corrs in dataset.correspondences], dtype=np.int64)
    lidar_points = _concat_or_empty(dataset.lidar_sweeps, 3)
    image_feature_points = _concat_or_empty(dataset.image_features, 2)
    corr_lidar_points = []
    corr_pixels = []
    corr_landmark_ids = []
    for frame_corrs in dataset.correspondences:
        for corr in frame_corrs:
            corr_lidar_points.append(corr.lidar_point)
            corr_pixels.append(corr.pixel)
            corr_landmark_ids.append(corr.landmark_id)
    return {
        "timestamps": dataset.timestamps,
        "gt_extrinsics": dataset.gt_extrinsics,
        "lidar_counts": lidar_counts,
        "lidar_points": lidar_points,
        "feature_counts": feature_counts,
        "image_feature_points": image_feature_points,
        "corr_counts": corr_counts,
        "corr_lidar_points": np.vstack(corr_lidar_points)
        if corr_lidar_points
        else np.empty((0, 3), dtype=float),
        "corr_pixels": np.vstack(corr_pixels) if corr_pixels else np.empty((0, 2), dtype=float),
        "corr_landmark_ids": np.array(corr_landmark_ids, dtype=np.int64),
    }


def _concat_or_empty(arrays: list[Array], width: int) -> Array:
    if not arrays or sum(array.shape[0] for array in arrays) == 0:
        return np.empty((0, width), dtype=float)
    return np.vstack(arrays)


def _save_npz_deterministic(path: Path, arrays: dict[str, Array]) -> None:
    with zipfile.ZipFile(path, mode="w", compression=zipfile.ZIP_STORED) as archive:
        for name in sorted(arrays):
            buffer = io.BytesIO()
            np.save(buffer, arrays[name], allow_pickle=False)
            info = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED
            archive.writestr(info, buffer.getvalue())


def _config_to_jsonable(config: SyntheticConfig) -> dict[str, Any]:
    return {
        "duration_s": config.duration_s,
        "rate_hz": config.rate_hz,
        "seed": config.seed,
        "camera": dataclasses.asdict(config.camera),
        "lidar": dataclasses.asdict(config.lidar),
        "scene": _scene_to_jsonable(config.scene),
        "initial_extrinsic": np.asarray(config.initial_extrinsic).tolist(),
        "drift_profile": _drift_to_jsonable(config.drift_profile),
    }


def _config_from_jsonable(data: dict[str, Any]) -> SyntheticConfig:
    drift = _drift_from_jsonable(data["drift_profile"])
    return SyntheticConfig(
        duration_s=float(data["duration_s"]),
        rate_hz=float(data["rate_hz"]),
        seed=int(data["seed"]),
        camera=CameraModel(**data["camera"]),
        lidar=LidarModel(
            **{
                **data["lidar"],
                "vertical_fov_deg": tuple(data["lidar"]["vertical_fov_deg"]),
            }
        ),
        scene=_scene_from_jsonable(data["scene"]),
        initial_extrinsic=np.array(data["initial_extrinsic"], dtype=float),
        drift_profile=drift,
    )


def _scene_to_jsonable(scene: SceneConfig) -> dict[str, Any]:
    return {
        "planes": [
            {
                "normal": plane.normal.tolist(),
                "offset": plane.offset,
                "extent": list(plane.extent),
                "center": plane.center.tolist(),
            }
            for plane in scene.planes
        ],
        "lines": [
            {"start": line.start.tolist(), "end": line.end.tolist()}
            for line in scene.lines
        ],
        "landmarks": [
            {"position": landmark.position.tolist()} for landmark in scene.landmarks
        ],
    }


def _scene_from_jsonable(data: dict[str, Any]) -> SceneConfig:
    return SceneConfig(
        planes=[
            Plane(
                normal=np.array(item["normal"], dtype=float),
                offset=float(item["offset"]),
                extent=tuple(item["extent"]),
                center=np.array(item["center"], dtype=float),
            )
            for item in data["planes"]
        ],
        lines=[
            LineSegment(
                start=np.array(item["start"], dtype=float),
                end=np.array(item["end"], dtype=float),
            )
            for item in data["lines"]
        ],
        landmarks=[
            Landmark(position=np.array(item["position"], dtype=float))
            for item in data["landmarks"]
        ],
    )


def _drift_to_jsonable(drift: DriftProfile) -> dict[str, Any]:
    if isinstance(drift, StaticDrift):
        return {"type": "StaticDrift", "T": np.asarray(drift.T).tolist()}
    if isinstance(drift, LinearDrift):
        return {
            "type": "LinearDrift",
            "T_start": np.asarray(drift.T_start).tolist(),
            "angular_velocity_rad_s": np.asarray(drift.angular_velocity_rad_s).tolist(),
            "linear_velocity_m_s": np.asarray(drift.linear_velocity_m_s).tolist(),
        }
    raise TypeError(f"unsupported drift profile for serialization: {type(drift)!r}")


def _drift_from_jsonable(data: dict[str, Any]) -> DriftProfile:
    if data["type"] == "StaticDrift":
        return StaticDrift(T=np.array(data["T"], dtype=float))
    if data["type"] == "LinearDrift":
        return LinearDrift(
            T_start=np.array(data["T_start"], dtype=float),
            angular_velocity_rad_s=np.array(data["angular_velocity_rad_s"], dtype=float),
            linear_velocity_m_s=np.array(data["linear_velocity_m_s"], dtype=float),
        )
    raise ValueError(f"unknown drift profile type: {data['type']}")
