#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from functools import lru_cache
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter


_SIM_APP = None
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_EXAMPLE_POINTCLOUD_DIR = REPO_ROOT / "script" / "data" / "example_forest_pointcloud"
DEFAULT_DIGITAL_TWIN_DIR = REPO_ROOT / "data" / "digital_twin"
DEFAULT_CANOPY_PROTOTYPE_CANDIDATES: tuple[Path, ...] = (
    REPO_ROOT.parent / "isaac-sim" / "plan" / "canopy_assets" / "pine_tree.usd",
    SCRIPT_DIR / "canopy_assets" / "pine_tree.usd",
)


def default_canopy_prototypes() -> list[Path]:
    return [path for path in DEFAULT_CANOPY_PROTOTYPE_CANDIDATES if path.exists()]


@dataclass
class GroundConfig:
    resolution: float = 0.5
    thickness: float = 0.5
    padding: float = 4.0
    max_dim: int = 2048
    voxel_size: float = 0.2
    clip_low_q: float = 1.0
    clip_high_q: float = 99.5
    smoothing_sigma: float = 0.5
    collider_approximation: str = "sdf"


@dataclass
class TrunkConfig:
    voxel_size: float = 0.15
    cluster_cell_size: float = 0.8
    min_points: int = 20
    min_cluster_size: int = 60
    radius_scale: float = 1.0
    height_scale: float = 1.08
    radius_min: float = 0.12
    radius_max: float = 0.45
    height_min: float = 0.8
    height_max: float = 12.0
    prune_min_center_distance: float = 0.75
    prune_overlap_factor: float = 0.9
    ground_offset: float = 0.02


@dataclass
class CanopyConfig:
    prototype_paths: list[Path]
    scale_min: float = 0.95
    scale_max: float = 1.15
    radius_scale_min: float = 2.4
    radius_scale_max: float = 4.2
    height_scale_min: float = 0.70
    height_scale_max: float = 1.18
    z_offset_min: float = -0.18
    z_offset_max: float = 0.25
    trunk_canopy_top_gap: float = 1.0
    color_jitter_r: float = 0.04
    color_jitter_g: float = 0.04
    color_jitter_b: float = 0.05
    fallback_radius_multiplier: float = 2.8
    fallback_radius_floor: float = 1.1


@dataclass
class ForestConfig:
    ground: GroundConfig
    trunk: TrunkConfig
    canopy: CanopyConfig


@dataclass
class GroundMesh:
    vertices: np.ndarray  # (N, 3)
    faces: np.ndarray  # (M, 3)
    bounds_min: np.ndarray
    bounds_max: np.ndarray
    normals: np.ndarray | None = None
    x_coords: np.ndarray | None = None
    y_coords: np.ndarray | None = None
    height_grid: np.ndarray | None = None
    effective_resolution: float | None = None
    watertight: bool | None = None
    boundary_edges: int | None = None


@dataclass
class TrunkPrimitive:
    center: np.ndarray  # (3,)
    radius: float
    height: float
    support: int = 0


def load_las_xyz(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".ply":
        return load_ply_xyz(path)

    try:
        import laspy
    except Exception as exc:  # pragma: no cover - depends on runtime environment
        raise RuntimeError(
            f"laspy is required to read {path}. Install laspy or convert the input to PLY."
        ) from exc

    las = laspy.read(str(path))
    xyz = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    if xyz.size == 0:
        raise ValueError(f"empty LAS file: {path}")
    return xyz


def load_ply_xyz(path: Path) -> np.ndarray:
    type_map = {
        "char": "i1",
        "int8": "i1",
        "uchar": "u1",
        "uint8": "u1",
        "short": "i2",
        "int16": "i2",
        "ushort": "u2",
        "uint16": "u2",
        "int": "i4",
        "int32": "i4",
        "uint": "u4",
        "uint32": "u4",
        "float": "f4",
        "float32": "f4",
        "double": "f8",
        "float64": "f8",
    }

    with path.open("rb") as f:
        header_lines: list[str] = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"unexpected EOF while reading PLY header: {path}")
            decoded = line.decode("ascii", errors="strict").rstrip("\r\n")
            header_lines.append(decoded)
            if decoded == "end_header":
                break

        if not header_lines or header_lines[0] != "ply":
            raise ValueError(f"invalid PLY file: {path}")

        fmt = None
        vertex_count = None
        vertex_props: list[tuple[str, str]] = []
        current_element = None
        for line in header_lines[1:]:
            if line.startswith("format "):
                fmt = line.split()[1]
            elif line.startswith("element "):
                parts = line.split()
                current_element = parts[1]
                if current_element == "vertex":
                    vertex_count = int(parts[2])
            elif line.startswith("property ") and current_element == "vertex":
                parts = line.split()
                if parts[1] == "list":
                    raise ValueError(f"PLY vertex lists are not supported: {path}")
                vertex_props.append((parts[2], parts[1]))

        if fmt not in {"binary_little_endian", "ascii"}:
            raise ValueError(f"unsupported PLY format {fmt!r} in {path}")
        if vertex_count is None:
            raise ValueError(f"PLY file has no vertex element: {path}")

        prop_names = [name for name, _ in vertex_props]
        if not {"x", "y", "z"}.issubset(prop_names):
            raise ValueError(f"PLY vertex data does not contain x/y/z: {path}")

        dtype = np.dtype([(name, "<" + type_map[typ]) for name, typ in vertex_props])
        if fmt == "binary_little_endian":
            vertices = np.fromfile(f, dtype=dtype, count=vertex_count)
        else:
            text = f.read().decode("ascii", errors="strict").strip().splitlines()
            if len(text) < vertex_count:
                raise ValueError(f"PLY file ended early: {path}")
            rows = [line.split() for line in text[:vertex_count]]
            data = np.array(rows, dtype=np.float64)
            x_idx = prop_names.index("x")
            y_idx = prop_names.index("y")
            z_idx = prop_names.index("z")
            return data[:, [x_idx, y_idx, z_idx]].astype(np.float64)

    xyz = np.column_stack((vertices["x"], vertices["y"], vertices["z"])).astype(np.float64)
    if xyz.size == 0:
        raise ValueError(f"empty PLY file: {path}")
    return xyz


def robust_shift(ground_xyz: np.ndarray) -> np.ndarray:
    xy = np.percentile(ground_xyz[:, :2], 2.0, axis=0)
    z = np.percentile(ground_xyz[:, 2], 1.0)
    return np.array([xy[0], xy[1], z], dtype=np.float64)


def voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0:
        return xyz
    quantized = np.floor(xyz / float(voxel_size)).astype(np.int64)
    _, unique_idx = np.unique(quantized, axis=0, return_index=True)
    return xyz[np.sort(unique_idx)]


def clip_z_outliers(xyz: np.ndarray, low_q: float, high_q: float) -> np.ndarray:
    if xyz.shape[0] == 0:
        return xyz
    lo = float(np.percentile(xyz[:, 2], low_q))
    hi = float(np.percentile(xyz[:, 2], high_q))
    keep = (xyz[:, 2] >= lo) & (xyz[:, 2] <= hi)
    if not np.any(keep):
        return xyz
    return xyz[keep]


def _effective_resolution(bounds_min: np.ndarray, bounds_max: np.ndarray, resolution: float, max_dim: int) -> float:
    if max_dim <= 0:
        return float(resolution)
    span = np.asarray(bounds_max, dtype=np.float64) - np.asarray(bounds_min, dtype=np.float64)
    limit_x = float(span[0] / max(1, max_dim - 1))
    limit_y = float(span[1] / max(1, max_dim - 1))
    return float(max(resolution, limit_x, limit_y))


def nearest_fill(grid: np.ndarray) -> np.ndarray:
    valid = np.isfinite(grid)
    if not np.any(valid):
        raise ValueError("ground grid is entirely empty after binning")

    sample_y, sample_x = np.where(valid)
    sample_xy = np.column_stack((sample_x, sample_y)).astype(np.float64)
    sample_z = grid[valid].astype(np.float64)

    query_y, query_x = np.where(~valid)
    if query_x.size == 0:
        return grid

    nearest = griddata(sample_xy, sample_z, (query_x, query_y), method="nearest")
    filled = grid.copy()
    filled[~valid] = nearest
    return filled


def _fill_grid(grid: np.ndarray) -> np.ndarray:
    return nearest_fill(grid)


def build_height_grid_from_points(
    xyz: np.ndarray,
    resolution: float,
    padding: float,
    max_dim: int,
    voxel_size: float,
    clip_low_q: float,
    clip_high_q: float,
    smoothing_sigma: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.shape[0] == 0:
        raise ValueError("empty point cloud")

    xyz = clip_z_outliers(xyz, clip_low_q, clip_high_q)
    xyz = voxel_downsample(xyz, voxel_size)
    if xyz.shape[0] == 0:
        raise ValueError("point cloud vanished after downsampling / clipping")

    shift = robust_shift(xyz)
    shifted = xyz - shift

    xy = shifted[:, :2]
    z = shifted[:, 2]
    bounds_min = np.min(xy, axis=0) - padding
    bounds_max = np.max(xy, axis=0) + padding
    effective_resolution = _effective_resolution(bounds_min, bounds_max, resolution, max_dim)

    nx = int(math.ceil((bounds_max[0] - bounds_min[0]) / effective_resolution)) + 1
    ny = int(math.ceil((bounds_max[1] - bounds_min[1]) / effective_resolution)) + 1
    if nx < 2 or ny < 2:
        raise ValueError("ground extent is too small to build a mesh")

    cell_x = np.floor((xy[:, 0] - bounds_min[0]) / effective_resolution).astype(np.int32)
    cell_y = np.floor((xy[:, 1] - bounds_min[1]) / effective_resolution).astype(np.int32)
    cell_x = np.clip(cell_x, 0, nx - 1)
    cell_y = np.clip(cell_y, 0, ny - 1)

    sums = np.zeros((ny, nx), dtype=np.float64)
    counts = np.zeros((ny, nx), dtype=np.int32)
    np.add.at(sums, (cell_y, cell_x), z)
    np.add.at(counts, (cell_y, cell_x), 1)

    grid = np.full((ny, nx), np.nan, dtype=np.float64)
    valid = counts > 0
    grid[valid] = sums[valid] / counts[valid]
    grid = _fill_grid(grid)

    xs = bounds_min[0] + np.arange(nx, dtype=np.float64) * effective_resolution
    ys = bounds_min[1] + np.arange(ny, dtype=np.float64) * effective_resolution
    grid_x, grid_y = np.meshgrid(xs, ys)

    if smoothing_sigma > 0.0:
        sigma_cells = float(smoothing_sigma / max(effective_resolution, 1e-6))
        smooth = gaussian_filter(grid, sigma=sigma_cells, mode="nearest")
        grid = 0.55 * grid + 0.45 * smooth

    sample_xy = np.column_stack((grid_x[valid], grid_y[valid]))
    sample_z = grid[valid]
    if sample_xy.shape[0] >= 4:
        linear = griddata(sample_xy, sample_z, (grid_x, grid_y), method="linear")
        grid = np.where(np.isnan(linear), grid, linear)

    return xs, ys, grid, shift, float(effective_resolution)


def build_watertight_mesh(
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    height_grid: np.ndarray,
    thickness: float,
) -> tuple[np.ndarray, np.ndarray]:
    ny, nx = height_grid.shape
    if ny < 2 or nx < 2:
        raise ValueError("height grid must be at least 2x2")

    grid_x, grid_y = np.meshgrid(x_coords, y_coords)
    top_vertices = np.column_stack((grid_x.ravel(), grid_y.ravel(), height_grid.ravel()))
    bottom_vertices = top_vertices.copy()
    bottom_vertices[:, 2] -= float(thickness)
    vertices = np.vstack((top_vertices, bottom_vertices))
    offset = top_vertices.shape[0]

    def top_vid(iy: int, ix: int) -> int:
        return iy * nx + ix

    def bot_vid(iy: int, ix: int) -> int:
        return offset + iy * nx + ix

    faces: list[tuple[int, int, int]] = []
    for iy in range(ny - 1):
        for ix in range(nx - 1):
            v0 = top_vid(iy, ix)
            v1 = top_vid(iy, ix + 1)
            v2 = top_vid(iy + 1, ix)
            v3 = top_vid(iy + 1, ix + 1)
            faces.append((v0, v1, v2))
            faces.append((v1, v3, v2))

            b0 = bot_vid(iy, ix)
            b1 = bot_vid(iy, ix + 1)
            b2 = bot_vid(iy + 1, ix)
            b3 = bot_vid(iy + 1, ix + 1)
            faces.append((b0, b2, b1))
            faces.append((b1, b2, b3))

    for iy in range(ny - 1):
        t0 = top_vid(iy, 0)
        t1 = top_vid(iy + 1, 0)
        b0 = bot_vid(iy, 0)
        b1 = bot_vid(iy + 1, 0)
        faces.append((t0, t1, b1))
        faces.append((t0, b1, b0))

        t0 = top_vid(iy, nx - 1)
        t1 = top_vid(iy + 1, nx - 1)
        b0 = bot_vid(iy, nx - 1)
        b1 = bot_vid(iy + 1, nx - 1)
        faces.append((t0, b1, t1))
        faces.append((t0, b0, b1))

    for ix in range(nx - 1):
        t0 = top_vid(0, ix)
        t1 = top_vid(0, ix + 1)
        b0 = bot_vid(0, ix)
        b1 = bot_vid(0, ix + 1)
        faces.append((t0, b0, b1))
        faces.append((t0, b1, t1))

        t0 = top_vid(ny - 1, ix)
        t1 = top_vid(ny - 1, ix + 1)
        b0 = bot_vid(ny - 1, ix)
        b1 = bot_vid(ny - 1, ix + 1)
        faces.append((t0, b1, b0))
        faces.append((t0, t1, b1))

    return vertices.astype(np.float64), np.asarray(faces, dtype=np.int32)


def compute_vertex_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(vertices, dtype=np.float64)
    tris = vertices[faces]
    face_normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    np.add.at(normals, faces[:, 0], face_normals)
    np.add.at(normals, faces[:, 1], face_normals)
    np.add.at(normals, faces[:, 2], face_normals)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, np.maximum(norms, 1e-12))
    return normals


def count_boundary_edges(faces: np.ndarray) -> int:
    edges = np.vstack(
        (
            np.sort(faces[:, [0, 1]], axis=1),
            np.sort(faces[:, [1, 2]], axis=1),
            np.sort(faces[:, [2, 0]], axis=1),
        )
    )
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return int(np.sum(counts == 1))


def mesh_watertight_check(faces: np.ndarray) -> bool:
    return count_boundary_edges(faces) == 0


def build_ground_mesh(
    ground_xyz: np.ndarray,
    target_resolution: float = 0.5,
    padding: float = 4.0,
    max_dim: int = 2048,
    thickness: float = 0.5,
    voxel_size: float = 0.2,
    clip_low_q: float = 1.0,
    clip_high_q: float = 99.5,
    smoothing_sigma: float = 0.5,
) -> GroundMesh:
    xs, ys, grid, shift, effective_resolution = build_height_grid_from_points(
        ground_xyz,
        resolution=target_resolution,
        padding=padding,
        max_dim=max_dim,
        voxel_size=voxel_size,
        clip_low_q=clip_low_q,
        clip_high_q=clip_high_q,
        smoothing_sigma=smoothing_sigma,
    )
    vertices, faces_arr = build_watertight_mesh(xs, ys, grid, thickness)
    normals = compute_vertex_normals(vertices, faces_arr)
    bounds_min_3d = np.array([float(xs[0]), float(ys[0]), float(np.min(grid) - float(thickness))], dtype=np.float64)
    bounds_max_3d = np.array([float(xs[-1]), float(ys[-1]), float(np.max(grid))], dtype=np.float64)
    return GroundMesh(
        vertices=vertices,
        faces=faces_arr,
        normals=normals,
        bounds_min=bounds_min_3d,
        bounds_max=bounds_max_3d,
        x_coords=xs,
        y_coords=ys,
        height_grid=grid,
        effective_resolution=effective_resolution,
        watertight=mesh_watertight_check(faces_arr),
        boundary_edges=count_boundary_edges(faces_arr),
    )


def sample_ground_height(ground: GroundMesh, x: float, y: float) -> float:
    if ground.x_coords is None or ground.y_coords is None or ground.height_grid is None:
        raise ValueError("ground mesh does not carry sampling data")

    xs = ground.x_coords
    ys = ground.y_coords
    grid = ground.height_grid

    x = float(np.clip(x, float(xs[0]), float(xs[-1])))
    y = float(np.clip(y, float(ys[0]), float(ys[-1])))

    ix = int(np.searchsorted(xs, x, side="right") - 1)
    iy = int(np.searchsorted(ys, y, side="right") - 1)
    ix = int(np.clip(ix, 0, len(xs) - 2))
    iy = int(np.clip(iy, 0, len(ys) - 2))

    x0 = float(xs[ix])
    x1 = float(xs[ix + 1])
    y0 = float(ys[iy])
    y1 = float(ys[iy + 1])
    tx = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
    ty = 0.0 if y1 == y0 else (y - y0) / (y1 - y0)

    z00 = float(grid[iy, ix])
    z10 = float(grid[iy, ix + 1])
    z01 = float(grid[iy + 1, ix])
    z11 = float(grid[iy + 1, ix + 1])
    z0 = z00 * (1.0 - tx) + z10 * tx
    z1 = z01 * (1.0 - tx) + z11 * tx
    return float(z0 * (1.0 - ty) + z1 * ty)


def prune_overlapping_trunks(
    trunks: list[TrunkPrimitive],
    min_center_distance: float = 0.75,
    overlap_factor: float = 0.9,
) -> list[TrunkPrimitive]:
    if not trunks:
        return []

    scored = sorted(
        trunks,
        key=lambda trunk: (
            -int(trunk.support),
            -(float(trunk.radius) * float(trunk.height)),
            float(trunk.center[0]),
            float(trunk.center[1]),
            float(trunk.center[2]),
        ),
    )
    kept: list[TrunkPrimitive] = []
    for trunk in scored:
        xy = np.asarray(trunk.center[:2], dtype=np.float64)
        too_close = False
        for other in kept:
            other_xy = np.asarray(other.center[:2], dtype=np.float64)
            dist = float(np.linalg.norm(xy - other_xy))
            if dist < min_center_distance:
                too_close = True
                break
            if dist < float(overlap_factor) * float(trunk.radius + other.radius):
                too_close = True
                break
        if not too_close:
            kept.append(trunk)

    return kept


def cluster_trunks(
    trunk_xyz: np.ndarray,
    z_scale: float = 0.05,
    voxel_size: float = 0.15,
    cluster_cell_size: float = 0.8,
    min_points: int = 20,
    min_cluster_size: int = 60,
    radius_scale: float = 1.0,
    height_scale: float = 1.08,
    radius_min: float = 0.12,
    radius_max: float = 0.45,
    height_min: float = 0.8,
    height_max: float = 12.0,
    min_center_distance: float = 0.75,
    overlap_factor: float = 0.9,
) -> list[TrunkPrimitive]:
    if trunk_xyz.shape[0] == 0:
        return []

    pts = trunk_xyz.astype(np.float64)
    if voxel_size > 0:
        q = np.floor(pts / float(voxel_size)).astype(np.int32)
        _, unique_idx = np.unique(q, axis=0, return_index=True)
        pts = pts[np.sort(unique_idx)]

    if pts.shape[0] == 0:
        return []

    primitives: list[TrunkPrimitive] = []
    xy = pts[:, :2]
    z = pts[:, 2]
    cell_size = max(float(cluster_cell_size), 0.25)
    origin = np.min(xy, axis=0)
    cell_xy = np.floor((xy - origin) / cell_size).astype(np.int32)

    cells, inverse, counts = np.unique(cell_xy, axis=0, return_inverse=True, return_counts=True)
    if cells.shape[0] == 0:
        return []

    cell_to_idx = {tuple(cell): i for i, cell in enumerate(cells)}
    parent = np.arange(cells.shape[0], dtype=np.int32)
    rank = np.zeros(cells.shape[0], dtype=np.int32)

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a: int, b: int) -> None:
        ra = find(a)
        rb = find(b)
        if ra == rb:
            return
        if rank[ra] < rank[rb]:
            parent[ra] = rb
        elif rank[ra] > rank[rb]:
            parent[rb] = ra
        else:
            parent[rb] = ra
            rank[ra] += 1

    neighbor_offsets = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    ]
    for idx, cell in enumerate(cells):
        if counts[idx] < min_points:
            continue
        x, y = int(cell[0]), int(cell[1])
        for dx, dy in neighbor_offsets:
            neighbor = (x + dx, y + dy)
            j = cell_to_idx.get(neighbor)
            if j is not None and counts[j] >= min_points:
                union(idx, j)

    component_to_cells: dict[int, list[int]] = {}
    for idx in range(cells.shape[0]):
        if counts[idx] < min_points:
            continue
        root = find(idx)
        component_to_cells.setdefault(root, []).append(idx)

    for cell_indices in component_to_cells.values():
        point_mask = np.isin(inverse, cell_indices)
        cluster = pts[point_mask]
        if cluster.shape[0] < min_cluster_size:
            continue

        center_xy = np.median(cluster[:, :2], axis=0)
        radial = np.linalg.norm(cluster[:, :2] - center_xy, axis=1)
        radius = float(np.percentile(radial, 95) * radius_scale)
        radius = float(np.clip(radius, radius_min, radius_max))

        z0 = float(np.percentile(cluster[:, 2], 2))
        z1 = float(np.percentile(cluster[:, 2], 98))
        height = float(np.clip((z1 - z0) * height_scale, height_min, height_max))
        center = np.array([center_xy[0], center_xy[1], z0 + height / 2.0], dtype=np.float64)
        primitives.append(
            TrunkPrimitive(
                center=center,
                radius=radius,
                height=height,
                support=int(cluster.shape[0]),
            )
        )

    return prune_overlapping_trunks(
        primitives,
        min_center_distance=min_center_distance,
        overlap_factor=overlap_factor,
    )


def _lazy_import_usd():
    global _SIM_APP
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
    except Exception:
        # Isaac Sim often needs a SimulationApp instance before pxr becomes importable.
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
        _SIM_APP = app
        try:
            from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
        except Exception as exc:  # pragma: no cover - depends on Isaac Sim runtime
            app.close()
            raise RuntimeError(
                "pxr is not available even after starting SimulationApp. "
                "Run this script inside the Isaac Sim Python environment."
            ) from exc
        return Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, app
    return Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, None


def _set_display_color(gprim, color: tuple[float, float, float]) -> None:
    gprim.CreateDisplayColorAttr([color])


def _bind_preview_material(
    stage,
    UsdShade,
    Sdf,
    Gf,
    prim,
    material_name: str,
    diffuse: tuple[float, float, float],
    roughness: float,
    binding_strength: str | None = None,
) -> None:
    looks_path = Sdf.Path("/World/Looks")
    looks_prim = stage.GetPrimAtPath(str(looks_path))
    if not looks_prim.IsValid():
        from pxr import UsdGeom  # local import to avoid circular dependency in type hints

        UsdGeom.Scope.Define(stage, looks_path)

    material_path = looks_path.AppendChild(material_name)
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path.AppendChild("PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    if binding_strength is None:
        UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(material)
    else:
        UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(material, bindingStrength=binding_strength)


def _apply_static_collision(UsdPhysics, prim, approximation: str | None = None) -> None:
    from pxr import UsdGeom

    prim_obj = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim_obj)
    if prim_obj.IsA(UsdGeom.Mesh) and approximation is not None:
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim_obj)
        mesh_collision.CreateApproximationAttr().Set(str(approximation))


def _ensure_physics_scene(stage, UsdPhysics, Sdf) -> None:
    scene_path = Sdf.Path("/World/PhysicsScene")
    if stage.GetPrimAtPath(scene_path).IsValid():
        return
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)


def _safe_relativize(path: Path, base: Path) -> str:
    try:
        return str(path.resolve().relative_to(base.resolve()))
    except Exception:
        return str(path)


def _choose_canopy_prototype(
    trunks: list[TrunkPrimitive],
    idx: int,
    prototype_paths: list[Path],
    canopy_cfg: CanopyConfig,
    seed: int,
) -> tuple[Path, float, float, tuple[float, float, float]]:
    rng = np.random.default_rng(seed + idx * 9973)
    proto = prototype_paths[int(rng.integers(0, len(prototype_paths)))]
    trunk = trunks[idx]

    radius_scale = float(rng.uniform(canopy_cfg.radius_scale_min, canopy_cfg.radius_scale_max))
    height_scale = float(rng.uniform(canopy_cfg.height_scale_min, canopy_cfg.height_scale_max))
    base_scale = max(float(trunk.radius) * radius_scale, float(trunk.height) * 0.08 * height_scale)
    factor = float(rng.uniform(canopy_cfg.scale_min, canopy_cfg.scale_max))
    uniform_scale = float(np.clip(base_scale * factor, 0.9, 4.6))
    z_jitter = float(rng.uniform(canopy_cfg.z_offset_min, canopy_cfg.z_offset_max))
    diffuse = (
        float(np.clip(CANOPY_GREEN[0] + rng.uniform(-canopy_cfg.color_jitter_r, canopy_cfg.color_jitter_r), 0.05, 0.30)),
        float(np.clip(CANOPY_GREEN[1] + rng.uniform(-canopy_cfg.color_jitter_g, canopy_cfg.color_jitter_g), 0.18, 0.42)),
        float(np.clip(CANOPY_GREEN[2] + rng.uniform(-canopy_cfg.color_jitter_b, canopy_cfg.color_jitter_b), 0.04, 0.28)),
    )
    return proto, uniform_scale, z_jitter, diffuse


@lru_cache(maxsize=None)
def _prototype_z_bounds(path_str: str) -> tuple[float, float]:
    Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, _ = _lazy_import_usd()
    _ = Gf, Sdf, UsdLux, UsdPhysics, UsdShade

    stage = Usd.Stage.Open(path_str)
    if stage is None:
        raise RuntimeError(f"failed to open canopy prototype: {path_str}")

    root = stage.GetDefaultPrim()
    if not root or not root.IsValid():
        children = [child for child in stage.GetPseudoRoot().GetChildren() if child.IsValid()]
        if not children:
            raise RuntimeError(f"canopy prototype has no valid root prim: {path_str}")
        root = children[0]

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    world_range = bbox_cache.ComputeWorldBound(root).ComputeAlignedRange()
    z_min = float(world_range.GetMin()[2])
    z_max = float(world_range.GetMax()[2])
    if not math.isfinite(z_min) or not math.isfinite(z_max):
        raise RuntimeError(f"invalid canopy bounds for {path_str}")
    return z_min, z_max


GROUND_BROWN = (0.24, 0.17, 0.09)
TRUNK_BROWN = (0.47, 0.30, 0.15)
CANOPY_GREEN = (0.14, 0.31, 0.12)


def _canopy_translate_z(trunk: TrunkPrimitive, canopy_height_offset: float, canopy_overlap: float) -> float:
    trunk_top = float(trunk.center[2] + trunk.height * 0.5)
    return float(trunk_top + trunk.height * (canopy_height_offset - 0.5) - canopy_overlap)


def write_usd(
    output_path: Path,
    ground: GroundMesh,
    trunks: list[TrunkPrimitive],
    ground_cfg: GroundConfig | None = None,
    trunk_cfg: TrunkConfig | None = None,
    canopy_cfg: CanopyConfig | None = None,
    add_canopy: bool = False,
    canopy_seed: int = 13,
    manifest_path: Path | None = None,
    manifest_source_dir: Path | None = None,
    manifest_shift: np.ndarray | None = None,
    manifest_options: dict | None = None,
) -> list[dict[str, object]]:
    Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, sim_app = _lazy_import_usd()
    ground_cfg = ground_cfg or GroundConfig()
    trunk_cfg = trunk_cfg or TrunkConfig()
    canopy_cfg = canopy_cfg or CanopyConfig(prototype_paths=default_canopy_prototypes())

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    _ensure_physics_scene(stage, UsdPhysics, Sdf)

    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    # Light and a simple sky-like background.
    dome = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Sun"))
    dome.CreateIntensityAttr(50000.0)
    dome.CreateAngleAttr(0.53)
    dome_xform = UsdGeom.XformCommonAPI(dome)
    dome_xform.SetRotate((-55.0, 0.0, 35.0))

    UsdGeom.Scope.Define(stage, Sdf.Path("/World/Looks"))

    ground_prim = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/Ground"))
    ground_prim.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in ground.vertices])
    ground_prim.CreateFaceVertexCountsAttr([3] * int(ground.faces.shape[0]))
    ground_prim.CreateFaceVertexIndicesAttr(ground.faces.astype(np.int32).ravel().tolist())
    if ground.normals is not None:
        ground_prim.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in ground.normals])
        ground_prim.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    ground_prim.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    ground_prim.CreateDoubleSidedAttr(False)
    ground_prim.CreateSubdivisionSchemeAttr("none")
    _bind_preview_material(stage, UsdShade, Sdf, Gf, ground_prim, "GroundMat", GROUND_BROWN, 0.98)
    _apply_static_collision(UsdPhysics, ground_prim, approximation=ground_cfg.collider_approximation)

    trees_scope = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Trees"))

    canopy_proto_paths = [Path(p) for p in canopy_cfg.prototype_paths if Path(p).exists()]
    tree_records: list[dict[str, object]] = []

    for idx, trunk in enumerate(trunks):
        trunk_height = float(np.clip(trunk.height, trunk_cfg.height_min, trunk_cfg.height_max))
        trunk_radius = float(np.clip(trunk.radius, trunk_cfg.radius_min, trunk_cfg.radius_max))
        ground_z = sample_ground_height(ground, float(trunk.center[0]), float(trunk.center[1]))
        trunk_base_z = float(ground_z + trunk_cfg.ground_offset)
        trunk_center_z = float(trunk_base_z + trunk_height * 0.5)
        canopy_proto_name = None
        canopy_scale_value = None
        canopy_translate_z = None
        canopy_world_top = None
        if add_canopy:
            canopy_path = Sdf.Path(f"/World/Trees/Tree_{idx:04d}/Canopy")
            canopy_prim = UsdGeom.Xform.Define(stage, canopy_path)
            canopy_xform = UsdGeom.XformCommonAPI(canopy_prim)
            canopy_proto_name = None
            canopy_translate_z = None

            if canopy_proto_paths:
                proto_path, uniform_scale, z_jitter, diffuse = _choose_canopy_prototype(
                    trunks,
                    idx,
                    canopy_proto_paths,
                    canopy_cfg,
                    canopy_seed,
                )
                canopy_proto_name = str(proto_path)
                canopy_scale_value = float(uniform_scale)
                canopy_xform.SetScale((canopy_scale_value, canopy_scale_value, canopy_scale_value))
                canopy_xform.SetRotate((0.0, 0.0, float((idx * 37) % 360)))
                canopy_prim.GetPrim().GetReferences().AddReference(str(proto_path))
                canopy_prim.GetPrim().SetInstanceable(True)
                _canopy_min_z, canopy_max_z = _prototype_z_bounds(str(proto_path))
                canopy_shape_margin = max(float(trunk_height) * 0.15, float(trunk_radius) * 4.0)
                canopy_world_top = float(
                    trunk_base_z
                    + trunk_height
                    + canopy_cfg.trunk_canopy_top_gap
                    + canopy_shape_margin
                )
                canopy_translate_z = float(canopy_world_top - canopy_max_z * canopy_scale_value - z_jitter)
                canopy_xform.SetTranslate(
                    (
                        float(trunk.center[0]),
                        float(trunk.center[1]),
                        float(canopy_translate_z),
                    )
                )
                _bind_preview_material(
                    stage,
                    UsdShade,
                    Sdf,
                    Gf,
                    canopy_prim,
                    "CanopyMat",
                    diffuse,
                    0.95,
                    binding_strength=UsdShade.Tokens.strongerThanDescendants,
                )
            else:
                canopy_prim = UsdGeom.Sphere.Define(stage, canopy_path)
                canopy_radius = float(max(trunk_radius * canopy_cfg.fallback_radius_multiplier, canopy_cfg.fallback_radius_floor))
                canopy_prim.CreateRadiusAttr(canopy_radius)
                canopy_xform = UsdGeom.XformCommonAPI(canopy_prim)
                canopy_shape_margin = max(float(trunk_height) * 0.15, canopy_radius)
                canopy_world_top = float(
                    trunk_base_z
                    + trunk_height
                    + canopy_cfg.trunk_canopy_top_gap
                    + canopy_shape_margin
                )
                canopy_translate_z = float(canopy_world_top - canopy_radius)
                canopy_xform.SetTranslate((float(trunk.center[0]), float(trunk.center[1]), canopy_translate_z))
                canopy_world_top = float(canopy_translate_z + canopy_radius)
                _bind_preview_material(stage, UsdShade, Sdf, Gf, canopy_prim, "CanopyMat", CANOPY_GREEN, 0.88)

            desired_trunk_top = float(canopy_world_top - canopy_cfg.trunk_canopy_top_gap)
            if desired_trunk_top > trunk_base_z + trunk_height:
                trunk_height = float(np.clip(desired_trunk_top - trunk_base_z, trunk_cfg.height_min, trunk_cfg.height_max))
                trunk_center_z = trunk_base_z + trunk_height * 0.5

        trunk_path = Sdf.Path(f"/World/Trees/Tree_{idx:04d}/Trunk")
        trunk_prim = UsdGeom.Cylinder.Define(stage, trunk_path)
        trunk_prim.CreateHeightAttr(float(trunk_height))
        trunk_prim.CreateRadiusAttr(float(trunk_radius))
        trunk_prim.CreateAxisAttr(UsdGeom.Tokens.z)
        trunk_xform = UsdGeom.XformCommonAPI(trunk_prim)
        trunk_xform.SetTranslate(
            (float(trunk.center[0]), float(trunk.center[1]), float(trunk_center_z))
        )
        _bind_preview_material(stage, UsdShade, Sdf, Gf, trunk_prim, "TrunkMat", TRUNK_BROWN, 0.92)
        _apply_static_collision(UsdPhysics, trunk_prim)

        tree_records.append(
            {
                "index": int(idx),
                "trunk": {
                    "center": [float(trunk.center[0]), float(trunk.center[1]), float(trunk_center_z)],
                    "radius": float(trunk_radius),
                    "height": float(trunk_height),
                    "support": int(trunk.support),
                    "ground_z": float(ground_z),
                    "base_z": float(trunk_base_z),
                },
                "canopy": {
                    "prototype": canopy_proto_name,
                    "scale": None if canopy_scale_value is None else float(canopy_scale_value),
                    "translate_z": None if canopy_translate_z is None else float(canopy_translate_z),
                    "world_top": None if canopy_world_top is None else float(canopy_world_top),
                },
            }
        )

    # Keep the tree scope authored even if it only contains children.
    stage.GetPrimAtPath("/World/Trees").SetActive(True)
    stage.Save()

    if manifest_path is not None and manifest_source_dir is not None and manifest_shift is not None:
        manifest = make_manifest(
            source_dir=manifest_source_dir,
            shift=np.asarray(manifest_shift, dtype=np.float64),
            ground=ground,
            ground_cfg=ground_cfg,
            trunks=trunks,
            tree_records=tree_records,
            output_path=output_path,
            add_canopy=add_canopy,
            options={} if manifest_options is None else dict(manifest_options),
        )
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    return tree_records


def build_cache(
    *,
    source_dir: Path,
    ground_path: Path,
    trunk_path: Path,
    canopy_path: Path | None,
    ground_cfg: GroundConfig,
    trunk_cfg: TrunkConfig,
    dbscan_min_points: int,
    min_cluster_size: int,
) -> dict:
    ground_xyz = load_las_xyz(ground_path)
    trunk_xyz = load_las_xyz(trunk_path)

    shift = robust_shift(ground_xyz)
    ground_xyz = ground_xyz - shift
    trunk_xyz = trunk_xyz - shift

    canopy_count = None
    if canopy_path is not None and canopy_path.exists():
        canopy_count = int(load_las_xyz(canopy_path).shape[0])

    ground_mesh = build_ground_mesh(
        ground_xyz,
        target_resolution=ground_cfg.resolution,
        padding=ground_cfg.padding,
        max_dim=ground_cfg.max_dim,
        thickness=ground_cfg.thickness,
        voxel_size=ground_cfg.voxel_size,
        clip_low_q=ground_cfg.clip_low_q,
        clip_high_q=ground_cfg.clip_high_q,
        smoothing_sigma=ground_cfg.smoothing_sigma,
    )
    trunks = cluster_trunks(
        trunk_xyz,
        z_scale=0.0,
        voxel_size=trunk_cfg.voxel_size,
        cluster_cell_size=trunk_cfg.cluster_cell_size,
        min_points=max(1, int(dbscan_min_points)),
        min_cluster_size=max(1, int(min_cluster_size)),
        radius_scale=trunk_cfg.radius_scale,
        height_scale=trunk_cfg.height_scale,
        radius_min=trunk_cfg.radius_min,
        radius_max=trunk_cfg.radius_max,
        height_min=trunk_cfg.height_min,
        height_max=trunk_cfg.height_max,
        min_center_distance=trunk_cfg.prune_min_center_distance,
        overlap_factor=trunk_cfg.prune_overlap_factor,
    )

    cache = {
        "source_dir": str(source_dir),
        "shift": shift.astype(np.float64),
        "ground_vertices": ground_mesh.vertices.astype(np.float32),
        "ground_faces": ground_mesh.faces.astype(np.int32),
        "ground_bounds_min": ground_mesh.bounds_min.astype(np.float64),
        "ground_bounds_max": ground_mesh.bounds_max.astype(np.float64),
        "ground_normals": np.asarray(ground_mesh.normals if ground_mesh.normals is not None else np.zeros_like(ground_mesh.vertices), dtype=np.float32),
        "ground_effective_resolution": np.array([
            -1.0 if ground_mesh.effective_resolution is None else float(ground_mesh.effective_resolution)
        ], dtype=np.float64),
        "ground_watertight": np.array([1 if ground_mesh.watertight else 0], dtype=np.int32),
        "ground_boundary_edges": np.array([int(ground_mesh.boundary_edges or 0)], dtype=np.int32),
        "ground_thickness": np.array([ground_cfg.thickness], dtype=np.float64),
        "ground_x_coords": np.asarray(ground_mesh.x_coords, dtype=np.float64),
        "ground_y_coords": np.asarray(ground_mesh.y_coords, dtype=np.float64),
        "ground_height_grid": np.asarray(ground_mesh.height_grid, dtype=np.float32),
        "trunk_centers": np.asarray([t.center for t in trunks], dtype=np.float32),
        "trunk_radii": np.asarray([t.radius for t in trunks], dtype=np.float32),
        "trunk_heights": np.asarray([t.height for t in trunks], dtype=np.float32),
        "trunk_supports": np.asarray([t.support for t in trunks], dtype=np.int32),
        "canopy_count": np.array([-1 if canopy_count is None else canopy_count], dtype=np.int32),
    }
    return cache


def write_cache(cache_path: Path, cache: dict) -> None:
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache_path, **cache)


def load_cache(cache_path: Path) -> dict:
    raw = np.load(cache_path, allow_pickle=True)
    cache = {key: raw[key] for key in raw.files}
    return cache


def cache_to_ground(cache: dict) -> GroundMesh:
    normals = cache.get("ground_normals", None)
    return GroundMesh(
        vertices=np.asarray(cache["ground_vertices"], dtype=np.float64),
        faces=np.asarray(cache["ground_faces"], dtype=np.int32),
        bounds_min=np.asarray(cache["ground_bounds_min"], dtype=np.float64),
        bounds_max=np.asarray(cache["ground_bounds_max"], dtype=np.float64),
        normals=None if normals is None else np.asarray(normals, dtype=np.float64),
        x_coords=np.asarray(cache["ground_x_coords"], dtype=np.float64),
        y_coords=np.asarray(cache["ground_y_coords"], dtype=np.float64),
        height_grid=np.asarray(cache["ground_height_grid"], dtype=np.float64),
        effective_resolution=float(np.asarray(cache.get("ground_effective_resolution", np.array([-1.0]))).reshape(-1)[0]),
        watertight=bool(int(np.asarray(cache.get("ground_watertight", np.array([0]))).reshape(-1)[0])),
        boundary_edges=int(np.asarray(cache.get("ground_boundary_edges", np.array([0]))).reshape(-1)[0]),
    )


def cache_to_trunks(cache: dict) -> list[TrunkPrimitive]:
    centers = np.asarray(cache["trunk_centers"], dtype=np.float64)
    radii = np.asarray(cache["trunk_radii"], dtype=np.float64).reshape(-1)
    heights = np.asarray(cache["trunk_heights"], dtype=np.float64).reshape(-1)
    supports = np.asarray(cache.get("trunk_supports", np.zeros_like(radii, dtype=np.int32)), dtype=np.int32).reshape(-1)
    trunks = []
    for center, radius, height, support in zip(centers, radii, heights, supports):
        trunks.append(
            TrunkPrimitive(
                center=np.asarray(center, dtype=np.float64),
                radius=float(radius),
                height=float(height),
                support=int(support),
            )
        )
    return trunks


def make_manifest(
    *,
    source_dir: Path,
    shift: np.ndarray,
    ground: GroundMesh,
    ground_cfg: GroundConfig,
    trunks: list[TrunkPrimitive],
    tree_records: list[dict[str, object]],
    output_path: Path,
    add_canopy: bool,
    options: dict,
) -> dict:
    return {
        "source_dir": str(source_dir),
        "output_usd": str(output_path),
        "shift": [float(v) for v in shift.tolist()],
        "ground_bounds_min": [float(v) for v in ground.bounds_min.tolist()],
        "ground_bounds_max": [float(v) for v in ground.bounds_max.tolist()],
        "ground_thickness": float(ground_cfg.thickness),
        "ground_effective_resolution": None if ground.effective_resolution is None else float(ground.effective_resolution),
        "ground_boundary_edges": int(ground.boundary_edges or 0),
        "ground_watertight": bool(ground.watertight) if ground.watertight is not None else None,
        "trunk_count": len(trunks),
        "add_canopy": add_canopy,
        "options": options,
        "trunks": [
            {
                "center": [float(v) for v in trunk.center.tolist()],
                "radius": float(trunk.radius),
                "height": float(trunk.height),
                "support": int(trunk.support),
            }
            for trunk in trunks
        ],
        "trees": tree_records,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate an Isaac Sim forest digital twin from point cloud files.")
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=DEFAULT_EXAMPLE_POINTCLOUD_DIR,
        help="Directory that contains the example PLY files.",
    )
    parser.add_argument(
        "--ground",
        type=Path,
        default=Path("porvoo-20250520-000013_ground_sec.ply"),
        help="Ground PLY file, relative to --source-dir unless absolute.",
    )
    parser.add_argument(
        "--trunk",
        type=Path,
        default=Path("porvoo-20250520-000013_trunk_sec.ply"),
        help="Trunk PLY file, relative to --source-dir unless absolute.",
    )
    parser.add_argument(
        "--canopy",
        type=Path,
        default=Path("porvoo-20250520-000013_canopy_sec.ply"),
        help="Canopy PLY file, relative to --source-dir unless absolute.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_DIGITAL_TWIN_DIR,
        help="Directory that will receive the generated USD, cache, and manifest.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional USD output path. Relative paths are resolved under --output-dir.",
    )
    parser.add_argument(
        "--cache",
        type=Path,
        default=None,
        help="Optional cache output path. Relative paths are resolved under --output-dir.",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Optional JSON manifest path. Relative paths are resolved under --output-dir.",
    )
    parser.add_argument("--ground-resolution", "--grid-resolution", dest="ground_resolution", type=float, default=0.5)
    parser.add_argument("--ground-padding", "--grid-padding", dest="ground_padding", type=float, default=4.0)
    parser.add_argument("--ground-max-dim", "--grid-max-dim", dest="ground_max_dim", type=int, default=2048)
    parser.add_argument("--ground-thickness", type=float, default=0.5)
    parser.add_argument(
        "--ground-collider-approximation",
        type=str,
        default="sdf",
        choices=["none", "sdf", "meshSimplification"],
    )
    parser.add_argument("--ground-voxel-size", type=float, default=0.2)
    parser.add_argument("--ground-clip-low-q", type=float, default=1.0)
    parser.add_argument("--ground-clip-high-q", type=float, default=99.5)
    parser.add_argument("--ground-smoothing-sigma", type=float, default=0.5)
    parser.add_argument("--cluster-cell-size", type=float, default=None, help="XY cell size used when grouping trunk points.")
    parser.add_argument("--dbscan-eps", type=float, default=0.8, help="Legacy alias used when --cluster-cell-size is not set.")
    parser.add_argument("--dbscan-min-points", type=int, default=20)
    parser.add_argument("--min-cluster-size", type=int, default=60)
    parser.add_argument("--voxel-size", type=float, default=0.15)
    parser.add_argument("--z-scale", type=float, default=0.05)
    parser.add_argument("--trunk-radius-min", type=float, default=0.12)
    parser.add_argument("--trunk-radius-max", type=float, default=0.45)
    parser.add_argument("--trunk-height-min", type=float, default=0.8)
    parser.add_argument("--trunk-height-max", type=float, default=12.0)
    parser.add_argument("--trunk-prune-min-distance", type=float, default=0.75)
    parser.add_argument("--trunk-prune-overlap-factor", type=float, default=0.9)
    parser.add_argument("--trunk-ground-offset", type=float, default=0.02)
    parser.add_argument("--add-canopy", action="store_true")
    parser.add_argument(
        "--canopy-prototype",
        type=Path,
        action="append",
        default=None,
        help="USD canopy prototype to reference. Can be passed multiple times.",
    )
    parser.add_argument("--canopy-seed", type=int, default=13, help="Seed used for canopy prototype selection.")
    parser.add_argument("--canopy-scale-min", type=float, default=0.95)
    parser.add_argument("--canopy-scale-max", type=float, default=1.15)
    parser.add_argument("--canopy-radius-scale-min", type=float, default=2.4)
    parser.add_argument("--canopy-radius-scale-max", type=float, default=4.2)
    parser.add_argument("--canopy-height-scale-min", type=float, default=0.70)
    parser.add_argument("--canopy-height-scale-max", type=float, default=1.18)
    parser.add_argument("--canopy-z-offset-min", type=float, default=-0.18)
    parser.add_argument("--canopy-z-offset-max", type=float, default=0.25)
    parser.add_argument("--trunk-canopy-top-gap", type=float, default=1.0)
    parser.add_argument("--canopy-color-jitter", type=float, default=0.05, help="Symmetric RGB jitter used for canopy material coloration.")
    parser.add_argument(
        "--preprocess-only",
        action="store_true",
        help="Read LAS inputs and write a cache, but do not write USD.",
    )
    parser.add_argument(
        "--from-cache",
        type=Path,
        default=None,
        help="Load a preprocessed cache and skip LAS parsing / clustering.",
    )
    return parser.parse_args()


def resolve_input(base: Path, p: Path) -> Path:
    return p if p.is_absolute() else base / p


def resolve_output(base: Path, path: Path | None, default_name: str) -> Path:
    if path is None:
        return base / default_name
    return path if path.is_absolute() else base / path


def build_forest_config(args: argparse.Namespace, canopy_prototypes: list[Path]) -> ForestConfig:
    ground_cfg = GroundConfig(
        resolution=float(args.ground_resolution),
        thickness=float(args.ground_thickness),
        padding=float(args.ground_padding),
        max_dim=int(args.ground_max_dim),
        voxel_size=float(args.ground_voxel_size),
        clip_low_q=float(args.ground_clip_low_q),
        clip_high_q=float(args.ground_clip_high_q),
        smoothing_sigma=float(args.ground_smoothing_sigma),
        collider_approximation=str(args.ground_collider_approximation),
    )
    trunk_cfg = TrunkConfig(
        voxel_size=float(args.voxel_size),
        cluster_cell_size=float(args.cluster_cell_size if args.cluster_cell_size is not None else args.dbscan_eps),
        min_points=int(args.dbscan_min_points),
        min_cluster_size=int(args.min_cluster_size),
        radius_scale=1.0,
        height_scale=1.08,
        radius_min=float(args.trunk_radius_min),
        radius_max=float(args.trunk_radius_max),
        height_min=float(args.trunk_height_min),
        height_max=float(args.trunk_height_max),
        prune_min_center_distance=float(args.trunk_prune_min_distance),
        prune_overlap_factor=float(args.trunk_prune_overlap_factor),
        ground_offset=float(args.trunk_ground_offset),
    )
    canopy_cfg = CanopyConfig(
        prototype_paths=[Path(p) for p in canopy_prototypes],
        scale_min=float(args.canopy_scale_min),
        scale_max=float(args.canopy_scale_max),
        radius_scale_min=float(args.canopy_radius_scale_min),
        radius_scale_max=float(args.canopy_radius_scale_max),
        height_scale_min=float(args.canopy_height_scale_min),
        height_scale_max=float(args.canopy_height_scale_max),
        z_offset_min=float(args.canopy_z_offset_min),
        z_offset_max=float(args.canopy_z_offset_max),
        trunk_canopy_top_gap=float(args.trunk_canopy_top_gap),
        color_jitter_r=float(args.canopy_color_jitter),
        color_jitter_g=float(args.canopy_color_jitter),
        color_jitter_b=float(args.canopy_color_jitter),
    )
    return ForestConfig(ground=ground_cfg, trunk=trunk_cfg, canopy=canopy_cfg)


def main() -> int:
    global _SIM_APP
    args = parse_args()
    try:
        source_dir = args.source_dir.resolve()
        canopy_prototypes = args.canopy_prototype if args.canopy_prototype is not None else default_canopy_prototypes()
        forest_cfg = build_forest_config(args, canopy_prototypes)

        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = resolve_output(output_dir, args.output, "forest_demo.usda")
        cache_path = resolve_output(output_dir, args.cache, "forest_demo_cache.npz")
        manifest_path = resolve_output(output_dir, args.manifest, "forest_demo_manifest.json")

        ground_path = resolve_input(source_dir, args.ground)
        trunk_path = resolve_input(source_dir, args.trunk)
        canopy_path = resolve_input(source_dir, args.canopy)

        if args.from_cache is not None:
            cache_path = args.from_cache.resolve()

        if args.from_cache is None:
            if not ground_path.exists():
                raise FileNotFoundError(f"missing ground LAS: {ground_path}")
            if not trunk_path.exists():
                raise FileNotFoundError(f"missing trunk LAS: {trunk_path}")

            cache = build_cache(
                source_dir=source_dir,
                ground_path=ground_path,
                trunk_path=trunk_path,
                canopy_path=canopy_path if canopy_path.exists() else None,
                ground_cfg=forest_cfg.ground,
                trunk_cfg=forest_cfg.trunk,
                dbscan_min_points=args.dbscan_min_points,
                min_cluster_size=args.min_cluster_size,
            )
            write_cache(cache_path, cache)
            if args.preprocess_only:
                print(f"Wrote cache: {cache_path}")
                return 0
        else:
            if not cache_path.exists():
                raise FileNotFoundError(f"missing cache: {cache_path}")
            cache = load_cache(cache_path)

        ground_mesh = cache_to_ground(cache)
        trunks = cache_to_trunks(cache)
        shift = np.asarray(cache["shift"], dtype=np.float64)

        tree_records = write_usd(
            output_path,
            ground_mesh,
            trunks,
            ground_cfg=forest_cfg.ground,
            trunk_cfg=forest_cfg.trunk,
            canopy_cfg=forest_cfg.canopy,
            add_canopy=args.add_canopy,
            canopy_seed=args.canopy_seed,
            manifest_path=manifest_path,
            manifest_source_dir=source_dir,
            manifest_shift=shift,
            manifest_options={
                "ground_resolution": forest_cfg.ground.resolution,
                "ground_padding": forest_cfg.ground.padding,
                "ground_max_dim": forest_cfg.ground.max_dim,
                "ground_thickness": forest_cfg.ground.thickness,
                "ground_collider_approximation": forest_cfg.ground.collider_approximation,
                "ground_voxel_size": forest_cfg.ground.voxel_size,
                "ground_clip_low_q": forest_cfg.ground.clip_low_q,
                "ground_clip_high_q": forest_cfg.ground.clip_high_q,
                "ground_smoothing_sigma": forest_cfg.ground.smoothing_sigma,
                "grid_resolution": forest_cfg.ground.resolution,
                "grid_padding": forest_cfg.ground.padding,
                "grid_max_dim": forest_cfg.ground.max_dim,
                "cluster_cell_size": forest_cfg.trunk.cluster_cell_size,
                "dbscan_eps": args.dbscan_eps,
                "dbscan_min_points": args.dbscan_min_points,
                "min_cluster_size": args.min_cluster_size,
                "voxel_size": forest_cfg.trunk.voxel_size,
                "trunk_radius_min": forest_cfg.trunk.radius_min,
                "trunk_radius_max": forest_cfg.trunk.radius_max,
                "trunk_height_min": forest_cfg.trunk.height_min,
                "trunk_height_max": forest_cfg.trunk.height_max,
                "trunk_prune_min_distance": forest_cfg.trunk.prune_min_center_distance,
                "trunk_prune_overlap_factor": forest_cfg.trunk.prune_overlap_factor,
                "trunk_ground_offset": forest_cfg.trunk.ground_offset,
                "canopy_prototypes": [str(p) for p in canopy_prototypes],
                "canopy_seed": args.canopy_seed,
                "canopy_scale_min": forest_cfg.canopy.scale_min,
                "canopy_scale_max": forest_cfg.canopy.scale_max,
                "canopy_radius_scale_min": forest_cfg.canopy.radius_scale_min,
                "canopy_radius_scale_max": forest_cfg.canopy.radius_scale_max,
                "canopy_height_scale_min": forest_cfg.canopy.height_scale_min,
                "canopy_height_scale_max": forest_cfg.canopy.height_scale_max,
                "canopy_z_offset_min": forest_cfg.canopy.z_offset_min,
                "canopy_z_offset_max": forest_cfg.canopy.z_offset_max,
                "trunk_canopy_top_gap": forest_cfg.canopy.trunk_canopy_top_gap,
                "canopy_color_jitter": args.canopy_color_jitter,
            },
        )

        print(f"Wrote USD: {output_path}")
        print(f"Wrote manifest: {manifest_path}")
        print(f"Wrote cache: {cache_path}")
        print(f"Ground cells: {ground_mesh.vertices.shape[0]} vertices")
        print(f"Tree trunks: {len(trunks)}")
        canopy_count = int(np.asarray(cache.get("canopy_count", np.array([-1]))).reshape(-1)[0])
        if canopy_count >= 0:
            print(f"Canopy points available: {canopy_count}")
        return 0
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
            _SIM_APP = None


if __name__ == "__main__":
    raise SystemExit(main())
