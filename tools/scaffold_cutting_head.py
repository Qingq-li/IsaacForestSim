#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def ensure_xform(stage: Usd.Stage, path: str) -> UsdGeom.Xform:
    return UsdGeom.Xform.Define(stage, path)


def reset_xform_ops(xformable: UsdGeom.Xformable) -> None:
    xformable.ClearXformOpOrder()


def set_trs(prim, translate=(0, 0, 0), orient=None, scale=(1, 1, 1)) -> None:
    x = UsdGeom.Xformable(prim)
    reset_xform_ops(x)
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if orient is None:
        orient = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    x.AddOrientOp().Set(orient)
    x.AddScaleOp().Set(Gf.Vec3f(*scale))


def add_cube(
    stage: Usd.Stage,
    path: str,
    translate,
    scale,
    color,
    collision: bool = False,
) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    set_trs(cube.GetPrim(), translate=translate, scale=scale)
    if not collision:
        UsdGeom.Gprim(cube).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    else:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def add_cylinder(
    stage: Usd.Stage,
    path: str,
    translate,
    scale,
    radius: float,
    height: float,
    axis: str,
    color,
    collision: bool = False,
) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set(axis)
    set_trs(cyl.GetPrim(), translate=translate, scale=scale)
    if not collision:
        UsdGeom.Gprim(cyl).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    else:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())


def remove_if_exists(stage: Usd.Stage, path: str) -> None:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)


def build_head_body(stage: Usd.Stage, base_path: str, collision: bool) -> None:
    colors = {
        "shell": (0.66, 0.68, 0.66),
        "dark": (0.18, 0.18, 0.18),
    }
    add_cube(
        stage,
        f"{base_path}/upper_shell",
        translate=(0.16, 0.0, -0.10),
        scale=(0.22, 0.26, 0.12),
        color=colors["shell"],
        collision=collision,
    )
    add_cube(
        stage,
        f"{base_path}/lower_shell",
        translate=(0.25, 0.0, -0.28),
        scale=(0.28, 0.18, 0.18),
        color=colors["shell"],
        collision=collision,
    )
    add_cube(
        stage,
        f"{base_path}/stem_neck",
        translate=(0.42, 0.0, -0.16),
        scale=(0.12, 0.08, 0.30),
        color=colors["shell"],
        collision=collision,
    )
    add_cube(
        stage,
        f"{base_path}/rear_spine",
        translate=(0.02, 0.0, 0.02),
        scale=(0.10, 0.12, 0.20),
        color=colors["dark"],
        collision=collision,
    )


def build_side_assembly(stage: Usd.Stage, side_root: str, side: str, collision: bool) -> None:
    sign = 1.0 if side == "left" else -1.0
    shell = (0.69, 0.71, 0.69)
    dark = (0.16, 0.16, 0.16)

    add_cube(
        stage,
        f"{side_root}/main_arm",
        translate=(0.16, 0.0, 0.0),
        scale=(0.18, 0.05, 0.05),
        color=shell,
        collision=collision,
    )
    add_cube(
        stage,
        f"{side_root}/upper_arm",
        translate=(0.18, 0.0, 0.12),
        scale=(0.22, 0.04, 0.04),
        color=shell,
        collision=collision,
    )
    add_cube(
        stage,
        f"{side_root}/lower_arm",
        translate=(0.18, 0.0, -0.12),
        scale=(0.22, 0.04, 0.04),
        color=shell,
        collision=collision,
    )
    add_cube(
        stage,
        f"{side_root}/upper_pad",
        translate=(0.33, 0.0, 0.10),
        scale=(0.08, 0.03, 0.08),
        color=dark,
        collision=collision,
    )
    add_cube(
        stage,
        f"{side_root}/lower_pad",
        translate=(0.33, 0.0, -0.10),
        scale=(0.08, 0.03, 0.08),
        color=dark,
        collision=collision,
    )
    add_cylinder(
        stage,
        f"{side_root}/feed_roller",
        translate=(0.12, 0.0, 0.0),
        scale=(1.0, 1.0, 1.0),
        radius=0.075,
        height=0.12,
        axis="Z",
        color=dark,
        collision=collision,
    )
    add_cube(
        stage,
        f"{side_root}/roller_support",
        translate=(0.06, 0.0, 0.0),
        scale=(0.08, 0.03, 0.14),
        color=shell,
        collision=collision,
    )
    add_cube(
        stage,
        f"{side_root}/outer_rib",
        translate=(0.20, sign * 0.02, 0.0),
        scale=(0.12, 0.02, 0.18),
        color=shell,
        collision=collision,
    )


def build_saw_module(stage: Usd.Stage, base_path: str, collision: bool) -> None:
    shell = (0.70, 0.70, 0.68)
    metal = (0.58, 0.60, 0.60)
    dark = (0.14, 0.14, 0.14)

    add_cube(
        stage,
        f"{base_path}/saw_housing",
        translate=(0.02, -0.05, -0.01),
        scale=(0.16, 0.08, 0.10),
        color=shell,
        collision=collision,
    )
    add_cube(
        stage,
        f"{base_path}/saw_guard",
        translate=(0.10, -0.10, 0.02),
        scale=(0.12, 0.05, 0.05),
        color=dark,
        collision=collision,
    )
    add_cylinder(
        stage,
        f"{base_path}/saw_disc",
        translate=(0.16, -0.14, -0.03),
        scale=(1.0, 1.0, 1.0),
        radius=0.18,
        height=0.025,
        axis="Y",
        color=metal,
        collision=collision,
    )


def build_visuals_and_collisions(stage: Usd.Stage) -> None:
    targets = [
        "/visuals/TOOL_TILT/cutting_head_v1",
        "/colliders/TOOL_TILT/cutting_head_v1",
        "/visuals/CUTTER_ARM/cutting_head_v1",
        "/colliders/CUTTER_ARM/cutting_head_v1",
        "/visuals/GRIPPER_LEFT/cutting_head_v1",
        "/colliders/GRIPPER_LEFT/cutting_head_v1",
        "/visuals/GRIPPER_RIGHT/cutting_head_v1",
        "/colliders/GRIPPER_RIGHT/cutting_head_v1",
    ]
    for path in targets:
        remove_if_exists(stage, path)

    tool_vis = ensure_xform(stage, "/visuals/TOOL_TILT/cutting_head_v1")
    tool_col = ensure_xform(stage, "/colliders/TOOL_TILT/cutting_head_v1")
    cut_vis = ensure_xform(stage, "/visuals/CUTTER_ARM/cutting_head_v1")
    cut_col = ensure_xform(stage, "/colliders/CUTTER_ARM/cutting_head_v1")
    left_vis = ensure_xform(stage, "/visuals/GRIPPER_LEFT/cutting_head_v1")
    left_col = ensure_xform(stage, "/colliders/GRIPPER_LEFT/cutting_head_v1")
    right_vis = ensure_xform(stage, "/visuals/GRIPPER_RIGHT/cutting_head_v1")
    right_col = ensure_xform(stage, "/colliders/GRIPPER_RIGHT/cutting_head_v1")

    build_head_body(stage, str(tool_vis.GetPath()), collision=False)
    build_head_body(stage, str(tool_col.GetPath()), collision=True)
    build_saw_module(stage, str(cut_vis.GetPath()), collision=False)
    build_saw_module(stage, str(cut_col.GetPath()), collision=True)
    build_side_assembly(stage, str(left_vis.GetPath()), side="left", collision=False)
    build_side_assembly(stage, str(left_col.GetPath()), side="left", collision=True)
    build_side_assembly(stage, str(right_vis.GetPath()), side="right", collision=False)
    build_side_assembly(stage, str(right_col.GetPath()), side="right", collision=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Author a first-pass realistic cutting-head scaffold.")
    parser.add_argument("usd_path", type=Path)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.usd_path))
    if not stage:
        raise RuntimeError(f"Failed to open stage: {args.usd_path}")

    build_visuals_and_collisions(stage)
    stage.GetRootLayer().Save()
    print(f"Updated cutting-head scaffold in {args.usd_path}")


if __name__ == "__main__":
    main()
