#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


ROOT = "/World/CuttingHeadValidation"


def set_xform(prim: Usd.Prim, translate: tuple[float, float, float]) -> None:
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    x.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))


def ensure_root(stage: Usd.Stage) -> None:
    UsdGeom.Xform.Define(stage, ROOT)


def add_cylinder(
    stage: Usd.Stage,
    path: str,
    *,
    radius: float,
    height: float,
    axis: str,
    translate: tuple[float, float, float],
    color: tuple[float, float, float],
    dynamic: bool,
    mass: float = 0.0,
) -> None:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    prim = UsdGeom.Cylinder.Define(stage, path).GetPrim()
    geom = UsdGeom.Cylinder(prim)
    geom.GetRadiusAttr().Set(radius)
    geom.GetHeightAttr().Set(height)
    geom.GetAxisAttr().Set(axis)
    set_xform(prim, translate)
    UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(prim)
    if dynamic:
        prim.ApplyAPI(UsdPhysics.RigidBodyAPI)
        prim.ApplyAPI(UsdPhysics.MassAPI)
        prim.CreateAttribute("physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(True)
        prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(False)
        prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float, custom=False).Set(float(mass))


def set_mode(stage: Usd.Stage, mode: int) -> None:
    prim = stage.OverridePrim("/Graph/ros_drive_graph/cutting_head_targets")
    prim.CreateAttribute("inputs:mode", Sdf.ValueTypeNames.Int, custom=True).Set(mode)


def main() -> None:
    parser = argparse.ArgumentParser(description="Add validation cylinders to the main harvester stage.")
    parser.add_argument("usd_path", type=Path)
    parser.add_argument("--mode", type=int, default=1)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.usd_path.resolve()))
    if not stage:
        raise RuntimeError(f"Failed to open stage: {args.usd_path}")

    ensure_root(stage)
    add_cylinder(
        stage,
        f"{ROOT}/StandingTrunk",
        radius=0.11,
        height=2.4,
        axis="Z",
        translate=(-1.28, -0.49, 1.20),
        color=(0.52, 0.34, 0.18),
        dynamic=False,
    )
    add_cylinder(
        stage,
        f"{ROOT}/FeedLog",
        radius=0.08,
        height=1.4,
        axis="X",
        translate=(-1.20, -0.49, 0.60),
        color=(0.68, 0.47, 0.23),
        dynamic=True,
        mass=55.0,
    )
    add_cylinder(
        stage,
        f"{ROOT}/CutTargetMarker",
        radius=0.125,
        height=0.02,
        axis="X",
        translate=(-1.42, -0.49, 0.44),
        color=(0.78, 0.14, 0.14),
        dynamic=False,
    )
    set_mode(stage, args.mode)
    stage.GetRootLayer().Save()
    print(f"Added cutting-head validation cylinders to: {args.usd_path}")


if __name__ == "__main__":
    main()
