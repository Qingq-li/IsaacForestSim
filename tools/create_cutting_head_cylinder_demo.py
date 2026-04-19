#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def add_xform_ops(prim: Usd.Prim, translate: tuple[float, float, float]) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))


def add_static_cylinder(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    axis: str,
    translate: tuple[float, float, float],
    color: tuple[float, float, float],
) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set(axis)
    add_xform_ops(cyl.GetPrim(), translate)
    UsdGeom.Gprim(cyl).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())


def add_dynamic_cylinder(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    axis: str,
    translate: tuple[float, float, float],
    color: tuple[float, float, float],
    mass: float,
) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set(axis)
    add_xform_ops(cyl.GetPrim(), translate)
    UsdGeom.Gprim(cyl).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    prim = cyl.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.ApplyAPI(UsdPhysics.RigidBodyAPI)
    prim.ApplyAPI(UsdPhysics.MassAPI)
    prim.CreateAttribute("physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(True)
    prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(False)
    prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float, custom=False).Set(mass)


def set_mode_override(stage: Usd.Stage, mode: int) -> None:
    prim = stage.OverridePrim("/Graph/ros_drive_graph/cutting_head_targets")
    prim.CreateAttribute("inputs:mode", Sdf.ValueTypeNames.Int, custom=True).Set(mode)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a cutting-head cylinder demo stage.")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", type=int, default=0, help="Initial cutting-head mode override.")
    args = parser.parse_args()

    source = args.source.resolve()
    output = args.output.resolve()

    stage = Usd.Stage.CreateNew(str(output))
    stage.SetMetadata("defaultPrim", "World")
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    root = stage.GetRootLayer()
    root.subLayerPaths.append(str(source))

    rig = UsdGeom.Xform.Define(stage, "/World/CuttingHeadCylinderDemo").GetPrim()
    add_xform_ops(rig, (0.0, 0.0, 0.0))

    add_static_cylinder(
        stage,
        "/World/CuttingHeadCylinderDemo/StandingTrunk",
        radius=0.11,
        height=2.4,
        axis="Z",
        translate=(-1.28, -0.49, 1.20),
        color=(0.52, 0.34, 0.18),
    )
    add_dynamic_cylinder(
        stage,
        "/World/CuttingHeadCylinderDemo/FeedLog",
        radius=0.08,
        height=1.4,
        axis="X",
        translate=(-1.20, -0.49, 0.60),
        color=(0.68, 0.47, 0.23),
        mass=55.0,
    )
    add_static_cylinder(
        stage,
        "/World/CuttingHeadCylinderDemo/CutTargetMarker",
        radius=0.125,
        height=0.02,
        axis="X",
        translate=(-1.42, -0.49, 0.44),
        color=(0.78, 0.14, 0.14),
    )

    set_mode_override(stage, args.mode)
    stage.GetRootLayer().Save()
    print(f"Wrote cutting-head cylinder demo: {output}")


if __name__ == "__main__":
    main()
