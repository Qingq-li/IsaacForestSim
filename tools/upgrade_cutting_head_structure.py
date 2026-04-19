#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


ROOT = "/HarveriIsaac"
VISUALS = "/visuals"
COLLIDERS = "/colliders"


@dataclass(frozen=True)
class BodySpec:
    name: str
    parent_body: str
    local_anchor: tuple[float, float, float]
    mass: float
    diagonal_inertia: tuple[float, float, float]
    center_of_mass: tuple[float, float, float]
    orient_offset: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    body0: str
    body1: str
    local_pos0: tuple[float, float, float]
    axis: str | None = None
    local_rot0: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    lower: float | None = None
    upper: float | None = None
    stiffness: float = 0.0
    damping: float = 0.0
    max_force: float = 500.0
    max_velocity: float = 90.0
    target_position: float = 0.0
    target_velocity: float = 0.0


BODY_SPECS = [
    BodySpec(
        name="HEAD_FRAME",
        parent_body="TOOL_TILT",
        local_anchor=(0.22, -0.02, -0.22),
        mass=28.0,
        diagonal_inertia=(1.2, 1.4, 1.0),
        center_of_mass=(0.08, 0.0, -0.02),
    ),
    BodySpec(
        name="CLAMP_ARM_UL",
        parent_body="GRIPPER_LEFT",
        local_anchor=(0.16, 0.00, 0.10),
        mass=5.0,
        diagonal_inertia=(0.05, 0.07, 0.07),
        center_of_mass=(0.12, 0.0, 0.0),
    ),
    BodySpec(
        name="CLAMP_ARM_LL",
        parent_body="GRIPPER_LEFT",
        local_anchor=(0.16, 0.00, -0.10),
        mass=5.0,
        diagonal_inertia=(0.05, 0.07, 0.07),
        center_of_mass=(0.12, 0.0, 0.0),
    ),
    BodySpec(
        name="CLAMP_ARM_UR",
        parent_body="GRIPPER_RIGHT",
        local_anchor=(0.16, 0.00, 0.10),
        mass=5.0,
        diagonal_inertia=(0.05, 0.07, 0.07),
        center_of_mass=(0.12, 0.0, 0.0),
    ),
    BodySpec(
        name="CLAMP_ARM_LR",
        parent_body="GRIPPER_RIGHT",
        local_anchor=(0.16, 0.00, -0.10),
        mass=5.0,
        diagonal_inertia=(0.05, 0.07, 0.07),
        center_of_mass=(0.12, 0.0, 0.0),
    ),
    BodySpec(
        name="FEED_ARM_LEFT",
        parent_body="HEAD_FRAME",
        local_anchor=(0.18, 0.19, 0.03),
        mass=8.0,
        diagonal_inertia=(0.08, 0.08, 0.05),
        center_of_mass=(0.10, 0.0, 0.0),
    ),
    BodySpec(
        name="FEED_ARM_RIGHT",
        parent_body="HEAD_FRAME",
        local_anchor=(0.18, -0.19, 0.03),
        mass=8.0,
        diagonal_inertia=(0.08, 0.08, 0.05),
        center_of_mass=(0.10, 0.0, 0.0),
    ),
    BodySpec(
        name="FEED_ROLLER_LEFT",
        parent_body="FEED_ARM_LEFT",
        local_anchor=(0.12, -0.03, 0.0),
        mass=4.0,
        diagonal_inertia=(0.02, 0.02, 0.01),
        center_of_mass=(0.0, 0.0, 0.0),
    ),
    BodySpec(
        name="FEED_ROLLER_RIGHT",
        parent_body="FEED_ARM_RIGHT",
        local_anchor=(0.12, 0.03, 0.0),
        mass=4.0,
        diagonal_inertia=(0.02, 0.02, 0.01),
        center_of_mass=(0.0, 0.0, 0.0),
    ),
    BodySpec(
        name="SAW_SWING",
        parent_body="HEAD_FRAME",
        local_anchor=(0.16, -0.20, -0.12),
        mass=10.0,
        diagonal_inertia=(0.09, 0.09, 0.06),
        center_of_mass=(0.08, 0.0, 0.0),
    ),
    BodySpec(
        name="SAW_DISC",
        parent_body="SAW_SWING",
        local_anchor=(0.14, -0.07, -0.02),
        mass=6.0,
        diagonal_inertia=(0.03, 0.03, 0.02),
        center_of_mass=(0.0, 0.0, 0.0),
    ),
    BodySpec(
        name="MEASURING_WHEEL",
        parent_body="HEAD_FRAME",
        local_anchor=(0.20, 0.0, 0.16),
        mass=2.5,
        diagonal_inertia=(0.01, 0.01, 0.008),
        center_of_mass=(0.0, 0.0, 0.0),
    ),
]


JOINT_SPECS = [
    JointSpec(
        name="J_HEAD_FRAME_MOUNT",
        joint_type="fixed",
        body0="TOOL_TILT",
        body1="HEAD_FRAME",
        local_pos0=(0.22, -0.02, -0.22),
    ),
    JointSpec(
        name="J_CLAMP_ARM_UL",
        joint_type="revolute",
        body0="GRIPPER_LEFT",
        body1="CLAMP_ARM_UL",
        local_pos0=(0.16, 0.00, 0.10),
        axis="Z",
        lower=-10.0,
        upper=35.0,
        stiffness=3200.0,
        damping=45.0,
        max_force=1200.0,
        max_velocity=90.0,
    ),
    JointSpec(
        name="J_CLAMP_ARM_LL",
        joint_type="revolute",
        body0="GRIPPER_LEFT",
        body1="CLAMP_ARM_LL",
        local_pos0=(0.16, 0.00, -0.10),
        axis="Z",
        lower=-35.0,
        upper=10.0,
        stiffness=3200.0,
        damping=45.0,
        max_force=1200.0,
        max_velocity=90.0,
    ),
    JointSpec(
        name="J_CLAMP_ARM_UR",
        joint_type="revolute",
        body0="GRIPPER_RIGHT",
        body1="CLAMP_ARM_UR",
        local_pos0=(0.16, 0.00, 0.10),
        axis="Z",
        lower=-35.0,
        upper=10.0,
        stiffness=3200.0,
        damping=45.0,
        max_force=1200.0,
        max_velocity=90.0,
    ),
    JointSpec(
        name="J_CLAMP_ARM_LR",
        joint_type="revolute",
        body0="GRIPPER_RIGHT",
        body1="CLAMP_ARM_LR",
        local_pos0=(0.16, 0.00, -0.10),
        axis="Z",
        lower=-10.0,
        upper=35.0,
        stiffness=3200.0,
        damping=45.0,
        max_force=1200.0,
        max_velocity=90.0,
    ),
    JointSpec(
        name="J_FEED_ARM_LEFT",
        joint_type="revolute",
        body0="HEAD_FRAME",
        body1="FEED_ARM_LEFT",
        axis="Z",
        local_pos0=(0.18, 0.19, 0.03),
        lower=-30.0,
        upper=15.0,
        stiffness=2600.0,
        damping=40.0,
        max_force=1600.0,
        max_velocity=75.0,
        target_position=-0.16,
    ),
    JointSpec(
        name="J_FEED_ARM_RIGHT",
        joint_type="revolute",
        body0="HEAD_FRAME",
        body1="FEED_ARM_RIGHT",
        axis="Z",
        local_pos0=(0.18, -0.19, 0.03),
        lower=-15.0,
        upper=30.0,
        stiffness=2600.0,
        damping=40.0,
        max_force=1600.0,
        max_velocity=75.0,
        target_position=0.16,
    ),
    JointSpec(
        name="J_FEED_ROLLER_LEFT_SPIN",
        joint_type="revolute",
        body0="FEED_ARM_LEFT",
        body1="FEED_ROLLER_LEFT",
        axis="Z",
        local_pos0=(0.12, -0.03, 0.0),
        stiffness=0.0,
        damping=12.0,
        max_force=1500.0,
        max_velocity=240.0,
    ),
    JointSpec(
        name="J_FEED_ROLLER_RIGHT_SPIN",
        joint_type="revolute",
        body0="FEED_ARM_RIGHT",
        body1="FEED_ROLLER_RIGHT",
        axis="Z",
        local_pos0=(0.12, 0.03, 0.0),
        stiffness=0.0,
        damping=12.0,
        max_force=1500.0,
        max_velocity=240.0,
    ),
    JointSpec(
        name="J_FELLING_SAW_SWING",
        joint_type="revolute",
        body0="HEAD_FRAME",
        body1="SAW_SWING",
        axis="Z",
        local_pos0=(0.16, -0.20, -0.12),
        lower=-8.0,
        upper=78.0,
        stiffness=1800.0,
        damping=36.0,
        max_force=2600.0,
        max_velocity=85.0,
    ),
    JointSpec(
        name="J_SAW_DISC_SPIN",
        joint_type="revolute",
        body0="SAW_SWING",
        body1="SAW_DISC",
        axis="Y",
        local_pos0=(0.14, -0.07, -0.02),
        stiffness=0.0,
        damping=8.0,
        max_force=3000.0,
        max_velocity=600.0,
    ),
    JointSpec(
        name="J_MEASURING_WHEEL",
        joint_type="revolute",
        body0="HEAD_FRAME",
        body1="MEASURING_WHEEL",
        axis="Y",
        local_pos0=(0.20, 0.0, 0.16),
        stiffness=0.0,
        damping=4.0,
        max_force=400.0,
        max_velocity=360.0,
    ),
]


def quatd(value) -> Gf.Quatd:
    if isinstance(value, Gf.Quatd):
        return value
    if isinstance(value, Gf.Quatf):
        imag = value.GetImaginary()
        return Gf.Quatd(value.GetReal(), imag[0], imag[1], imag[2])
    return Gf.Quatd(value[0], value[1], value[2], value[3])


def quatf(value) -> Gf.Quatf:
    if isinstance(value, Gf.Quatf):
        return value
    if isinstance(value, Gf.Quatd):
        imag = value.GetImaginary()
        return Gf.Quatf(value.GetReal(), imag[0], imag[1], imag[2])
    return Gf.Quatf(value[0], value[1], value[2], value[3])


def clear_xform_ops(prim: Usd.Prim) -> None:
    UsdGeom.Xformable(prim).ClearXformOpOrder()


def set_body_xform(prim: Usd.Prim, translation: Gf.Vec3d, orient: Gf.Quatd) -> None:
    x = UsdGeom.Xformable(prim)
    clear_xform_ops(prim)
    x.AddTranslateOp().Set(translation)
    x.AddOrientOp().Set(quatf(orient))
    x.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))


def compose(parent_translate: Gf.Vec3d, parent_orient: Gf.Quatd, local_pos, local_orient) -> tuple[Gf.Vec3d, Gf.Quatd]:
    rot = Gf.Rotation(parent_orient)
    world_t = parent_translate + rot.TransformDir(Gf.Vec3d(*local_pos))
    world_q = parent_orient * quatd(local_orient)
    return world_t, world_q


def get_pose(stage: Usd.Stage, body_name: str) -> tuple[Gf.Vec3d, Gf.Quatd]:
    prim = stage.GetPrimAtPath(f"{ROOT}/{body_name}")
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim for pose lookup: {ROOT}/{body_name}")
    x = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op.Get() for op in x.GetOrderedXformOps()}
    if "xformOp:translate" not in ops or "xformOp:orient" not in ops:
        raise RuntimeError(f"Missing xform ops for pose lookup: {ROOT}/{body_name}")
    return Gf.Vec3d(*ops["xformOp:translate"]), quatd(ops["xformOp:orient"])


def resolve_parent_pose(stage: Usd.Stage, fallback_stage: Usd.Stage, body_name: str) -> tuple[Gf.Vec3d, Gf.Quatd]:
    prim = stage.GetPrimAtPath(f"{ROOT}/{body_name}")
    if prim and prim.IsValid():
        try:
            return get_pose(stage, body_name)
        except RuntimeError:
            pass
    return get_pose(fallback_stage, body_name)


def ensure_body(
    stage: Usd.Stage,
    name: str,
    translate: Gf.Vec3d,
    orient: Gf.Quatd,
    with_visuals: bool,
    with_collisions: bool,
    base_asset_path: str | None = None,
) -> Usd.Prim:
    prim = UsdGeom.Xform.Define(stage, f"{ROOT}/{name}").GetPrim()
    set_body_xform(prim, translate, orient)
    if with_visuals:
        vis = UsdGeom.Xform.Define(stage, f"{ROOT}/{name}/visuals").GetPrim()
        vis.SetInstanceable(True)
        vis.GetReferences().ClearReferences()
        if base_asset_path:
            vis.GetReferences().AddReference(base_asset_path, f"{VISUALS}/{name}")
        else:
            vis.GetReferences().AddInternalReference(f"{VISUALS}/{name}")
    if with_collisions:
        col = UsdGeom.Xform.Define(stage, f"{ROOT}/{name}/collisions").GetPrim()
        col.SetInstanceable(True)
        col.GetReferences().ClearReferences()
        if base_asset_path:
            col.GetReferences().AddReference(base_asset_path, f"{COLLIDERS}/{name}")
        else:
            col.GetReferences().AddInternalReference(f"{COLLIDERS}/{name}")
    return prim


def add_cube(stage: Usd.Stage, path: str, translate, scale, color, collision=False) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(cube)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    x.AddScaleOp().Set(Gf.Vec3f(*scale))
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    else:
        UsdGeom.Gprim(cube).CreateDisplayColorAttr([Gf.Vec3f(*color)])


def add_cylinder(stage: Usd.Stage, path: str, translate, radius, height, axis, color, collision=False) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set(axis)
    x = UsdGeom.Xformable(cyl)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    x.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    if collision:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    else:
        UsdGeom.Gprim(cyl).CreateDisplayColorAttr([Gf.Vec3f(*color)])


def author_prototypes(base_stage: Usd.Stage) -> None:
    shell = (0.71, 0.72, 0.70)
    dark = (0.16, 0.16, 0.16)
    steel = (0.58, 0.60, 0.60)
    accent = (0.80, 0.54, 0.18)

    for name in [spec.name for spec in BODY_SPECS]:
        for root in [VISUALS, COLLIDERS]:
            path = f"{root}/{name}"
            if base_stage.GetPrimAtPath(path):
                base_stage.RemovePrim(path)
            UsdGeom.Xform.Define(base_stage, path)

    for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
        add_cube(base_stage, f"{root}/HEAD_FRAME/core", (0.08, 0.0, 0.0), (0.20, 0.19, 0.16), shell, collision)
        add_cube(base_stage, f"{root}/HEAD_FRAME/upper_hood", (0.02, 0.0, 0.14), (0.16, 0.18, 0.05), dark, collision)
        add_cube(base_stage, f"{root}/HEAD_FRAME/back_spine", (-0.08, 0.0, 0.03), (0.08, 0.12, 0.14), shell, collision)
        add_cube(base_stage, f"{root}/HEAD_FRAME/channel_left", (0.12, 0.13, 0.00), (0.12, 0.03, 0.13), accent, collision)
        add_cube(base_stage, f"{root}/HEAD_FRAME/channel_right", (0.12, -0.13, 0.00), (0.12, 0.03, 0.13), accent, collision)
        add_cube(base_stage, f"{root}/HEAD_FRAME/channel_back", (-0.01, 0.0, 0.00), (0.05, 0.11, 0.12), dark, collision)
        add_cube(base_stage, f"{root}/HEAD_FRAME/channel_floor", (0.07, 0.0, -0.11), (0.15, 0.10, 0.02), dark, collision)

    arm_shapes = {
        "CLAMP_ARM_UL": [(0.12, 0.0, 0.0), (0.16, 0.0, -0.02)],
        "CLAMP_ARM_LL": [(0.12, 0.0, 0.0), (0.16, 0.0, 0.02)],
        "CLAMP_ARM_UR": [(0.12, 0.0, 0.0), (0.16, 0.0, -0.02)],
        "CLAMP_ARM_LR": [(0.12, 0.0, 0.0), (0.16, 0.0, 0.02)],
    }
    for name, pads in arm_shapes.items():
        for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
            add_cube(base_stage, f"{root}/{name}/beam", pads[0], (0.18, 0.03, 0.03), shell, collision)
            add_cube(base_stage, f"{root}/{name}/pad", pads[1], (0.10, 0.025, 0.07), dark, collision)
            add_cube(base_stage, f"{root}/{name}/brace", (0.04, 0.0, 0.0), (0.06, 0.025, 0.08), shell, collision)

    for name, side in [("FEED_ARM_LEFT", 1.0), ("FEED_ARM_RIGHT", -1.0)]:
        for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
            add_cube(base_stage, f"{root}/{name}/arm_beam", (0.09, 0.0, 0.0), (0.12, 0.035, 0.035), shell, collision)
            add_cube(base_stage, f"{root}/{name}/knuckle", (0.02, 0.0, 0.0), (0.05, 0.05, 0.05), dark, collision)
            add_cube(base_stage, f"{root}/{name}/guard", (0.14, 0.03 * side, 0.0), (0.06, 0.02, 0.05), accent, collision)

    for name in ["FEED_ROLLER_LEFT", "FEED_ROLLER_RIGHT"]:
        for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
            add_cylinder(base_stage, f"{root}/{name}/roller", (0.0, 0.0, 0.0), 0.075, 0.11, "Z", dark, collision)
            add_cube(base_stage, f"{root}/{name}/hub", (0.0, 0.0, 0.0), (0.05, 0.05, 0.05), shell, collision)

    for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
        add_cube(base_stage, f"{root}/SAW_SWING/housing", (0.06, 0.0, 0.0), (0.12, 0.05, 0.08), shell, collision)
        add_cube(base_stage, f"{root}/SAW_SWING/arm", (-0.04, 0.0, 0.02), (0.08, 0.035, 0.05), dark, collision)
        add_cube(base_stage, f"{root}/SAW_SWING/guard", (0.15, -0.02, -0.02), (0.06, 0.03, 0.06), accent, collision)

    for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
        add_cylinder(base_stage, f"{root}/SAW_DISC/disc", (0.0, 0.0, 0.0), 0.17, 0.022, "Y", steel, collision)
        add_cube(base_stage, f"{root}/SAW_DISC/hub", (0.0, 0.0, 0.0), (0.04, 0.05, 0.05), dark, collision)

    for root, collision in [(VISUALS, False), (COLLIDERS, True)]:
        add_cylinder(base_stage, f"{root}/MEASURING_WHEEL/wheel", (0.0, 0.0, 0.0), 0.055, 0.04, "Y", accent, collision)
        add_cube(base_stage, f"{root}/MEASURING_WHEEL/fork", (-0.03, 0.0, 0.0), (0.03, 0.025, 0.06), shell, collision)


def author_robot_bodies(robot_stage: Usd.Stage, base_stage: Usd.Stage, base_asset_path: str) -> None:
    for spec in BODY_SPECS:
        parent_t, parent_q = resolve_parent_pose(robot_stage, base_stage, spec.parent_body)
        world_t, world_q = compose(parent_t, parent_q, spec.local_anchor, spec.orient_offset)
        ensure_body(
            robot_stage,
            spec.name,
            world_t,
            world_q,
            with_visuals=True,
            with_collisions=False,
            base_asset_path=base_asset_path,
        )


def author_physics_bodies_and_joints(physics_stage: Usd.Stage, base_stage: Usd.Stage, base_asset_path: str) -> None:
    for spec in BODY_SPECS:
        parent_t, parent_q = resolve_parent_pose(physics_stage, base_stage, spec.parent_body)
        world_t, world_q = compose(parent_t, parent_q, spec.local_anchor, spec.orient_offset)
        prim = ensure_body(
            physics_stage,
            spec.name,
            world_t,
            world_q,
            with_visuals=True,
            with_collisions=True,
            base_asset_path=base_asset_path,
        )
        prim.ApplyAPI(UsdPhysics.RigidBodyAPI)
        prim.ApplyAPI(UsdPhysics.MassAPI)
        prim.CreateAttribute("physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(True)
        prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(False)
        prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float, custom=False).Set(spec.mass)
        prim.CreateAttribute("physics:centerOfMass", Sdf.ValueTypeNames.Point3f, custom=False).Set(Gf.Vec3f(*spec.center_of_mass))
        prim.CreateAttribute("physics:diagonalInertia", Sdf.ValueTypeNames.Float3, custom=False).Set(Gf.Vec3f(*spec.diagonal_inertia))
        prim.CreateAttribute("physics:principalAxes", Sdf.ValueTypeNames.Quatf, custom=False).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    joints_scope = UsdGeom.Scope.Define(physics_stage, f"{ROOT}/joints").GetPrim()
    for spec in JOINT_SPECS:
        path = f"{ROOT}/joints/{spec.name}"
        if physics_stage.GetPrimAtPath(path):
            physics_stage.RemovePrim(path)
        if spec.joint_type == "fixed":
            joint = UsdPhysics.FixedJoint.Define(physics_stage, path).GetPrim()
        elif spec.joint_type == "revolute":
            joint = UsdPhysics.RevoluteJoint.Define(physics_stage, path).GetPrim()
            joint.ApplyAPI(UsdPhysics.DriveAPI, "angular")
        else:
            raise ValueError(f"Unsupported joint type: {spec.joint_type}")
        joint.CreateRelationship("physics:body0").SetTargets([Sdf.Path(f"{ROOT}/{spec.body0}")])
        joint.CreateRelationship("physics:body1").SetTargets([Sdf.Path(f"{ROOT}/{spec.body1}")])
        joint.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f, custom=False).Set(Gf.Vec3f(*spec.local_pos0))
        joint.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f, custom=False).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf, custom=False).Set(quatf(spec.local_rot0))
        joint.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf, custom=False).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        if spec.joint_type == "revolute":
            joint.CreateAttribute("physics:axis", Sdf.ValueTypeNames.Token, custom=False).Set(spec.axis)
            joint.CreateAttribute("drive:angular:physics:type", Sdf.ValueTypeNames.Token, custom=False).Set("force")
            joint.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float, custom=False).Set(spec.stiffness)
            joint.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float, custom=False).Set(spec.damping)
            joint.CreateAttribute("drive:angular:physics:maxForce", Sdf.ValueTypeNames.Float, custom=False).Set(spec.max_force)
            joint.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float, custom=False).Set(spec.target_position)
            joint.CreateAttribute("drive:angular:physics:targetVelocity", Sdf.ValueTypeNames.Float, custom=False).Set(spec.target_velocity)
            joint.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, custom=False).Set(spec.max_velocity)
            joint.CreateAttribute("physxJoint:jointFriction", Sdf.ValueTypeNames.Float, custom=False).Set(2.0)
            if spec.lower is not None:
                joint.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float, custom=False).Set(spec.lower)
            if spec.upper is not None:
                joint.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float, custom=False).Set(spec.upper)


def main() -> None:
    parser = argparse.ArgumentParser(description="Upgrade the harvester cutting head with explicit arm/roller/saw bodies.")
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--robot", required=True, type=Path)
    parser.add_argument("--physics", required=True, type=Path)
    args = parser.parse_args()

    base_stage = Usd.Stage.Open(str(args.base))
    robot_stage = Usd.Stage.Open(str(args.robot))
    physics_stage = Usd.Stage.Open(str(args.physics))
    if not base_stage or not robot_stage or not physics_stage:
        raise RuntimeError("Failed to open one or more stages.")

    author_prototypes(base_stage)
    base_stage.GetRootLayer().Save()

    robot_stage = Usd.Stage.Open(str(args.robot))
    physics_stage = Usd.Stage.Open(str(args.physics))
    base_asset_path = str(args.base.resolve())
    author_robot_bodies(robot_stage, base_stage, base_asset_path)
    author_physics_bodies_and_joints(physics_stage, base_stage, base_asset_path)

    robot_stage.GetRootLayer().Save()
    physics_stage.GetRootLayer().Save()
    print("Upgraded cutting head structure in base/robot/physics layers.")


if __name__ == "__main__":
    main()
