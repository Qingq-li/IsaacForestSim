#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from pxr import Sdf, Usd


AXLE_SEPARATION = 1.9476267844438553
TRACK_WIDTH = 1.5339090526103973
WHEEL_RADIUS = 0.4

# Preserve the existing steering sign convention from the authored graph.
STEER_SIGN = -1.0

MIN_LINEAR_FOR_STEER = 0.3
MAX_STEER_ANGLE = 1.0
MAX_STEER_RATE = 0.35
MAX_LINEAR_SPEED = 2.5
MAX_ANGULAR_SPEED = 0.6
MAX_LINEAR_ACCEL = 0.8
MAX_LINEAR_DECEL = 1.2
MAX_ANGULAR_ACCEL = 0.5
MAX_WHEEL_SPEED = 8.0

FEED_ARM_LEFT_DEFAULT = -0.16
FEED_ARM_RIGHT_DEFAULT = 0.16
FEED_ARM_LEFT_MIN = -0.52
FEED_ARM_LEFT_MAX = 0.26
FEED_ARM_RIGHT_MIN = -0.26
FEED_ARM_RIGHT_MAX = 0.52
FEED_ARM_RATE_LIMIT = 0.45

SAW_SWING_DEFAULT = 0.0
SAW_SWING_MIN = -0.14
SAW_SWING_MAX = 1.36
SAW_SWING_RATE_LIMIT = 0.75

MEASURING_WHEEL_SPEED_DEFAULT = 0.0
MAX_MEASURING_WHEEL_SPEED = 12.0
MEASURING_WHEEL_ACCEL_LIMIT = 20.0

MAX_FEED_ROLLER_SPEED = 14.0
FEED_ROLLER_ACCEL_LIMIT = 18.0
MAX_SAW_DISC_SPEED = 120.0
SAW_DISC_ACCEL_LIMIT = 220.0

CUTTING_HEAD_MODE = 0

IDLE_FEED_ARM_LEFT = -0.16
IDLE_FEED_ARM_RIGHT = 0.16
IDLE_SAW_SWING = 0.0
IDLE_FEED_ROLLER_SPEED = 0.0
IDLE_MEASURING_WHEEL_SPEED = 0.0
IDLE_SAW_DISC_SPEED = 0.0

GRASP_FEED_ARM_LEFT = -0.28
GRASP_FEED_ARM_RIGHT = 0.28
GRASP_SAW_SWING = 0.0
GRASP_FEED_ROLLER_SPEED = 0.0
GRASP_MEASURING_WHEEL_SPEED = 0.0
GRASP_SAW_DISC_SPEED = 0.0

FEED_FEED_ARM_LEFT = -0.24
FEED_FEED_ARM_RIGHT = 0.24
FEED_SAW_SWING = 0.0
FEED_ROLLER_LEFT_SPEED = 5.5
FEED_ROLLER_RIGHT_SPEED = -5.5
FEED_MEASURING_WHEEL_SPEED = 4.0
FEED_SAW_DISC_SPEED = 0.0

CUT_FEED_ARM_LEFT = -0.26
CUT_FEED_ARM_RIGHT = 0.26
CUT_SAW_SWING = 1.18
CUT_FEED_ROLLER_SPEED = 0.0
CUT_MEASURING_WHEEL_SPEED = 0.0
CUT_SAW_DISC_SPEED = 95.0

CLAMP_ARM_RATE_LIMIT = 0.55
CLAMP_ARM_UL_MIN = -0.17
CLAMP_ARM_UL_MAX = 0.61
CLAMP_ARM_LL_MIN = -0.61
CLAMP_ARM_LL_MAX = 0.17
CLAMP_ARM_UR_MIN = -0.61
CLAMP_ARM_UR_MAX = 0.17
CLAMP_ARM_LR_MIN = -0.17
CLAMP_ARM_LR_MAX = 0.61

IDLE_CLAMP_ARM_UL = 0.0
IDLE_CLAMP_ARM_LL = 0.0
IDLE_CLAMP_ARM_UR = 0.0
IDLE_CLAMP_ARM_LR = 0.0

GRASP_CLAMP_ARM_UL = 0.38
GRASP_CLAMP_ARM_LL = -0.38
GRASP_CLAMP_ARM_UR = -0.38
GRASP_CLAMP_ARM_LR = 0.38

FEED_CLAMP_ARM_UL = 0.30
FEED_CLAMP_ARM_LL = -0.30
FEED_CLAMP_ARM_UR = -0.30
FEED_CLAMP_ARM_LR = 0.30

CUT_CLAMP_ARM_UL = 0.42
CUT_CLAMP_ARM_LL = -0.42
CUT_CLAMP_ARM_UR = -0.42
CUT_CLAMP_ARM_LR = 0.42


def set_constant(stage: Usd.Stage, prim_path: str, value: float) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        raise RuntimeError(f"Missing prim: {prim_path}")
    attr = prim.GetAttribute("inputs:value")
    if not attr:
        raise RuntimeError(f"Missing attribute {prim_path}.inputs:value")
    attr.Set(value)


def connect_input(stage: Usd.Stage, dst_attr_path: str, src_attr_path: str) -> None:
    attr = stage.GetAttributeAtPath(dst_attr_path)
    if not attr:
        raise RuntimeError(f"Missing attribute: {dst_attr_path}")
    attr.SetConnections([Sdf.Path(src_attr_path)])


def create_attr(prim: Usd.Prim, name: str, type_name, custom: bool = True, value=None, custom_data=None):
    attr = prim.CreateAttribute(name, type_name, custom=custom)
    if value is not None:
        attr.Set(value)
    if custom_data:
        attr.SetMetadata("customData", custom_data)
    return attr


def ensure_script_node(stage: Usd.Stage, usd_path: Path) -> str:
    node_path = "/Graph/ros_drive_graph/cmd_vel_filter"
    prim = stage.GetPrimAtPath(node_path)
    if not prim:
        prim = stage.DefinePrim(node_path, "OmniGraphNode")

    create_attr(prim, "node:type", Sdf.ValueTypeNames.Token, custom=False, value="omni.graph.scriptnode.ScriptNode")
    create_attr(prim, "node:typeVersion", Sdf.ValueTypeNames.Int, custom=False, value=2)
    create_attr(
        prim,
        "inputs:execIn",
        Sdf.ValueTypeNames.UInt,
        custom=True,
        custom_data={"isExecution": True},
    )
    create_attr(
        prim,
        "outputs:execOut",
        Sdf.ValueTypeNames.UInt,
        custom=True,
        custom_data={"isExecution": True},
    )
    create_attr(prim, "inputs:script", Sdf.ValueTypeNames.String, custom=True, value="")
    create_attr(
        prim,
        "inputs:scriptPath",
        Sdf.ValueTypeNames.Token,
        custom=True,
        value="ros_drive_controller.py",
    )
    create_attr(prim, "inputs:usePath", Sdf.ValueTypeNames.Bool, custom=True, value=True)
    create_attr(prim, "state:omni_initialized", Sdf.ValueTypeNames.Bool, custom=True, value=False)

    create_attr(prim, "inputs:linear_x", Sdf.ValueTypeNames.Double, value=0.0)
    create_attr(prim, "inputs:angular_z", Sdf.ValueTypeNames.Double, value=0.0)
    create_attr(prim, "inputs:dt", Sdf.ValueTypeNames.Double, value=0.0)
    create_attr(prim, "inputs:wheel_base", Sdf.ValueTypeNames.Double, value=AXLE_SEPARATION)
    create_attr(prim, "inputs:track_width", Sdf.ValueTypeNames.Double, value=TRACK_WIDTH)
    create_attr(prim, "inputs:wheel_radius", Sdf.ValueTypeNames.Double, value=WHEEL_RADIUS)
    create_attr(prim, "inputs:steer_sign", Sdf.ValueTypeNames.Double, value=STEER_SIGN)
    create_attr(prim, "inputs:min_linear_for_steer", Sdf.ValueTypeNames.Double, value=MIN_LINEAR_FOR_STEER)
    create_attr(prim, "inputs:max_steer_angle", Sdf.ValueTypeNames.Double, value=MAX_STEER_ANGLE)
    create_attr(prim, "inputs:max_steer_rate", Sdf.ValueTypeNames.Double, value=MAX_STEER_RATE)
    create_attr(prim, "inputs:max_linear_speed", Sdf.ValueTypeNames.Double, value=MAX_LINEAR_SPEED)
    create_attr(prim, "inputs:max_angular_speed", Sdf.ValueTypeNames.Double, value=MAX_ANGULAR_SPEED)
    create_attr(prim, "inputs:max_linear_accel", Sdf.ValueTypeNames.Double, value=MAX_LINEAR_ACCEL)
    create_attr(prim, "inputs:max_linear_decel", Sdf.ValueTypeNames.Double, value=MAX_LINEAR_DECEL)
    create_attr(prim, "inputs:max_angular_accel", Sdf.ValueTypeNames.Double, value=MAX_ANGULAR_ACCEL)
    create_attr(prim, "inputs:max_wheel_speed", Sdf.ValueTypeNames.Double, value=MAX_WHEEL_SPEED)
    create_attr(prim, "outputs:steer_command", Sdf.ValueTypeNames.DoubleArray, value=[0.0])
    create_attr(prim, "outputs:wheel_command", Sdf.ValueTypeNames.DoubleArray, value=[0.0, 0.0, 0.0, 0.0])

    prim.CreateAttribute("ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2, custom=False).Set((500.0, -150.0))
    return node_path


def ensure_cutting_head_script_node(stage: Usd.Stage) -> str:
    node_path = "/Graph/ros_drive_graph/cutting_head_targets"
    prim = stage.GetPrimAtPath(node_path)
    if not prim:
        prim = stage.DefinePrim(node_path, "OmniGraphNode")

    create_attr(prim, "node:type", Sdf.ValueTypeNames.Token, custom=False, value="omni.graph.scriptnode.ScriptNode")
    create_attr(prim, "node:typeVersion", Sdf.ValueTypeNames.Int, custom=False, value=2)
    create_attr(prim, "inputs:execIn", Sdf.ValueTypeNames.UInt, custom=True, custom_data={"isExecution": True})
    create_attr(prim, "outputs:execOut", Sdf.ValueTypeNames.UInt, custom=True, custom_data={"isExecution": True})
    create_attr(prim, "inputs:script", Sdf.ValueTypeNames.String, custom=True, value="")
    create_attr(
        prim,
        "inputs:scriptPath",
        Sdf.ValueTypeNames.Token,
        custom=True,
        value="cutting_head_controller.py",
    )
    create_attr(prim, "inputs:usePath", Sdf.ValueTypeNames.Bool, custom=True, value=True)
    create_attr(prim, "state:omni_initialized", Sdf.ValueTypeNames.Bool, custom=True, value=False)

    create_attr(prim, "inputs:dt", Sdf.ValueTypeNames.Double, value=0.0)
    create_attr(prim, "inputs:mode", Sdf.ValueTypeNames.Int, value=CUTTING_HEAD_MODE)
    create_attr(prim, "inputs:clamp_arm_rate_limit", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_RATE_LIMIT)
    create_attr(prim, "inputs:clamp_arm_ul_min", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_UL_MIN)
    create_attr(prim, "inputs:clamp_arm_ul_max", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_UL_MAX)
    create_attr(prim, "inputs:clamp_arm_ll_min", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_LL_MIN)
    create_attr(prim, "inputs:clamp_arm_ll_max", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_LL_MAX)
    create_attr(prim, "inputs:clamp_arm_ur_min", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_UR_MIN)
    create_attr(prim, "inputs:clamp_arm_ur_max", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_UR_MAX)
    create_attr(prim, "inputs:clamp_arm_lr_min", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_LR_MIN)
    create_attr(prim, "inputs:clamp_arm_lr_max", Sdf.ValueTypeNames.Double, value=CLAMP_ARM_LR_MAX)
    create_attr(prim, "inputs:feed_arm_left_min", Sdf.ValueTypeNames.Double, value=FEED_ARM_LEFT_MIN)
    create_attr(prim, "inputs:feed_arm_left_max", Sdf.ValueTypeNames.Double, value=FEED_ARM_LEFT_MAX)
    create_attr(prim, "inputs:feed_arm_right_min", Sdf.ValueTypeNames.Double, value=FEED_ARM_RIGHT_MIN)
    create_attr(prim, "inputs:feed_arm_right_max", Sdf.ValueTypeNames.Double, value=FEED_ARM_RIGHT_MAX)
    create_attr(prim, "inputs:feed_arm_rate_limit", Sdf.ValueTypeNames.Double, value=FEED_ARM_RATE_LIMIT)
    create_attr(prim, "inputs:saw_swing_min", Sdf.ValueTypeNames.Double, value=SAW_SWING_MIN)
    create_attr(prim, "inputs:saw_swing_max", Sdf.ValueTypeNames.Double, value=SAW_SWING_MAX)
    create_attr(prim, "inputs:saw_swing_rate_limit", Sdf.ValueTypeNames.Double, value=SAW_SWING_RATE_LIMIT)
    create_attr(prim, "inputs:max_feed_roller_speed", Sdf.ValueTypeNames.Double, value=MAX_FEED_ROLLER_SPEED)
    create_attr(prim, "inputs:feed_roller_accel_limit", Sdf.ValueTypeNames.Double, value=FEED_ROLLER_ACCEL_LIMIT)
    create_attr(
        prim,
        "inputs:max_measuring_wheel_speed",
        Sdf.ValueTypeNames.Double,
        value=MAX_MEASURING_WHEEL_SPEED,
    )
    create_attr(
        prim,
        "inputs:measuring_wheel_accel_limit",
        Sdf.ValueTypeNames.Double,
        value=MEASURING_WHEEL_ACCEL_LIMIT,
    )
    create_attr(prim, "inputs:max_saw_disc_speed", Sdf.ValueTypeNames.Double, value=MAX_SAW_DISC_SPEED)
    create_attr(prim, "inputs:saw_disc_accel_limit", Sdf.ValueTypeNames.Double, value=SAW_DISC_ACCEL_LIMIT)
    create_attr(prim, "inputs:idle_clamp_arm_ul", Sdf.ValueTypeNames.Double, value=IDLE_CLAMP_ARM_UL)
    create_attr(prim, "inputs:idle_clamp_arm_ll", Sdf.ValueTypeNames.Double, value=IDLE_CLAMP_ARM_LL)
    create_attr(prim, "inputs:idle_clamp_arm_ur", Sdf.ValueTypeNames.Double, value=IDLE_CLAMP_ARM_UR)
    create_attr(prim, "inputs:idle_clamp_arm_lr", Sdf.ValueTypeNames.Double, value=IDLE_CLAMP_ARM_LR)
    create_attr(prim, "inputs:idle_feed_arm_left", Sdf.ValueTypeNames.Double, value=IDLE_FEED_ARM_LEFT)
    create_attr(prim, "inputs:idle_feed_arm_right", Sdf.ValueTypeNames.Double, value=IDLE_FEED_ARM_RIGHT)
    create_attr(prim, "inputs:idle_saw_swing", Sdf.ValueTypeNames.Double, value=IDLE_SAW_SWING)
    create_attr(prim, "inputs:idle_feed_roller_speed", Sdf.ValueTypeNames.Double, value=IDLE_FEED_ROLLER_SPEED)
    create_attr(
        prim, "inputs:idle_measuring_wheel_speed", Sdf.ValueTypeNames.Double, value=IDLE_MEASURING_WHEEL_SPEED
    )
    create_attr(prim, "inputs:idle_saw_disc_speed", Sdf.ValueTypeNames.Double, value=IDLE_SAW_DISC_SPEED)
    create_attr(prim, "inputs:grasp_clamp_arm_ul", Sdf.ValueTypeNames.Double, value=GRASP_CLAMP_ARM_UL)
    create_attr(prim, "inputs:grasp_clamp_arm_ll", Sdf.ValueTypeNames.Double, value=GRASP_CLAMP_ARM_LL)
    create_attr(prim, "inputs:grasp_clamp_arm_ur", Sdf.ValueTypeNames.Double, value=GRASP_CLAMP_ARM_UR)
    create_attr(prim, "inputs:grasp_clamp_arm_lr", Sdf.ValueTypeNames.Double, value=GRASP_CLAMP_ARM_LR)
    create_attr(prim, "inputs:grasp_feed_arm_left", Sdf.ValueTypeNames.Double, value=GRASP_FEED_ARM_LEFT)
    create_attr(prim, "inputs:grasp_feed_arm_right", Sdf.ValueTypeNames.Double, value=GRASP_FEED_ARM_RIGHT)
    create_attr(prim, "inputs:grasp_saw_swing", Sdf.ValueTypeNames.Double, value=GRASP_SAW_SWING)
    create_attr(prim, "inputs:grasp_feed_roller_speed", Sdf.ValueTypeNames.Double, value=GRASP_FEED_ROLLER_SPEED)
    create_attr(
        prim,
        "inputs:grasp_measuring_wheel_speed",
        Sdf.ValueTypeNames.Double,
        value=GRASP_MEASURING_WHEEL_SPEED,
    )
    create_attr(prim, "inputs:grasp_saw_disc_speed", Sdf.ValueTypeNames.Double, value=GRASP_SAW_DISC_SPEED)
    create_attr(prim, "inputs:feed_clamp_arm_ul", Sdf.ValueTypeNames.Double, value=FEED_CLAMP_ARM_UL)
    create_attr(prim, "inputs:feed_clamp_arm_ll", Sdf.ValueTypeNames.Double, value=FEED_CLAMP_ARM_LL)
    create_attr(prim, "inputs:feed_clamp_arm_ur", Sdf.ValueTypeNames.Double, value=FEED_CLAMP_ARM_UR)
    create_attr(prim, "inputs:feed_clamp_arm_lr", Sdf.ValueTypeNames.Double, value=FEED_CLAMP_ARM_LR)
    create_attr(prim, "inputs:feed_feed_arm_left", Sdf.ValueTypeNames.Double, value=FEED_FEED_ARM_LEFT)
    create_attr(prim, "inputs:feed_feed_arm_right", Sdf.ValueTypeNames.Double, value=FEED_FEED_ARM_RIGHT)
    create_attr(prim, "inputs:feed_saw_swing", Sdf.ValueTypeNames.Double, value=FEED_SAW_SWING)
    create_attr(prim, "inputs:feed_roller_left_speed", Sdf.ValueTypeNames.Double, value=FEED_ROLLER_LEFT_SPEED)
    create_attr(prim, "inputs:feed_roller_right_speed", Sdf.ValueTypeNames.Double, value=FEED_ROLLER_RIGHT_SPEED)
    create_attr(prim, "inputs:feed_measuring_wheel_speed", Sdf.ValueTypeNames.Double, value=FEED_MEASURING_WHEEL_SPEED)
    create_attr(prim, "inputs:feed_saw_disc_speed", Sdf.ValueTypeNames.Double, value=FEED_SAW_DISC_SPEED)
    create_attr(prim, "inputs:cut_clamp_arm_ul", Sdf.ValueTypeNames.Double, value=CUT_CLAMP_ARM_UL)
    create_attr(prim, "inputs:cut_clamp_arm_ll", Sdf.ValueTypeNames.Double, value=CUT_CLAMP_ARM_LL)
    create_attr(prim, "inputs:cut_clamp_arm_ur", Sdf.ValueTypeNames.Double, value=CUT_CLAMP_ARM_UR)
    create_attr(prim, "inputs:cut_clamp_arm_lr", Sdf.ValueTypeNames.Double, value=CUT_CLAMP_ARM_LR)
    create_attr(prim, "inputs:cut_feed_arm_left", Sdf.ValueTypeNames.Double, value=CUT_FEED_ARM_LEFT)
    create_attr(prim, "inputs:cut_feed_arm_right", Sdf.ValueTypeNames.Double, value=CUT_FEED_ARM_RIGHT)
    create_attr(prim, "inputs:cut_saw_swing", Sdf.ValueTypeNames.Double, value=CUT_SAW_SWING)
    create_attr(prim, "inputs:cut_feed_roller_speed", Sdf.ValueTypeNames.Double, value=CUT_FEED_ROLLER_SPEED)
    create_attr(
        prim, "inputs:cut_measuring_wheel_speed", Sdf.ValueTypeNames.Double, value=CUT_MEASURING_WHEEL_SPEED
    )
    create_attr(prim, "inputs:cut_saw_disc_speed", Sdf.ValueTypeNames.Double, value=CUT_SAW_DISC_SPEED)
    create_attr(prim, "outputs:position_command", Sdf.ValueTypeNames.DoubleArray, value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    create_attr(prim, "outputs:velocity_command", Sdf.ValueTypeNames.DoubleArray, value=[0.0, 0.0, 0.0, 0.0])

    prim.CreateAttribute("ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2, custom=False).Set((520.0, -420.0))
    return node_path


def ensure_articulation_controller(stage: Usd.Stage, node_path: str, joint_names: list[str], pos: tuple[float, float]) -> str:
    prim = stage.GetPrimAtPath(node_path)
    if not prim:
        prim = stage.DefinePrim(node_path, "OmniGraphNode")

    create_attr(
        prim,
        "node:type",
        Sdf.ValueTypeNames.Token,
        custom=False,
        value="isaacsim.core.nodes.IsaacArticulationController",
    )
    create_attr(prim, "node:typeVersion", Sdf.ValueTypeNames.Int, custom=False, value=1)
    create_attr(prim, "inputs:execIn", Sdf.ValueTypeNames.UInt, custom=True, custom_data={"isExecution": True})
    create_attr(prim, "inputs:robotPath", Sdf.ValueTypeNames.String, custom=True, value="")
    create_attr(prim, "inputs:jointNames", Sdf.ValueTypeNames.TokenArray, custom=True, value=joint_names)
    create_attr(prim, "inputs:jointIndices", Sdf.ValueTypeNames.IntArray, custom=True, value=[])
    create_attr(prim, "inputs:positionCommand", Sdf.ValueTypeNames.DoubleArray, custom=True, value=[])
    create_attr(prim, "inputs:velocityCommand", Sdf.ValueTypeNames.DoubleArray, custom=True, value=[])
    create_attr(prim, "inputs:effortCommand", Sdf.ValueTypeNames.DoubleArray, custom=True, value=[])
    prim.CreateAttribute("ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2, custom=False).Set(pos)
    return node_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Update /Graph/ros_drive_graph with articulated-vehicle formulas."
    )
    parser.add_argument("usd_path", type=Path)
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="Skip creating a .bak copy next to the USD before editing.",
    )
    args = parser.parse_args()

    usd_path = args.usd_path.resolve()
    if not usd_path.exists():
        raise FileNotFoundError(usd_path)

    if not args.no_backup:
        backup_path = usd_path.with_suffix(usd_path.suffix + ".bak")
        shutil.copy2(usd_path, backup_path)
        print(f"Backup written to: {backup_path}")

    stage = Usd.Stage.Open(str(usd_path))
    if not stage:
        raise RuntimeError(f"Failed to open stage: {usd_path}")

    half_wheelbase = AXLE_SEPARATION / 2.0

    # Preserve the simple authored graph as a fallback, but retune its constants.
    # The articulation controllers will be reconnected to the script node below.
    set_constant(stage, "/Graph/ros_drive_graph/whell_base", STEER_SIGN * half_wheelbase)
    set_constant(stage, "/Graph/ros_drive_graph/wheel_radius", WHEEL_RADIUS)
    set_constant(stage, "/Graph/ros_drive_graph/constant_double", TRACK_WIDTH)
    set_constant(stage, "/Graph/ros_drive_graph/constant_double_01", 0.5)

    connect_input(
        stage,
        "/Graph/ros_drive_graph/divide_01.inputs:a",
        "/Graph/ros_drive_graph/arctangent.outputs:value",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/divide_01.inputs:b",
        "/Graph/ros_drive_graph/constant_double_01.inputs:value",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/make_array.inputs:input0",
        "/Graph/ros_drive_graph/divide_01.outputs:result",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/divide_02.inputs:b",
        "/Graph/ros_drive_graph/wheel_radius.inputs:value",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/divide_03.inputs:b",
        "/Graph/ros_drive_graph/wheel_radius.inputs:value",
    )

    script_node_path = ensure_script_node(stage, usd_path)
    connect_input(
        stage,
        f"{script_node_path}.inputs:execIn",
        "/Graph/ros_drive_graph/on_playback_tick.outputs:tick",
    )
    connect_input(
        stage,
        f"{script_node_path}.inputs:linear_x",
        "/Graph/ros_drive_graph/break_3_vector_linear.outputs:x",
    )
    connect_input(
        stage,
        f"{script_node_path}.inputs:angular_z",
        "/Graph/ros_drive_graph/break_3_vector_angular.outputs:z",
    )
    connect_input(
        stage,
        f"{script_node_path}.inputs:dt",
        "/Graph/ros_drive_graph/on_playback_tick.outputs:deltaSeconds",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/articulation_controller.inputs:execIn",
        f"{script_node_path}.outputs:execOut",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/articulation_controller.inputs:positionCommand",
        f"{script_node_path}.outputs:steer_command",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/articulation_controller_01.inputs:execIn",
        f"{script_node_path}.outputs:execOut",
    )
    connect_input(
        stage,
        "/Graph/ros_drive_graph/articulation_controller_01.inputs:velocityCommand",
        f"{script_node_path}.outputs:wheel_command",
    )

    cutting_head_node_path = ensure_cutting_head_script_node(stage)
    connect_input(
        stage,
        f"{cutting_head_node_path}.inputs:execIn",
        "/Graph/ros_drive_graph/on_playback_tick.outputs:tick",
    )
    connect_input(
        stage,
        f"{cutting_head_node_path}.inputs:dt",
        "/Graph/ros_drive_graph/on_playback_tick.outputs:deltaSeconds",
    )

    head_position_controller = ensure_articulation_controller(
        stage,
        "/Graph/ros_drive_graph/articulation_controller_cutting_head_position",
        ["J_CLAMP_ARM_UL", "J_CLAMP_ARM_LL", "J_CLAMP_ARM_UR", "J_CLAMP_ARM_LR", "J_FEED_ARM_LEFT", "J_FEED_ARM_RIGHT", "J_FELLING_SAW_SWING"],
        (2030.0, -320.0),
    )
    head_velocity_controller = ensure_articulation_controller(
        stage,
        "/Graph/ros_drive_graph/articulation_controller_cutting_head_velocity",
        ["J_FEED_ROLLER_LEFT_SPIN", "J_FEED_ROLLER_RIGHT_SPIN", "J_MEASURING_WHEEL", "J_SAW_DISC_SPIN"],
        (2045.0, -465.0),
    )
    connect_input(
        stage,
        f"{head_position_controller}.inputs:execIn",
        f"{cutting_head_node_path}.outputs:execOut",
    )
    connect_input(
        stage,
        f"{head_position_controller}.inputs:positionCommand",
        f"{cutting_head_node_path}.outputs:position_command",
    )
    connect_input(
        stage,
        f"{head_velocity_controller}.inputs:execIn",
        f"{cutting_head_node_path}.outputs:execOut",
    )
    connect_input(
        stage,
        f"{head_velocity_controller}.inputs:velocityCommand",
        f"{cutting_head_node_path}.outputs:velocity_command",
    )

    root_layer = stage.GetRootLayer()
    root_layer.Save()

    print(f"Updated: {usd_path}")
    print("Formulas:")
    print(f"  steer_target = 2 * atan(({AXLE_SEPARATION / 2.0:.6f} * wz_filtered) / max(|vx_filtered|, {MIN_LINEAR_FOR_STEER:.3f}))")
    print(f"  steer = rate_limit(clamp(steer_target, +/-{MAX_STEER_ANGLE:.3f}), {MAX_STEER_RATE:.3f} rad/s)")
    print(f"  half_track = {TRACK_WIDTH:.6f} * 0.5 = {TRACK_WIDTH / 2.0:.6f}")
    print(f"  wheel_right = (vx_filtered + wz_filtered * {TRACK_WIDTH / 2.0:.6f}) / {WHEEL_RADIUS:.6f}")
    print(f"  wheel_left  = (vx_filtered - wz_filtered * {TRACK_WIDTH / 2.0:.6f}) / {WHEEL_RADIUS:.6f}")
    print("Cutting-head defaults:")
    print(f"  mode                   = {CUTTING_HEAD_MODE}")
    print(f"  grasp clamp arms       = ({GRASP_CLAMP_ARM_UL:.3f}, {GRASP_CLAMP_ARM_LL:.3f}, {GRASP_CLAMP_ARM_UR:.3f}, {GRASP_CLAMP_ARM_LR:.3f}) rad")
    print(f"  idle feed arms         = ({IDLE_FEED_ARM_LEFT:.3f}, {IDLE_FEED_ARM_RIGHT:.3f}) rad")
    print(f"  feed roller speeds     = ({FEED_ROLLER_LEFT_SPEED:.3f}, {FEED_ROLLER_RIGHT_SPEED:.3f}) rad/s")
    print(f"  cut saw swing          = {CUT_SAW_SWING:.3f} rad")
    print(f"  cut saw disc speed     = {CUT_SAW_DISC_SPEED:.3f} rad/s")


if __name__ == "__main__":
    main()
