# SAHA Harvester `ros_drive_graph` Notes

## Scope

This document records the current `/Graph/ros_drive_graph` control design in:

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd)

and the sidecar controller script used by the graph:

- [data/robotic_twin/saha-robot/ros_drive_controller.py](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/ros_drive_controller.py)

It also captures engineering guidance for future articulated forestry-machine control design.

## Why The Graph Was Changed

The original `ros_drive_graph` had two structural problems:

1. It mapped `/cmd_vel` into steering and wheel speed using a very raw kinematic approximation with no low-speed protection, no steering saturation, and no rate limiting.
2. It used wheel-speed scaling that was inconsistent with the intended wheel radius assumptions.

For a large articulated harvester, that produces visibly unrealistic behavior:

- steering becomes too aggressive near `vx ~= 0`
- wheel speeds do not track expected body speed cleanly
- the articulation joint can be commanded faster than a heavy chassis should respond

## Current Control Path

The graph now routes `/cmd_vel` through a ScriptNode:

- `/Graph/ros_drive_graph/cmd_vel_filter`

The articulation controllers consume the ScriptNode outputs:

- `J_STEER` position command from `outputs:steer_command`
- wheel velocity commands for:
  - `J_LF_WHEEL`
  - `J_RF_WHEEL`
  - `J_LH_WHEEL`
  - `J_RH_WHEEL`

The old math nodes remain in the graph as fallback/reference, but the live controller path is now the ScriptNode.

## Current Kinematic Assumptions

These values are authored into the ScriptNode inputs:

- `wheel_base = 1.9476267844438553`
- `track_width = 1.5339090526103973`
- `wheel_radius = 0.4`
- `steer_sign = -1.0`

Derived quantity:

- `half_track = 0.7669545263051987`

## Current Control Logic

The ScriptNode applies four layers of processing.

### 1. Command Limits

Incoming `/cmd_vel` is first clamped:

- `max_linear_speed = 2.5 m/s`
- `max_angular_speed = 0.6 rad/s`

### 2. Velocity Smoothing

The commanded chassis speeds are rate-limited:

- `max_linear_accel = 0.8 m/s^2`
- `max_linear_decel = 1.2 m/s^2`
- `max_angular_accel = 0.5 rad/s^2`

This creates filtered internal states:

- `vx_filtered`
- `wz_filtered`

### 3. Steering Protection And Rate Limit

To avoid singular steering at very low forward speed, steering uses:

- `min_linear_for_steer = 0.3 m/s`

The steering target is computed as:

```text
steer_target =
    steer_sign * 2 * atan((0.5 * wheel_base * wz_filtered) / max(|vx_filtered|, min_linear_for_steer))
```

Then it is constrained by:

- `max_steer_angle = 1.0 rad`
- `max_steer_rate = 0.35 rad/s`

This produces the final `J_STEER` command.

### 4. Wheel-Speed Mapping

Wheel angular speeds are computed from filtered body motion:

```text
right_speed = (vx_filtered + wz_filtered * half_track) / wheel_radius
left_speed  = (vx_filtered - wz_filtered * half_track) / wheel_radius
```

The four wheel commands are then assigned as:

```text
[right_speed, left_speed, right_speed, left_speed]
```

with a final saturation:

- `max_wheel_speed = 8.0 rad/s`

## Files Changed

The current setup was implemented in:

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd)
- [data/robotic_twin/saha-robot/ros_drive_controller.py](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/ros_drive_controller.py)
- [tools/update_ros_drive_graph.py](/home/prefor/IsaacForestSim/tools/update_ros_drive_graph.py)

Backup of the pre-edit USD:

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd.bak](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd.bak)

## Practical Validation Checklist

When validating this controller in Isaac Sim, check:

1. Small forward command with zero yaw:
   - wheel speeds should rise smoothly
   - `J_STEER` should stay near zero

2. Small forward command with small positive yaw:
   - `J_STEER` should move gradually, not snap
   - left/right wheel speeds should split consistently

3. Very small `vx` with nonzero `wz`:
   - steering should no longer jump toward a singular angle
   - output should saturate and move slowly

4. Step inputs in `/cmd_vel`:
   - body response should look filtered rather than instant
   - steering should obey a visible rate limit

5. High yaw demand at moderate speed:
   - steering should clamp before unrealistic articulation occurs
   - wheel speed saturation should prevent excessive wheel spin

## Design Guidance For Future Machines

For large articulated forestry machines, avoid treating the platform like a small generic mobile robot. The following rules matter.

### 1. Start From The Real Motion Topology

First decide what the machine actually is:

- rigid differential-drive
- front-axle steering
- skid steer
- center-articulated chassis
- articulated chassis with driven bogies
- articulated chassis plus active suspension/leg mechanisms

The control mapping from `/cmd_vel` must match that topology. If the machine turns primarily through a center articulation joint, the controller should be built around articulation geometry, not reused from a passenger-car or simple differential-drive model.

### 2. Use Real Geometric Quantities

Always derive these from the authored robot geometry or joint frames:

- wheelbase or equivalent articulation length
- track width
- wheel radius
- steering sign conventions
- frame definitions for `base_link`, axle frames, articulation joint frame

Do not rely on approximate constants that came from an earlier prototype unless they are revalidated after asset updates.

### 3. Protect Low-Speed Steering

Any formula containing division by longitudinal speed must explicitly handle:

- `vx -> 0`
- sign flips around zero
- reverse motion

Without this, large articulated machines will show unstable steering commands exactly in the regime where operators do delicate maneuvering.

### 4. Rate-Limit The Chassis, Not Just The Joint

For heavy work machines, command smoothing should exist at the body-motion level:

- linear acceleration limit
- deceleration limit
- angular acceleration limit

and also at the steering actuator level:

- steering angle limit
- steering rate limit

This separates operator intent from actuator feasibility.

### 5. Respect Operating Modes

The same machine may need different limits for:

- transport driving
- precision maneuvering
- boom deployed
- tool engaged with terrain/tree
- reversing

A single fixed `/cmd_vel` mapping is often insufficient. Mode-dependent limits are usually necessary.

### 6. Account For Manipulator State

On a harvester, boom extension and tool pose change the combined inertia and rollover risk. Good control design should eventually reduce allowable:

- maximum linear speed
- maximum angular speed
- steering rate
- curvature

when the boom is extended or carrying load.

### 7. Prefer A Dedicated Low-Level Controller Layer

`/cmd_vel` should be treated as a chassis intent interface, not a direct actuator command. A robust stack is usually:

1. ROS navigation or teleop publishes desired body motion
2. a low-level machine controller converts body motion into feasible articulation and wheel targets
3. articulation and wheel actuators track those feasible targets

This layer is where saturation, smoothing, safety logic, and machine-specific kinematics belong.

### 8. Keep The Graph Explainable

For maintainability, the control graph should make these quantities obvious:

- where `vx` and `wz` enter
- where smoothing happens
- where steering is computed
- where wheel speeds are computed
- where limits are applied

If the graph becomes too opaque with many tiny math nodes, move the logic into a documented ScriptNode or a dedicated extension.

### 9. Validate Against Expected Maneuvers

Do not validate only with one teleop command. Test:

- straight line
- constant-radius turn
- slalom-like alternating steering
- low-speed docking
- reverse turn
- braking from moderate speed

For articulated machines, behavior in reverse and near-zero speed is where simplistic models often fail first.

### 10. Separate Kinematics From Physics Tuning

If the machine still behaves badly after the command mapping is fixed, the next likely issues are:

- joint stiffness/damping
- drive force limits
- tire-ground friction
- mass/inertia distribution
- suspension or leg compliance

Do not try to solve physics problems only by distorting kinematic command formulas.

## Recommended Next Steps

Short term:

1. run the updated graph in Isaac Sim and verify that ScriptNode execution is enabled
2. tune the limits against realistic maneuvering targets
3. record observed steering angle and wheel speeds under representative `/cmd_vel`

Medium term:

1. add mode-dependent speed and steering limits
2. incorporate boom/tool state into allowed motion envelopes
3. consider migrating the ScriptNode logic into a dedicated controller module if this platform will be maintained long-term
