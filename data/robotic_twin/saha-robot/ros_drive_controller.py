import math


def _clamp(value, low, high):
    return max(low, min(high, value))


def _rate_limit(current, target, limit_per_sec, dt):
    if dt <= 0.0 or limit_per_sec <= 0.0:
        return target
    max_delta = limit_per_sec * dt
    delta = _clamp(target - current, -max_delta, max_delta)
    return current + delta


def setup(db):
    state = db.per_instance_state
    state.filtered_vx = 0.0
    state.filtered_wz = 0.0
    state.filtered_steer = 0.0


def compute(db):
    state = db.per_instance_state

    dt = float(db.inputs.dt) if db.inputs.dt and db.inputs.dt > 0.0 else 0.0
    vx_cmd = float(db.inputs.linear_x)
    wz_cmd = float(db.inputs.angular_z)

    vx_cmd = _clamp(vx_cmd, -db.inputs.max_linear_speed, db.inputs.max_linear_speed)
    wz_cmd = _clamp(wz_cmd, -db.inputs.max_angular_speed, db.inputs.max_angular_speed)

    accel_limit = db.inputs.max_linear_accel if vx_cmd >= state.filtered_vx else db.inputs.max_linear_decel
    state.filtered_vx = _rate_limit(state.filtered_vx, vx_cmd, accel_limit, dt)
    state.filtered_wz = _rate_limit(state.filtered_wz, wz_cmd, db.inputs.max_angular_accel, dt)

    vx_for_steer = state.filtered_vx
    if abs(vx_for_steer) < db.inputs.min_linear_for_steer:
        if abs(vx_for_steer) < 1e-6:
            vx_for_steer = db.inputs.min_linear_for_steer
        else:
            vx_for_steer = math.copysign(db.inputs.min_linear_for_steer, vx_for_steer)

    steer_target = 2.0 * math.atan((0.5 * db.inputs.wheel_base * state.filtered_wz) / vx_for_steer)
    steer_target *= db.inputs.steer_sign
    steer_target = _clamp(steer_target, -db.inputs.max_steer_angle, db.inputs.max_steer_angle)
    state.filtered_steer = _rate_limit(
        state.filtered_steer, steer_target, db.inputs.max_steer_rate, dt
    )

    wheel_radius = max(db.inputs.wheel_radius, 1e-6)
    half_track = 0.5 * db.inputs.track_width
    right_speed = (state.filtered_vx + state.filtered_wz * half_track) / wheel_radius
    left_speed = (state.filtered_vx - state.filtered_wz * half_track) / wheel_radius

    max_wheel_speed = db.inputs.max_wheel_speed
    if max_wheel_speed > 0.0:
        right_speed = _clamp(right_speed, -max_wheel_speed, max_wheel_speed)
        left_speed = _clamp(left_speed, -max_wheel_speed, max_wheel_speed)

    db.outputs.steer_command = [state.filtered_steer]
    db.outputs.wheel_command = [right_speed, left_speed, right_speed, left_speed]
    return True
