MODE_IDLE = 0
MODE_GRASP = 1
MODE_FEED = 2
MODE_CUT = 3


def _clamp(value, low, high):
    return max(low, min(high, value))


def _rate_limit(current, target, limit_per_sec, dt):
    if dt <= 0.0 or limit_per_sec <= 0.0:
        return target
    max_delta = limit_per_sec * dt
    delta = _clamp(target - current, -max_delta, max_delta)
    return current + delta


def _mode_targets(db, mode):
    if mode == MODE_GRASP:
        return (
            float(db.inputs.grasp_clamp_arm_ul),
            float(db.inputs.grasp_clamp_arm_ll),
            float(db.inputs.grasp_clamp_arm_ur),
            float(db.inputs.grasp_clamp_arm_lr),
            float(db.inputs.grasp_feed_arm_left),
            float(db.inputs.grasp_feed_arm_right),
            float(db.inputs.grasp_saw_swing),
            float(db.inputs.grasp_feed_roller_speed),
            float(db.inputs.grasp_feed_roller_speed),
            float(db.inputs.grasp_measuring_wheel_speed),
            float(db.inputs.grasp_saw_disc_speed),
        )
    if mode == MODE_FEED:
        return (
            float(db.inputs.feed_clamp_arm_ul),
            float(db.inputs.feed_clamp_arm_ll),
            float(db.inputs.feed_clamp_arm_ur),
            float(db.inputs.feed_clamp_arm_lr),
            float(db.inputs.feed_feed_arm_left),
            float(db.inputs.feed_feed_arm_right),
            float(db.inputs.feed_saw_swing),
            float(db.inputs.feed_roller_left_speed),
            float(db.inputs.feed_roller_right_speed),
            float(db.inputs.feed_measuring_wheel_speed),
            float(db.inputs.feed_saw_disc_speed),
        )
    if mode == MODE_CUT:
        return (
            float(db.inputs.cut_clamp_arm_ul),
            float(db.inputs.cut_clamp_arm_ll),
            float(db.inputs.cut_clamp_arm_ur),
            float(db.inputs.cut_clamp_arm_lr),
            float(db.inputs.cut_feed_arm_left),
            float(db.inputs.cut_feed_arm_right),
            float(db.inputs.cut_saw_swing),
            float(db.inputs.cut_feed_roller_speed),
            float(db.inputs.cut_feed_roller_speed),
            float(db.inputs.cut_measuring_wheel_speed),
            float(db.inputs.cut_saw_disc_speed),
        )
    return (
        float(db.inputs.idle_clamp_arm_ul),
        float(db.inputs.idle_clamp_arm_ll),
        float(db.inputs.idle_clamp_arm_ur),
        float(db.inputs.idle_clamp_arm_lr),
        float(db.inputs.idle_feed_arm_left),
        float(db.inputs.idle_feed_arm_right),
        float(db.inputs.idle_saw_swing),
        float(db.inputs.idle_feed_roller_speed),
        float(db.inputs.idle_feed_roller_speed),
        float(db.inputs.idle_measuring_wheel_speed),
        float(db.inputs.idle_saw_disc_speed),
    )


def setup(db):
    state = db.per_instance_state
    state.clamp_arm_ul = float(db.inputs.idle_clamp_arm_ul)
    state.clamp_arm_ll = float(db.inputs.idle_clamp_arm_ll)
    state.clamp_arm_ur = float(db.inputs.idle_clamp_arm_ur)
    state.clamp_arm_lr = float(db.inputs.idle_clamp_arm_lr)
    state.feed_arm_left = float(db.inputs.idle_feed_arm_left)
    state.feed_arm_right = float(db.inputs.idle_feed_arm_right)
    state.saw_swing = float(db.inputs.idle_saw_swing)
    state.feed_roller_left_speed = float(db.inputs.idle_feed_roller_speed)
    state.feed_roller_right_speed = float(db.inputs.idle_feed_roller_speed)
    state.measuring_wheel_speed = float(db.inputs.idle_measuring_wheel_speed)
    state.saw_disc_speed = float(db.inputs.idle_saw_disc_speed)


def compute(db):
    state = db.per_instance_state
    dt = float(db.inputs.dt) if db.inputs.dt and db.inputs.dt > 0.0 else 0.0

    mode = int(db.inputs.mode)
    (
        clamp_ul_target,
        clamp_ll_target,
        clamp_ur_target,
        clamp_lr_target,
        left_target,
        right_target,
        saw_target,
        roller_left_target,
        roller_right_target,
        measuring_target,
        saw_disc_target,
    ) = _mode_targets(db, mode)

    clamp_ul_target = _clamp(
        clamp_ul_target, float(db.inputs.clamp_arm_ul_min), float(db.inputs.clamp_arm_ul_max)
    )
    clamp_ll_target = _clamp(
        clamp_ll_target, float(db.inputs.clamp_arm_ll_min), float(db.inputs.clamp_arm_ll_max)
    )
    clamp_ur_target = _clamp(
        clamp_ur_target, float(db.inputs.clamp_arm_ur_min), float(db.inputs.clamp_arm_ur_max)
    )
    clamp_lr_target = _clamp(
        clamp_lr_target, float(db.inputs.clamp_arm_lr_min), float(db.inputs.clamp_arm_lr_max)
    )
    left_target = _clamp(left_target, float(db.inputs.feed_arm_left_min), float(db.inputs.feed_arm_left_max))
    right_target = _clamp(right_target, float(db.inputs.feed_arm_right_min), float(db.inputs.feed_arm_right_max))
    saw_target = _clamp(saw_target, float(db.inputs.saw_swing_min), float(db.inputs.saw_swing_max))
    roller_left_target = _clamp(
        roller_left_target, -float(db.inputs.max_feed_roller_speed), float(db.inputs.max_feed_roller_speed)
    )
    roller_right_target = _clamp(
        roller_right_target, -float(db.inputs.max_feed_roller_speed), float(db.inputs.max_feed_roller_speed)
    )
    measuring_target = _clamp(
        measuring_target, -float(db.inputs.max_measuring_wheel_speed), float(db.inputs.max_measuring_wheel_speed)
    )
    saw_disc_target = _clamp(
        saw_disc_target, -float(db.inputs.max_saw_disc_speed), float(db.inputs.max_saw_disc_speed)
    )

    state.clamp_arm_ul = _rate_limit(state.clamp_arm_ul, clamp_ul_target, float(db.inputs.clamp_arm_rate_limit), dt)
    state.clamp_arm_ll = _rate_limit(state.clamp_arm_ll, clamp_ll_target, float(db.inputs.clamp_arm_rate_limit), dt)
    state.clamp_arm_ur = _rate_limit(state.clamp_arm_ur, clamp_ur_target, float(db.inputs.clamp_arm_rate_limit), dt)
    state.clamp_arm_lr = _rate_limit(state.clamp_arm_lr, clamp_lr_target, float(db.inputs.clamp_arm_rate_limit), dt)
    state.feed_arm_left = _rate_limit(state.feed_arm_left, left_target, float(db.inputs.feed_arm_rate_limit), dt)
    state.feed_arm_right = _rate_limit(state.feed_arm_right, right_target, float(db.inputs.feed_arm_rate_limit), dt)
    state.saw_swing = _rate_limit(state.saw_swing, saw_target, float(db.inputs.saw_swing_rate_limit), dt)
    state.feed_roller_left_speed = _rate_limit(
        state.feed_roller_left_speed, roller_left_target, float(db.inputs.feed_roller_accel_limit), dt
    )
    state.feed_roller_right_speed = _rate_limit(
        state.feed_roller_right_speed, roller_right_target, float(db.inputs.feed_roller_accel_limit), dt
    )
    state.measuring_wheel_speed = _rate_limit(
        state.measuring_wheel_speed, measuring_target, float(db.inputs.measuring_wheel_accel_limit), dt
    )
    state.saw_disc_speed = _rate_limit(
        state.saw_disc_speed, saw_disc_target, float(db.inputs.saw_disc_accel_limit), dt
    )

    db.outputs.position_command = [
        state.clamp_arm_ul,
        state.clamp_arm_ll,
        state.clamp_arm_ur,
        state.clamp_arm_lr,
        state.feed_arm_left,
        state.feed_arm_right,
        state.saw_swing,
    ]
    db.outputs.velocity_command = [
        state.feed_roller_left_speed,
        state.feed_roller_right_speed,
        state.measuring_wheel_speed,
        state.saw_disc_speed,
    ]
    return True
