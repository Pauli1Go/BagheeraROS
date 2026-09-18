"""Controller-axis mapping kept independent from pygame and ROS for testing."""


def apply_deadzone(value: float, deadzone: float) -> float:
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    value = min(max(float(value), -1.0), 1.0)
    if abs(value) <= deadzone:
        return 0.0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return magnitude if value > 0.0 else -magnitude


def axes_to_twist(
    throttle_axis: float,
    steering_axis: float,
    deadzone: float,
    max_linear_speed: float,
    max_angular_speed: float,
):
    # SDL axis values are positive backward/right. ROS is forward/CCW positive.
    throttle = apply_deadzone(throttle_axis, deadzone)
    steering = apply_deadzone(steering_axis, deadzone)
    linear = -throttle * max_linear_speed
    # Manual mapping deliberately separates translation and rotation: while
    # the left stick commands motion, steering is ignored. The right stick
    # therefore only produces an in-place turn from a standstill.
    angular = 0.0 if throttle else -steering * max_angular_speed
    return linear, angular
