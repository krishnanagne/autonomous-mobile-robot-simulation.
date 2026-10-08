import numpy as np


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def compute_control(
    x,
    y,
    theta,
    target_x,
    target_y,
    sensor_angles=None,
    sensor_distances=None
):

    # -----------------------------
    # Target tracking
    # -----------------------------

    dx = target_x - x
    dy = target_y - y

    distance = np.sqrt(dx**2 + dy**2)

    target_angle = np.arctan2(dy, dx)

    heading_error = normalize_angle(
        target_angle - theta
    )

    # Controller gains
    Kp_distance = 0.8
    Kp_angle = 2.0

    # Basic target-following control
    v = Kp_distance * distance
    omega = Kp_angle * heading_error

    # -----------------------------
    # Obstacle avoidance
    # -----------------------------

    if sensor_angles is not None and sensor_distances is not None:

        closest_index = np.argmin(sensor_distances)

        closest_distance = sensor_distances[closest_index]
        closest_angle = sensor_angles[closest_index]

        safety_distance = 2.0

        if closest_distance < safety_distance:

            # Determine obstacle direction
            relative_angle = normalize_angle(
                closest_angle - theta
            )

            # Slow down near obstacle
            v = 0.3

            # Turn away from obstacle
            if relative_angle > 0:
                omega -= 1.5
            else:
                omega += 1.5

    # Limit velocities
    v = min(v, 1.5)
    omega = max(min(omega, 2.0), -2.0)

    return v, omega