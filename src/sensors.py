import numpy as np


def ray_distance(
    robot_x,
    robot_y,
    angle,
    obstacles,
    max_range=10.0,
    step=0.05
):
    """
    Simulate a single ray-based distance sensor.

    Returns the distance from the robot
    to the nearest obstacle along the ray.
    """

    distance = 0.0

    while distance <= max_range:

        # Position of the ray at this distance
        ray_x = robot_x + distance * np.cos(angle)
        ray_y = robot_y + distance * np.sin(angle)

        # Check each obstacle
        for obstacle in obstacles:

            x_min, y_min, x_max, y_max = obstacle

            if (
                x_min <= ray_x <= x_max
                and
                y_min <= ray_y <= y_max
            ):
                return distance

        distance += step

    return max_range