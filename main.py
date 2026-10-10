
import numpy as np
import matplotlib.pyplot as plt

from src.sensors import ray_distance
from src.controller import compute_control


# -----------------------------
# Robot initial state
# -----------------------------
x = 1.0
y = 1.0
theta = np.pi / 4

# Target position
target_x = 8.0
target_y = 8.0

# Environment: (xmin, ymin, xmax, ymax)
obstacles = [
    (4, 4, 6, 6)
]


# -----------------------------
# Simulation parameters
# -----------------------------
dt = 0.05
simulation_time = 30
time_steps = int(simulation_time / dt)

position_noise = 0.1
window_size = 5

# Five sensors relative to robot heading
sensor_offsets = np.deg2rad([-60, -30, 0, 30, 60])


# -----------------------------
# Data storage
# -----------------------------
trajectory_x = [x]
trajectory_y = [y]

measured_x_values = []
measured_y_values = []

filtered_x_values = []
filtered_y_values = []

measurement_window = []

target_reached = False


# -----------------------------
# Simulation loop
# -----------------------------
for step in range(time_steps):

    # 1. Sense the environment
    sensor_angles_current = theta + sensor_offsets

    sensor_distances_current = []

    for angle in sensor_angles_current:
        distance = ray_distance(
            x,
            y,
            angle,
            obstacles
        )
        sensor_distances_current.append(distance)

    # 2. Simulate noisy position measurements
    measured_x = x + np.random.normal(
        0, position_noise
    )
    measured_y = y + np.random.normal(
        0, position_noise
    )

    measured_x_values.append(measured_x)
    measured_y_values.append(measured_y)

    # 3. Update moving-average filter
    measurement_window.append(
        (measured_x, measured_y)
    )

    if len(measurement_window) > window_size:
        measurement_window.pop(0)

    estimated_x = np.mean(
        [p[0] for p in measurement_window]
    )
    estimated_y = np.mean(
        [p[1] for p in measurement_window]
    )

    filtered_x_values.append(estimated_x)
    filtered_y_values.append(estimated_y)

    # 4. Calculate control commands
    v, omega = compute_control(
        estimated_x,
        estimated_y,
        theta,
        target_x,
        target_y,
        sensor_angles_current,
        sensor_distances_current
    )

    # 5. Update actual robot state
    x = x + v * np.cos(theta) * dt
    y = y + v * np.sin(theta) * dt
    theta = theta + omega * dt

    # Normalize orientation
    theta = np.arctan2(
        np.sin(theta),
        np.cos(theta)
    )

    # Store actual trajectory
    trajectory_x.append(x)
    trajectory_y.append(y)

    # 6. Check whether target is reached
    distance_to_target = np.sqrt(
        (target_x - x) ** 2
        + (target_y - y) ** 2
    )

    if distance_to_target < 0.15:
        target_reached = True
        print(f"Target reached at step {step + 1}!")
        break


# -----------------------------
# Simulation results
# -----------------------------
if not target_reached:
    print(
        f"Target not reached. "
        f"Final distance: {distance_to_target:.2f}"
    )

print(f"Final robot position: ({x:.2f}, {y:.2f})")
print(f"Target position: ({target_x:.2f}, {target_y:.2f})")


# -----------------------------
# Plot environment and results
# -----------------------------
plt.figure(figsize=(10, 8))

# Actual robot trajectory
plt.plot(
    trajectory_x,
    trajectory_y,
    color="brown",
    linewidth=2,
    label="Robot Trajectory"
)

# Noisy measurements
plt.scatter(
    measured_x_values,
    measured_y_values,
    s=10,
    alpha=0.4,
    label="Noisy Measurements"
)

# Filtered position estimate
plt.plot(
    filtered_x_values,
    filtered_y_values,
    color="green",
    linewidth=2,
    label="Filtered Estimate"
)

# Starting position
plt.scatter(
    trajectory_x[0],
    trajectory_y[0],
    s=150,
    label="Start"
)

# Target
plt.scatter(
    target_x,
    target_y,
    s=200,
    marker="*",
    color="red",
    label="Target"
)

# Final position
plt.scatter(
    trajectory_x[-1],
    trajectory_y[-1],
    s=120,
    marker="X",
    label="Final Position"
)

# Obstacles
for xmin, ymin, xmax, ymax in obstacles:
    plt.gca().add_patch(
        plt.Rectangle(
            (xmin, ymin),
            xmax - xmin,
            ymax - ymin,
            fill=False,
            linewidth=2,
            color="black"
        )
    )

plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title(
    "Autonomous Mobile Robot — "
    "Closed-Loop Navigation and State Estimation"
)

plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.show()
