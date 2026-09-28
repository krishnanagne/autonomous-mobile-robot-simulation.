import numpy as np
import matplotlib.pyplot as plt
from src.sensors import ray_distance

# -----------------------------
# Robot initial state
# -----------------------------
x = 1.0
y = 1.0
theta = np.pi / 4

# Save initial position for sensor visualization
sensor_start_x = x
sensor_start_y = y

# -----------------------------
# Environment
# -----------------------------
obstacles = [
    (4, 4, 6, 6)
]

# -----------------------------
# Multiple sensors
# -----------------------------
sensor_angles = theta + np.deg2rad([-60, -30, 0, 30, 60])

sensor_distances = []

for angle in sensor_angles:

    distance = ray_distance(
        sensor_start_x,
        sensor_start_y,
        angle,
        obstacles
    )

    sensor_distances.append(distance)

    print(
        f"Sensor angle: {np.rad2deg(angle):.0f}° | "
        f"Distance: {distance:.2f}"
    )

# -----------------------------
# Basic perception
# -----------------------------

closest_distance = min(sensor_distances)

closest_index = sensor_distances.index(closest_distance)

closest_angle = sensor_angles[closest_index]

print("\n--- Perception ---")
print(f"Closest obstacle distance: {closest_distance:.2f}")
print(f"Obstacle direction: {np.rad2deg(closest_angle):.0f}°")

# -----------------------------
# Environment classification
# -----------------------------

clear_distance = 6.0
safety_distance = 3.0

if closest_distance > clear_distance:
    status = "CLEAR"
elif closest_distance > safety_distance:
    status = "CAUTION"
else:
    status = "BLOCKED"

print(f"Environment status: {status}")

# -----------------------------
# Choose safest direction
# -----------------------------

safest_index = np.argmax(sensor_distances)

safest_distance = sensor_distances[safest_index]

safest_angle = sensor_angles[safest_index]

print(f"Safest direction: {np.rad2deg(safest_angle):.0f}°")
print(f"Free distance: {safest_distance:.2f}")

# -----------------------------
# Robot motion parameters
# -----------------------------
v = 1.0          # Linear velocity
omega = 0.25     # Angular velocity
dt = 0.05        # Simulation timestep
simulation_time = 20

# -----------------------------
# Store robot trajectory
# -----------------------------
trajectory_x = []
trajectory_y = []

# -----------------------------
# Simulation loop
# -----------------------------

time_steps = int(simulation_time / dt)

safety_distance = 2.0

for _ in range(time_steps):

    # Store current position
    trajectory_x.append(x)
    trajectory_y.append(y)

    # -----------------------------
    # Sense environment
    # -----------------------------

    distance = ray_distance(
        x,
        y,
        theta,
        obstacles
    )

    # -----------------------------
    # Decide
    # -----------------------------

    if distance < safety_distance:
        omega_control = 1.0
    else:
        omega_control = 0.0

    # -----------------------------
    # Act
    # -----------------------------

    x = x + v * np.cos(theta) * dt
    y = y + v * np.sin(theta) * dt

    theta = theta + omega_control * dt 
# -----------------------------
# Plot environment
# -----------------------------
plt.figure(figsize=(8, 8))

# Draw all sensor rays
for angle, distance in zip(sensor_angles, sensor_distances):

    sensor_x = sensor_start_x + distance * np.cos(angle)
    sensor_y = sensor_start_y + distance * np.sin(angle)

    plt.plot(
        [sensor_start_x, sensor_x],
        [sensor_start_y, sensor_y],
        linestyle="--",
        alpha=0.7
    )

# Robot trajectory
plt.plot(
    trajectory_x,
    trajectory_y,
    label="Robot Trajectory"
)

# Starting position
plt.scatter(
    trajectory_x[0],
    trajectory_y[0],
    s=150,
    label="Start"
)

# Final position
plt.scatter(
    trajectory_x[-1],
    trajectory_y[-1],
    s=150,
    marker="*",
    label="Final Position"
)

# Obstacle
plt.gca().add_patch(
    plt.Rectangle(
        (4, 4),
        2,
        2,
        fill=False,
        linewidth=2,
        label="Obstacle"
    )
)

# Plot limits
plt.xlim(0, 10)
plt.ylim(0, 10)

plt.xlabel("X Position")
plt.ylabel("Y Position")

plt.title("Autonomous Mobile Robot — Motion Simulation")

plt.grid(True)
plt.legend()

plt.show()