import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Robot initial state
# -----------------------------
x = 1.0
y = 1.0
theta = 0.0

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

for _ in range(time_steps):

    # Store current position
    trajectory_x.append(x)
    trajectory_y.append(y)

    # Update robot state
    x = x + v * np.cos(theta) * dt
    y = y + v * np.sin(theta) * dt
    theta = theta + omega * dt

# -----------------------------
# Plot environment
# -----------------------------
plt.figure(figsize=(8, 8))

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