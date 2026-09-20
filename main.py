import numpy as np
import matplotlib.pyplot as plt

# Robot starting position
robot_x = 2
robot_y = 7

# Target position
target_x = 8
target_y = 3

# Plot environment
plt.figure(figsize=(8, 8))

# Robot
plt.scatter(robot_x, robot_y, s=150, label="Robot")

# Target
plt.scatter(target_x, target_y, s=150, marker="*", label="Target")

# Draw line between robot and target
plt.plot(
    [robot_x, target_x],
    [robot_y, target_y],
    linestyle="--",
    label="Direct Path"
)

#Obstacle
plt.gca().add_patch(
    plt.Rectangle((4, 4), 2, 2, fill=False)
)

plt.xlim(0, 10)
plt.ylim(0, 10)

plt.xlabel("X Position")
plt.ylabel("Y Position")

plt.title("Autonomous Mobile Robot Simulation")

plt.grid(True)
plt.legend()

plt.show()