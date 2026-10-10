# Autonomous Mobile Robot Simulation

A Python-based 2D simulation of autonomous mobile robot navigation, virtual distance sensing, reactive obstacle avoidance, closed-loop control, and position estimation under measurement noise.

## Project Overview

This project simulates a mobile robot navigating from a starting position to a target in a 2D environment containing a rectangular obstacle.

The robot uses five virtual distance-sensing rays and a proportional feedback controller to adjust its linear and angular velocities. Position measurements are corrupted with Gaussian noise, and a moving-average filter estimates the robot's position from recent measurements.

The simulation evaluates navigation performance and compares raw measurement errors with filtered position-estimation errors.

## Features

- **2D Robot Simulation:** Models the robot's position and heading over discrete simulation steps.
- **Virtual Distance Sensing:** Uses five rays at different angles relative to the robot's heading.
- **Reactive Obstacle Avoidance:** Adjusts robot speed and turning based on the closest reported sensor distance.
- **Closed-Loop Control:** Calculates linear and angular velocities using the estimated position, target position, heading, and sensor readings.
- **Noisy Position Measurements:** Adds Gaussian noise independently to measured x and y positions.
- **Moving-Average Filtering:** Smooths position measurements using a rolling window of five samples.
- **Performance Metrics:** Reports navigation status, path length, final target distance, raw measurement error, filtered estimation error, simulated duration, and number of steps.
- **Noise-Level Experiments:** Allows the position-noise parameter to be changed for individual simulation runs.

## Technology Stack

- Python 3
- NumPy
- Matplotlib

## Project Structure

```text
autonomous-mobile-robot-simulation/
├── main.py
├── src/
│   ├── controller.py
│   └── sensors.py
├── .gitignore
├── .gitattributes
└── README.md
```

## System Architecture

The simulation follows this sequence:

1. **Sensing:** Five rays estimate distances to rectangular obstacles.
2. **Measurement:** Gaussian noise is added to the robot's true x and y positions.
3. **Filtering:** A moving average of up to five recent measurements produces filtered x and y estimates.
4. **Control:** The controller calculates linear and angular velocities using the filtered position, simulated heading, target, and sensor readings.
5. **Motion Update:** The robot's position and heading are updated using a discrete-time motion model.
6. **Evaluation:** The simulation calculates navigation and position-estimation metrics and plots the results.

## How the Sensor Model Works

The `ray_distance()` function advances along a ray in increments of 0.05 simulation units and checks whether each sampled point lies inside a rectangular obstacle.

The function returns the first detected sampled distance or the configured maximum range of 10 units when no obstacle is detected.

This is a discrete ray-sampling approximation rather than an exact geometric ray–rectangle intersection calculation.

## How the Controller Works

The `compute_control()` function uses proportional control to calculate linear and angular velocities based on the target's distance and the robot's heading error.

When the closest sensor reading is below the configured safety distance of 2 units, the controller reduces the commanded linear velocity and adjusts the angular velocity to turn away from the sensor direction.

The controller is reactive and does not compute a globally optimal path.

## State Estimation

The simulation adds Gaussian noise to the true x and y positions. A rolling moving-average filter uses up to five recent measurements to calculate filtered position estimates.

The controller uses the filtered x and y coordinates, while the robot's heading comes directly from the simulated state.

The simulation compares raw and filtered position errors against the actual robot positions at the corresponding measurement times.

A moving-average filter can reduce measurement noise, but it can also introduce lag. It is not a Kalman filter, and it does not guarantee lower error in every run.

## Setup and Execution

### Prerequisites

Install Python 3 and pip.

### Install Dependencies

Open a terminal in the project root and run:

```bash
python -m pip install numpy matplotlib
```

### Run the Simulation

```bash
python main.py
```

The program prints performance metrics in the terminal and displays a plot containing the robot trajectory, noisy measurements, filtered estimates, starting position, target, final position, and obstacle.

## Performance Metrics

The simulation reports:

- **Navigation status:** Whether the robot reached the target within the configured tolerance.
- **Path length:** Total distance travelled along the simulated trajectory.
- **Final distance to target:** Distance between the final robot position and the target.
- **Mean raw measurement error:** Average Euclidean error between noisy measurements and actual positions.
- **Mean filtered estimation error:** Average Euclidean error between filtered estimates and actual positions.
- **Simulation duration:** Number of completed steps multiplied by the simulation time step.
- **Simulation steps:** Number of motion updates performed.

## Noise Experiments

The `position_noise` parameter controls the standard deviation of the Gaussian noise applied independently to x and y measurements.

Individual runs can be performed with different values, such as 0.1 and 0.3, by changing this parameter in `main.py`.

Because the noise is randomized and the program does not automatically execute repeated trials, individual runs should be treated as illustrative experiments rather than statistical proof of robustness.

Record results from your own final runs if you want to include specific numerical comparisons.

## Limitations

- The project is a 2D software simulation, not a physical robot implementation.
- Sensor distance estimation uses discrete ray sampling and may not return the exact obstacle-boundary distance.
- Obstacle avoidance is reactive and does not guarantee collision-free navigation in every environment.
- The controller does not perform global path planning.
- Position estimation uses a moving-average filter rather than a Kalman filter.
- The controller uses filtered position estimates but the simulated heading.
- Random measurement noise can cause results to vary between runs.
- The simulation does not yet evaluate performance across a statistically meaningful set of repeated trials.

## Future Improvements

Possible extensions include fixed random seeds for repeatable experiments, evaluation across multiple noise levels and repeated trials, more accurate sensor geometry, improved state estimation, and advanced path-planning algorithms.

## Learning Outcomes

This project explores mobile robot kinematics, virtual distance sensing, proportional feedback control, reactive obstacle avoidance, measurement noise, moving-average filtering, and simulation performance evaluation.