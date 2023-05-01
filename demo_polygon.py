import gtsam
import numpy as np

# Define a custom error function for the loop closing factor
def loop_error(this: gtsam.CustomFactor, values: gtsam.Values, H: np.ndarray):
    # Get the poses from the values
    pose1 = values.atPose2(this.keys()[0])
    pose2 = values.atPose2(this.keys()[1])

    # Get the measurement from the factor
    measurement = this.measured()

    # Compute the error between the poses and the measurement
    error = pose1.between(pose2).localCoordinates(measurement)

    # Optionally, compute the Jacobians using gtsam.Pose2 methods
    H1 = np.zeros((3, 3))
    H2 = np.zeros((3, 3))
    pose1.between(pose2, H1, H2)
    H[0] = -H1
    H[1] = -H2

    return error

# Create a factor graph
graph = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()


odomNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))


n = 12
dt = 0.1
cur_pose = gtsam.Pose2(0., 0., 0.)
v = np.array([0.2, 0, 0.45])
for i in range(n):
    initial.insert(i, cur_pose)
    delta = gtsam.Pose2(v * 0.1)
    cur_pose = cur_pose * delta
    graph.add(gtsam.BetweenFactorPose2(i, (i+1)%n, delta, odomNoise))

# Optimize the factor graph using Levenberg-Marquardt algorithm
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
result = optimizer.optimize()

# Print the optimized poses
print("Final Result:")
for i in range(n):
    print(f"Pose {i}: {result.atPose2(i)}")

import gtsam.plot

# Plot the factor graph and the optimized poses
gtsam.plot.plot_2d_graph(graph, result)
