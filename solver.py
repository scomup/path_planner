import gtsam
import numpy as np
from math_tools import *
from typing import List, Optional
from functools import partial
from matplotlib.patches import Polygon
from distance import distance_polygon_to_polygon_2d
from astar import *


def transform_polygon(x, p):
    R, t = makeRt(v2m(x))
    element = int(p.size / 2)
    tp = np.dot(R,p.T).T + np.array([t,]*(element))
    return tp

def get_polygon_dist(x, p1, p2):
    dist = distance_polygon_to_polygon_2d(transform_polygon(x, p1), p2)
    return dist

def get_polygon_error(x, p1, p2, max_dist):
    error = max_dist - get_polygon_dist(x, p1, p2)
    if(error < 0):
        error = 0
    return np.atleast_1d(error)


obstacles = []
robot_polygon = np.array([[-0.5, 0.2], [0.5, 0.2], [0.5, -0.2], [-0.5, -0.2]])

polygons = load_polygons()
for p in polygons:
    obstacles.append(p.get_verts())
#obstacles.append(np.array([[1.3, 1], [2.3, 1], [1.8, 0.23]]))
#obstacles.append(np.array([[0.3, -1],[1.3, -1],[0.8, -0.2]]))

def error_pose2_between(measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    key1 = this.keys()[0]
    key2 = this.keys()[1]
    pose1 = values.atPose2(key1).matrix()
    pose2 = values.atPose2(key2).matrix()
    T12 = np.linalg.inv(pose1) @ pose2
    T21 = np.linalg.inv(T12)
    R21,t21 = makeRt(T21)
    J = np.eye(3)
    J[0:2,0:2] = R21
    J[0:2,2] = -np.array([-t21[1], t21[0]])
    J = -J
    error = m2v(np.linalg.inv(measurement.matrix()) @ (T12))
    if jacobians is not None:
        jacobians[0] = J
        jacobians[1] = np.eye(3)
    return error

def error_polygon_distance(measurement , this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    key = this.keys()[0]
    pose = values.atPose2(key).matrix()
    robot_polygon = measurement[0]
    obstacle = measurement[1]
    max_dist = measurement[2]
    error = get_polygon_error( m2v(pose), robot_polygon, obstacle, max_dist)
    if jacobians is not None:
        jacobians[0] = numericalDerivative(get_polygon_error, [m2v(pose), robot_polygon, obstacle, max_dist], 0)
    return error

class Solver:
    def __init__(self) -> None:
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        self.dist_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2]))
        self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.001, 0.001, 0.001]))
        self.kinematic_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01]))
        self.max_dist = 0.5


    def add_node(self, i, cur_pose):
        self.initial.insert(i, cur_pose)

    def add_odom_factor(self, i, j, delta):
        self.graph.add(gtsam.CustomFactor(self.odom_noise, [i, j],partial(error_pose2_between, delta)))

    def add_polygon_factor(self, i, obstacle):
        dist = get_polygon_dist(m2v(self.initial.atPose2(i).matrix()), robot_polygon, obstacle)
        if(dist > 3):
            return
        obst = gtsam.CustomFactor(self.dist_noise, [i], partial(error_polygon_distance,  [robot_polygon, obstacle, self.max_dist] ))
        self.graph.add(obst)

    def add_prior_factor(self, i):
        self.graph.add( gtsam.PriorFactorPose2(i, self.initial.atPose2(i), self.prior_noise))

    def solve(self):
        params = gtsam.LevenbergMarquardtParams()
        params.setRelativeErrorTol(1e-5)
        params.setMaxIterations(100)
        params.setLinearSolverType("MULTIFRONTAL_CHOLESKY")
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial, params)
        result = optimizer.optimize()
        return result

if __name__ == '__main__':
    solver = Solver()
    n = 20
    dt = 1.
    cur_pose = gtsam.Pose2(0., 0., 0.)
    v = np.array([0.5, 0, 0])
    max_dist = 0.5

    # Add all node.
    for i in range(n):
        solver.add_node(i, cur_pose)
        delta = gtsam.Pose2(v * dt)
        cur_pose = cur_pose * delta

    # Add all odom factor.
    for i in range(n-1):
        solver.add_odom_factor(i, i+1, gtsam.Pose2(v * dt))

    # Add prior pose factor.
    solver.add_prior_factor(0)
    solver.add_prior_factor(n-1)



    # Add all obstacle factor.
    for i in range(n):
        for j in range(len(obstacles)):
            solver.add_polygon_factor(i, obstacles[j])

    result = solver.solve()

    from gtsam.utils.plot import *
    fig_name = "test"
    fig = plt.figure(fig_name)
    axes = fig.gca()
    axes.axis('equal')


    for i in range(n):
        pose = result.atPose2(i)
        plot_pose2(fig_name, pose)
        axes.add_patch( Polygon( transform_polygon( m2v(pose.matrix()), robot_polygon), closed=True, edgecolor = 'g', facecolor = 'g', alpha = 0.2))
        #break
    
    for i in range(len(obstacles)):
        axes.add_patch( Polygon( obstacles[i], closed=True, color = 'b'))
    
    plt.show()
