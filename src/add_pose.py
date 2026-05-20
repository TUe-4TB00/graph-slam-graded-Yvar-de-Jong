
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    
    dx = math.sqrt(2)
    dy = math.sqrt(2)
    dtheta = math.pi / 2
    odometry = gtsam.Pose2(dx, dy, dtheta)

    # 2. Add the factor connecting X(3) to X(4)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry, ODOMETRY_NOISE))

   
    target_x = 4.0 + math.sqrt(2)
    target_y = math.sqrt(2)
    target_theta = math.pi / 2
    
    pose_4_estimate = gtsam.Pose2(target_x, target_y, target_theta)
    
    # 4. Insert the clean estimate into the dictionary
    initial_estimate.insert(X(4), pose_4_estimate)

    return graph, initial_estimate