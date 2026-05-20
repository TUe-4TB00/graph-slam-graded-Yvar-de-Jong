import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # 1. Extract the optimized 2D pose of X(4) and 2D point of L(2) from the result
    pose_4 = result.atPose2(X(4))
    landmark_2 = result.atPoint2(L(2))
    
    # 2. Dynamically calculate the bearing and range from the pose to the landmark
    # pose_4.bearing(landmark_2) calculates the relative Rot2 angle to the landmark
    # pose_4.range(landmark_2) calculates the Euclidean distance to the landmark
    bearing_rot2 = pose_4.bearing(landmark_2)
    distance = pose_4.range(landmark_2)
    
    # 3. Add the factor to the graph using the calculated bearing (Rot2 object) and distance
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), bearing_rot2, distance, MEASUREMENT_NOISE))
    
    return graph