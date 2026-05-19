import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import X

ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.2, 0.2, 0.1])
)

def add_pose(graph, initial_estimate):
    # Rotate 45° and move ~2m
    odometry = gtsam.Pose2(
        2 * math.cos(math.pi / 4),
        2 * math.sin(math.pi / 4),
        math.pi / 4
    )

    # Add odometry factor
    graph.add(
        gtsam.BetweenFactorPose2(
            X(3),
            X(4),
            odometry,
            ODOMETRY_NOISE
        )
    )

    # Initial estimate
    prev_pose = initial_estimate.atPose2(X(3))
    pose4_guess = prev_pose.compose(odometry)

    initial_estimate.insert(X(4), pose4_guess)

    return graph, initial_estimate