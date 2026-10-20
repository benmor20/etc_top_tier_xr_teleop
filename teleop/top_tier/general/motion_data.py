from top_tier.general.xr_controllers import XRControllerButton, XRControllerFloat
from top_tier.hardware.joint import JointType

# maps button presses to list of waypoints representing a motion
MOTION_DATA_DICT = {
    # Quickdraw
    XRControllerButton.A: [
        {
            JointType.RightShoulderPitch: 0.56371,
            JointType.RightShoulderRoll: -0.1229,
            JointType.RightShoulderYaw: -0.22283,
            JointType.RightElbow: 0.72731,
            JointType.RightWristRoll: 0.16336,
            JointType.RightWristPitch: -0.04158,
            JointType.RightWristYaw: 0.22465,
            JointType.WaistYaw: 0.,
        },
        {
            JointType.RightShoulderPitch: 0.57995,
            JointType.RightShoulderRoll: -0.56581,
            JointType.RightShoulderYaw: -0.15564,
            JointType.RightElbow: -0.37211,
            JointType.RightWristRoll: 0.29777,
            JointType.RightWristPitch: 1.32755,
            JointType.RightWristYaw: 0.42441,
            JointType.WaistYaw: -2.617 / 2.,
        },
        {
            JointType.RightShoulderPitch: -1.49102,
            JointType.RightShoulderRoll: -0.06773,
            JointType.RightShoulderYaw: -0.40491,
            JointType.RightElbow: 1.35419,
            JointType.RightWristRoll: -0.10866,
            JointType.RightWristPitch: 0.04102,
            JointType.RightWristYaw: -0.09476,
            JointType.WaistYaw: -2.617,
        }
    ],

    XRControllerButton.B: [
        {
            JointType.WaistYaw: 0.
        }
    ],

    XRControllerButton.X: [
        {
            JointType.LeftThumbYaw: 0.,
        }
    ],

    XRControllerButton.Y: [
        {
            JointType.LeftThumbYaw: 1.047,
        }
    ],
}


HAND_OPEN = {
    XRControllerFloat.LeftTrigger: {
        JointType.LeftThumbYaw: 0.,
        JointType.LeftThumbJoint1: -0.7,
        JointType.LeftThumbJoint2: 0.,
    },
    XRControllerFloat.LeftSqueeze: {
        JointType.LeftIndexFingerJoint1: 0.,
        JointType.LeftIndexFingerJoint2: 0.,
        JointType.LeftMiddleFingerJoint1: 0.,
        JointType.LeftMiddleFingerJoint2: 0.,
    },
    XRControllerFloat.RightTrigger: {
        JointType.RightThumbYaw: 0.,
        JointType.RightThumbJoint1: 0.7,
        JointType.RightThumbJoint2: 0.,
    },
    XRControllerFloat.RightSqueeze: {
        JointType.RightIndexFingerJoint1: 0.,
        JointType.RightIndexFingerJoint2: 0.,
        JointType.RightMiddleFingerJoint1: 0.,
        JointType.RightMiddleFingerJoint2: 0.,
    }
}


HAND_CLOSED = {
    XRControllerFloat.LeftTrigger: {
        JointType.LeftThumbYaw: 0.,
        JointType.LeftThumbJoint1: 0.6,
        JointType.LeftThumbJoint2: 1.74,
    },
    XRControllerFloat.LeftSqueeze: {
        JointType.LeftIndexFingerJoint1: -1.57,
        JointType.LeftIndexFingerJoint2: -1.74,
        JointType.LeftMiddleFingerJoint1: -1.57,
        JointType.LeftMiddleFingerJoint2: -1.74,
    },
    XRControllerFloat.RightTrigger: {
        JointType.RightThumbYaw: 0.,
        JointType.RightThumbJoint1: -0.6,
        JointType.RightThumbJoint2: -1.74,
    },
    XRControllerFloat.RightSqueeze: {
        JointType.RightIndexFingerJoint1: 1.57,
        JointType.RightIndexFingerJoint2: 1.74,
        JointType.RightMiddleFingerJoint1: 1.57,
        JointType.RightMiddleFingerJoint2: 1.74,
    }
}


def interpolate_poses(*pose_and_weights: tuple[dict[JointType, float], float]) -> dict[JointType, float]:
    """
    Interpolate between a set of poses

    Args:
        pose_and_weights: each given argument should be a tuple. The first element is a pose to interpolate (a map of
            joint type to that joint's position in the pose), and the second element is that pose's relative weight. Any
            joints in only some given poses will be interpolated between the poses in which it is given

    Returns:
        a pose mapping joint types given in at least one pose to its interpolated value between all poses in which it is
        given, weighted by that pose's weight
    """
    weighted_sums: dict[JointType, float] = {}
    weight_totals: dict[JointType, float] = {}
    overall_weight = 0.

    for arg in pose_and_weights:
        if isinstance(arg, tuple):
            pose, weight = arg
        else:
            pose = arg
            weight = 1. - overall_weight
        if weight < 0:
            raise ValueError("Cannot have a negative weight in interpolation")
        for joint, position in pose.items():
            weighted_sums[joint] = weighted_sums.get(joint, 0.0) + position * weight
            weight_totals[joint] = weight_totals.get(joint, 0.0) + weight
        overall_weight += weight

    result: dict[JointType, float] = {}
    for joint, total_weight in weight_totals.items():
        if total_weight == 0.:
            raise ValueError(f"Joint {joint} has 0 total weight in interpolation")
        result[joint] = weighted_sums[joint] / total_weight

    return result
