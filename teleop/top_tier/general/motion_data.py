from top_tier.general.xr_controllers import XRControllerButton
from top_tier.hardware.joint import JointType

# maps button presses to list of waypoints representing a motion
MOTION_DATA_DICT = {
    # Quickdraw
    XRControllerButton.X: [
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
    ]
}
