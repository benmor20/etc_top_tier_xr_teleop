import os
import sys
import time

from top_tier.general.motion_data import MOTION_DATA_DICT
from top_tier.general.xr_controllers import XRControllerButton

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from top_tier.hardware.robot import Robot, RobotType


def main():
    robot = Robot(RobotType.G1)
    robot.initialize()
    print("Initialized")
    time.sleep(1.)
    robot.enable_low_level_arm_control()

    robot.loco_client.SetVelocity(0.2, 0., 0., 3.)
    robot.move_to_waypoints(MOTION_DATA_DICT[XRControllerButton.X], block=False)
    print("Queued motion")

    time.sleep(10.)
    robot.shutdown(False)


if __name__ == '__main__':
    main()
