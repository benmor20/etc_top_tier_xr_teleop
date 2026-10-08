import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from top_tier.hardware.robot import Robot, RobotType
from top_tier.general.constants import NETWORK_INTERFACE
from top_tier.general.motion_data import MOTION_DATA_DICT
from top_tier.general.xr_controllers import XRControllerButton


def main():
    ChannelFactoryInitialize(0, NETWORK_INTERFACE)
    robot = Robot(RobotType.G1)
    robot.initialize()
    print("Initialized")
    time.sleep(1.)
    robot.activate_control()

    print("Queuing motion")
    robot.move_to_waypoints(MOTION_DATA_DICT[XRControllerButton.X], block=True)

    time.sleep(10.)
    robot.shutdown(False)


if __name__ == '__main__':
    main()
