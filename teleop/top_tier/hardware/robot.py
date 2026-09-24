import time
from abc import ABC, abstractmethod
from enum import Enum

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.utils.crc import CRC

from teleop.top_tier.hardware.extensions import R1LocoClient, G1LocoClient
from teleop.top_tier.hardware.joint import Joint, JointType
from teleop.top_tier.general.constants import NETWORK_INTERFACE


_G1_JOINTS = {
    # Left arm
    JointType.LeftShoulderPitch: Joint(JointType.LeftShoulderPitch, 15, -3.0892, 2.6704, 40, 1),
    JointType.LeftShoulderRoll: Joint(JointType.LeftShoulderRoll, 16, -1.5882, 2.2515, 40, 1),
    JointType.LeftShoulderYaw: Joint(JointType.LeftShoulderYaw, 17, -2.618, 2.618, 40, 1),
    JointType.LeftElbow: Joint(JointType.LeftElbow, 18, -1.0472, 2.0944, 40, 1),
    JointType.LeftWristRoll: Joint(JointType.LeftWristRoll, 19, -1.972222054, 1.972222054, 40, 1),
    JointType.LeftWristPitch: Joint(JointType.LeftWristPitch, 20, -1.614429558, 1.614429558, 40, 1),
    JointType.LeftWristYaw: Joint(JointType.LeftWristYaw, 21, -1.614429558, 1.614429558, 40, 1),

    # Right arm
    JointType.RightShoulderPitch: Joint(JointType.RightShoulderPitch, 22, -3.0892, 2.6704, 40, 1),
    JointType.RightShoulderRoll: Joint(JointType.RightShoulderRoll, 23, -2.2515, 1.5882, 40, 1),
    JointType.RightShoulderYaw: Joint(JointType.RightShoulderYaw, 24, -2.618, 2.618, 40, 1),
    JointType.RightElbow: Joint(JointType.RightElbow, 25, -1.0472, 2.0944, 40, 1),
    JointType.RightWristRoll: Joint(JointType.RightWristRoll, 26, -1.972222054, 1.972222054, 40, 1),
    JointType.RightWristPitch: Joint(JointType.RightWristPitch, 27, -1.614429558, 1.614429558, 40, 1),
    JointType.RightWristYaw: Joint(JointType.RightWristYaw, 28, -1.614429558, 1.614429558, 40, 1),

    # Waist
    JointType.WaistYaw: Joint(JointType.WaistYaw, 12, -2.618, 2.618, 60, 1),
    JointType.WaistRoll: Joint(JointType.WaistRoll, 13, -0.52, 0.52, 40, 1),
    JointType.WaistPitch: Joint(JointType.WaistPitch, 14, -0.52, 0.52, 40, 1),

    # Left leg
    JointType.LeftHipPitch: Joint(JointType.LeftHipPitch, 0, -2.5307, 2.8798, 60, 1),
    JointType.LeftHipRoll: Joint(JointType.LeftHipRoll, 1, -0.5236, 2.9671, 60, 1),
    JointType.LeftHipYaw: Joint(JointType.LeftHipYaw, 2, -2.7576, 2.7576, 60, 1),
    JointType.LeftKnee: Joint(JointType.LeftKnee, 3, -0.087267, 2.8798, 100, 2),
    JointType.LeftAnklePitch: Joint(JointType.LeftAnklePitch, 4, -0.87267, 0.5236, 40, 1),
    JointType.LeftAnkleRoll: Joint(JointType.LeftAnkleRoll, 5, -0.2618, 0.2618, 40, 1),

    # Right leg
    JointType.RightHipPitch: Joint(JointType.RightHipPitch, 6, -2.5307, 2.8798, 60, 1),
    JointType.RightHipRoll: Joint(JointType.RightHipRoll, 7, -2.9671, 0.5236, 60, 1),
    JointType.RightHipYaw: Joint(JointType.RightHipYaw, 8, -2.7576, 2.7576, 60, 1),
    JointType.RightKnee: Joint(JointType.RightKnee, 9, -0.087267, 2.8798, 100, 2),
    JointType.RightAnklePitch: Joint(JointType.RightAnklePitch, 10, -0.87267, 0.5236, 40, 1),
    JointType.RightAnkleRoll: Joint(JointType.RightAnkleRoll, 11, -0.2618, 0.2618, 40, 1),
}
_R1_JOINTS = {
    # Head
    JointType.HeadPitch: Joint(JointType.HeadPitch, 29, -0.6283, 0.6283, 50, 2),
    JointType.HeadYaw: Joint(JointType.HeadYaw, 30, -2.0071, 2.0071, 10, 0.1),

    # Left arm
    JointType.LeftShoulderPitch: Joint(JointType.LeftShoulderPitch, 15, -3.1416, 2.0944, 100, 2),
    JointType.LeftShoulderRoll: Joint(JointType.LeftShoulderRoll, 16, -0.2269, 2.4784, 100, 2),
    JointType.LeftShoulderYaw: Joint(JointType.LeftShoulderYaw, 17, -1.9199, 1.9199, 100, 2),
    JointType.LeftElbow: Joint(JointType.LeftElbow, 18, -0.9757, 2.1850, 100, 2),
    JointType.LeftWristRoll: Joint(JointType.LeftWristRoll, 19, -1.9199, 1.9199, 50, 2),

    # Right arm
    JointType.RightShoulderPitch: Joint(JointType.RightShoulderPitch, 22, -3.1416, 2.0944, 100, 2),
    JointType.RightShoulderRoll: Joint(JointType.RightShoulderRoll, 23, -2.4784, 0.2269, 100, 2),
    JointType.RightShoulderYaw: Joint(JointType.RightShoulderYaw, 24, -1.9199, 1.9199, 100, 2),
    JointType.RightElbow: Joint(JointType.RightElbow, 25, -0.9757, 2.1850, 100, 2),
    JointType.RightWristRoll: Joint(JointType.RightWristRoll, 26, -1.9199, 1.9199, 50, 2),

    # Waist
    JointType.WaistRoll: Joint(JointType.WaistRoll, 12, -0.5236, 0.5236, 300, 5),
    JointType.WaistYaw: Joint(JointType.WaistYaw, 13, -2.618, 2.618, 300, 5),

    # Left leg
    JointType.LeftHipPitch: Joint(JointType.LeftHipPitch, 0, -2.9322, 2.5482, 200, 3),
    JointType.LeftHipRoll: Joint(JointType.LeftHipRoll, 1, -1.0472, 1.7453, 200, 3),
    JointType.LeftHipYaw: Joint(JointType.LeftHipYaw, 2, -2.7402, 2.7402, 200, 3),
    JointType.LeftKnee: Joint(JointType.LeftKnee, 3, -0.1745, 2.4260, 200, 3),
    JointType.LeftAnklePitch: Joint(JointType.LeftAnklePitch, 4, -0.8727, 0.5760, 200, 3),
    JointType.LeftAnkleRoll: Joint(JointType.LeftAnkleRoll, 5, -0.2618, 0.2618, 200, 3),

    # Right leg
    JointType.RightHipPitch: Joint(JointType.RightHipPitch, 6, -2.9322, 2.5482, 200, 3),
    JointType.RightHipRoll: Joint(JointType.RightHipRoll, 7, -1.7453, 1.0472, 200, 3),
    JointType.RightHipYaw: Joint(JointType.RightHipYaw, 8, -2.7402, 2.7402, 200, 3),
    JointType.RightKnee: Joint(JointType.RightKnee, 9, -0.1745, 2.4260, 200, 3),
    JointType.RightAnklePitch: Joint(JointType.RightAnklePitch, 10, -0.8727, 0.5760, 200, 3),
    JointType.RightAnkleRoll: Joint(JointType.RightAnkleRoll, 11, -0.2618, 0.2618, 200, 3),
}


class RobotType(Enum):
    G1 = 0
    R1 = 1

    @staticmethod
    def from_str(string: str) -> 'RobotType':
        """
        Args:
            string: a string representation of a RobotType

        Returns:
            The RobotType represented by the given string
        """
        return RobotType[string.upper()]

    def get_joint_map(self) -> dict[JointType, Joint]:
        if self == RobotType.G1:
            return _G1_JOINTS
        return _R1_JOINTS


class Robot:
    """
    Base class for all types of robots
    """
    def __init__(self, robot_type: RobotType):
        """
        Create a new instance of a robot

        Args:
            robot_type: the type of robot this instance represents
        """
        self._robot_type = robot_type

        ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        self._loco_client = G1LocoClient() if robot_type == RobotType.G1 else R1LocoClient()
        self._loco_client.SetTimeout(1.0)

        self._joints = self._robot_type.get_joint_map()
        self._state_sub = ChannelSubscriber(
            "rt/lowstate",
            LowState_,
        )
        self._arm_pub = ChannelPublisher(
            "rt/arm_sdk",
            LowCmd_,
        )
        self._arm_cmd = LowCmd_()
        self._crc = CRC()

    @property
    def robot_type(self) -> RobotType:
        """
        Returns:
            the type of robot this object represents
        """
        return self._robot_type

    @property
    def loco_client(self) -> G1LocoClient | R1LocoClient:
        """
        Returns:
            the LocoClient for this robot
        """
        return self._loco_client

    def initialize(self) -> None:
        """
        Initialize this robot
        """
        self._loco_client.Init()
        self._state_sub.Init()
        self._arm_pub.Init()

    def enable_low_level_arm_control(self) -> None:
        """
        Sets the relevant value so the upper body will start listening to low level control
        """
        pass # TODO

    def release_low_level_arm_control(self) -> None:
        """
        Slowly ramps down the relevant value so the upper body will no longer listen to low level control
        """
        pass # TODO

    def get_low_level_state(self) -> LowState_:
        """
        Returns:
            the current low-level robot state
        """
        state: LowState_ | None = None

        while state is None:
            state = self._state_sub.Read()
            time.sleep(0.01)

        return state

    def set_upper_body_position(self) -> None:
        """
        Set the position of the upper body

        Args:
            TODO (need some repr of pos, max allowed vel?)
        """
        pass
