import math
import threading
import time
from enum import Enum, auto
from typing import Callable

import numpy as np
from typing_extensions import override
from unitree_sdk2py.idl import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.utils.crc import CRC

from top_tier.general.repeated_event import RepeatedEvent
from top_tier.hardware.extensions import R1LocoClient, G1LocoClient
from top_tier.hardware.joint import Joint, JointType
from top_tier.general.constants import NETWORK_INTERFACE, CONTROL_DT, G1_ARM_SDK_WEIGHT_MOTOR_IDX
from top_tier.general.exceptions import IllegalRobotStateException, IllegalJointCommandException, UnknownRobotException
from top_tier.general.repeated_event import RepeatMode

from teleop.top_tier.hardware.hardware_device import HardwareDevice

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

        Raises:
            KeyError: if the input string is not a robot type
        """
        return RobotType[string.upper()]

    def get_joint_map(self) -> dict[JointType, Joint]:
        if self == RobotType.G1:
            return _G1_JOINTS
        return _R1_JOINTS


class RobotFSMState(Enum):
    Unknown = -1
    ZeroTorque = 0
    Damping = 1
    LockedStand = 4
    G1Walking = 500
    R1Walking = 811
    R1Balancing = 816

    @property
    def can_control_arms(self) -> bool:
        """
        Returns:
            True if this state allows for control of the upper body
        """
        return self in (RobotFSMState.G1Walking, RobotFSMState.R1Walking, RobotFSMState.R1Balancing)


class Robot(HardwareDevice[RobotType, LowState_, LowCmd_]):
    """
    Base class for all types of robots
    """
    def __init__(self, robot_type: RobotType):
        """
        Create a new instance of a robot and do all relevant setup

        Assumes ChannelFactoryInitialize has already been called

        Args:
            robot_type: the type of robot this instance represents
        """
        super().__init__(robot_type, "rt/lowstate", "rt/arm_sdk")

        self._loco_client = G1LocoClient() if robot_type == RobotType.G1 else R1LocoClient()
        self._loco_client.SetTimeout(1.0)
        self._robot_fsm_state = RobotFSMState.ZeroTorque
        self._crc = CRC()

    # ABSTRACT METHODS ----------------------------------------------------------------------------

    def _get_joint_map(self, device_type: RobotType) -> dict[JointType, Joint]:
        """
        Construct a mapping of joint types to the joints they represent for this device

        Args:
            device_type: the type of device this is

        Returns:
            a dict mapping every joint type on this device to its corresponding joint
        """
        return device_type.get_joint_map()

    @property
    def _state_type(self) -> type[LowState_]:
        return LowState_

    @property
    def _cmd_type(self) -> type[LowCmd_]:
        return LowCmd_

    @property
    def _state_default_factory(self) -> Callable[[], LowState_]:
        return unitree_hg_msg_dds__LowState_

    @property
    def _cmd_default_factory(self) -> Callable[[], LowCmd_]:
        return unitree_hg_msg_dds__LowCmd_

    @override
    def is_joint_controlled(self, joint_type: JointType) -> bool:
        """
        Determines if a joint is allowed to be controlled by this robot

        This is true when the robot has the joint and the joint is a upper body joint

        Args:
            joint_type: the joint to check

        Returns:
            a bool, whether the joint can be controlled by this device
        """
        return super().is_joint_controlled(joint_type) and joint_type.is_upper_body

    # MISC PROPERTIES -----------------------------------------------------------------------------

    @property
    def robot_fsm_state(self) -> RobotFSMState:
        """
        Returns:
            the current FSM state the robot is in
        """
        return self._robot_fsm_state

    @property
    def loco_client(self) -> G1LocoClient | R1LocoClient:
        """
        Returns:
            the LocoClient for this robot
        """
        return self._loco_client

    # JOINTS --------------------------------------------------------------------------------------

    def get_upper_body_joint_positions(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each upper body JointType on this robot and its corresponding joint position
        """
        return {jt: p for jt, p in self.get_current_joint_positions().items() if jt.is_upper_body}

    def get_lower_body_joint_positions(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each lower body JointType on this robot and its corresponding joint position
        """
        return {jt: p for jt, p in self.get_current_joint_positions().items() if not jt.is_upper_body}

    def _get_current_arm_sdk_weight(self) -> float:
        """
        Returns:
            the current arm sdk weight - 0 if entirely high-level control, 1 if entirely low-level control, or any value
                in between if it's a blend
        """
        if self.device_type == RobotType.G1:
            return self._current_cmd.motor_cmd[G1_ARM_SDK_WEIGHT_MOTOR_IDX].q
        if self.device_type == RobotType.R1:
            return self._current_cmd.mode_pr / 100.
        raise UnknownRobotException(f"Do not know how to get arm sdk weight from {self.device_type}")


    # INITIALIZATION ------------------------------------------------------------------------------

    def initialize(self) -> None:
        """
        Initialize this robot
        """
        self._loco_client.Init()
        super().initialize()

    @override
    def activate_control(self) -> None:
        """
        Change the control mode for the upper half of the body

        Note that this function will block if switching from low to high level control
        """
        super().activate_control()
        with self._cmd_lock:
            self._reset_cmd()
            # can go straight to 100 if we're moving the motors to their current position
            self._fill_cmd(self.get_upper_body_joint_positions())
            self._send_command()

    def set_fsm_state(self, target_state: RobotFSMState) -> bool:
        """
        Set the robot's FSM state to the target state

        Args:
            target_state: what FSM State to transition the robot into

        Returns:
            True if the robot successfully transitioned states, False otherwise
        """
        code = self.loco_client.SetFsmId(target_state.value)
        return code == 0


    # SHUTDOWN ------------------------------------------------------------------------------------

    def e_stop(self) -> None:
        """
        Stop the robot FAST

        Absolutely ridiculous that this needs to be software but at least it'll exist
        """
        self.loco_client.SetFsmId(RobotFSMState.Damping.value)
        super().shutdown()

    def shutdown(self, enter_damping: bool = True) -> None:
        """
        Shut down the robot

        Args:
            enter_damping: if True, will move the robot to damping mode
        """
        super().shutdown()
        if self.is_active:
            self.deactivate_control()
        if enter_damping:
            self.loco_client.SetFsmId(RobotFSMState.Damping.value)

    @override
    def deactivate_control(self, duration: float = 2.) -> None:
        """
        Slowly ramps down the relevant value so the upper body will no longer listen to low level control

        Args:
            duration: how long it should take to release the arm control (secs)
        """
        super().deactivate_control()
        with self._cmd_lock:
            steps = max(1, int(duration / CONTROL_DT))
            for weight in np.linspace(self._get_current_arm_sdk_weight(), 0.0, num=steps + 1):
                self._set_arm_sdk_weight(weight)
                self._send_command()
            self._set_arm_sdk_weight(0.0)

    # UPDATE CYCLE --------------------------------------------------------------------------------

    @override
    def _update(self) -> None:
        fsm_id = self._loco_client.GetFsmId()[1]
        try:
            self._robot_fsm_state = RobotFSMState(fsm_id)
        except ValueError:
            self._robot_fsm_state = RobotFSMState.Unknown

        super()._update()

    # ARM COMMANDS --------------------------------------------------------------------------------

    @override
    def _verify_pos(self, pos: dict[JointType, float]) -> None:
        """
        Run all error handling on the current pos, verifying it is correct and throwing an error if it is not

        Args:
            pos: the joint positions to verify

        Raises:
            IllegalRobotStateException: if the robot is not in an FSM state that is able to take joint commands, or if
                low level joint commands have not been enabled
            IllegalJointCommandException: if a lower-body joint position is present in pose
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        super()._verify_pos(pos)
        if not self._robot_fsm_state.can_control_arms:
            raise IllegalRobotStateException(f"Cannot control the arms when robot is in {self._robot_fsm_state}")

    @override
    def _reset_cmd(self) -> None:
        """
        Resets the arm command, so it is ready for a new command to be filled in
        """
        self._set_arm_sdk_weight(1.)
        super()._reset_cmd()

    def _set_arm_sdk_weight(self, weight: float) -> None:
        """
        Set the weight of low-level control to the current command

        Args:
            weight: what percentage (0-1) the robot should listen to the low-level command. 1 is entirely low-level,
                0 is entirely high-level
        """
        if self.device_type == RobotType.G1:
            self._current_cmd.motor_cmd[G1_ARM_SDK_WEIGHT_MOTOR_IDX].q = weight
        elif self.device_type == RobotType.R1:
            self._current_cmd.mode_pr = int(np.clip(weight, 0.0, 1.0) * 100.0)
        else:
            raise UnknownRobotException(f"Do not know how to set arm sdk weight on {self.device_type}")

    @override
    def _send_command(self, block_for_control_dt: bool = True) -> None:
        """
        Send the stored arm command

        Assumed that the arm command is entirely set up except for the Crc

        Args:
            block_for_control_dt: if True, will sleep the current thread for CONTROL_DT secs after the command is sent
        """
        self._current_cmd.crc = self._crc.Crc(self._current_cmd)
        super()._send_command(block_for_control_dt)
