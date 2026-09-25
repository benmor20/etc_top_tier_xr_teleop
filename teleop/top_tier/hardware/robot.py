import math
import time
from enum import Enum

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.utils.crc import CRC

from teleop.top_tier.general.repeated_event import RepeatedEvent
from teleop.top_tier.hardware.extensions import R1LocoClient, G1LocoClient
from teleop.top_tier.hardware.joint import Joint, JointType
from teleop.top_tier.general.constants import NETWORK_INTERFACE, CONTROL_DT, G1_ARM_SDK_WEIGHT_MOTOR_IDX
from top_tier.general.exceptions import IllegalRobotStateException, IllegalJointCommandException, \
    JointOutOfBoundsException, UnknownRobotException

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


class RobotFSMState(Enum):
    Unknown = -1
    ZeroTorque = 0
    Damping = 1
    LockedStand = 4
    G1Walking = 801
    R1Walking = 811
    R1Balancing = 816


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
        self._robot_fsm_state = RobotFSMState.ZeroTorque
        self._doing_low_level_arms = False

        self._state_sub = ChannelSubscriber(
            "rt/lowstate",
            LowState_,
        )
        self._last_lowstate_call_time = -1.
        self._arm_pub = ChannelPublisher(
            "rt/arm_sdk",
            LowCmd_,
        )
        self._arm_cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()

        self._state_update_event = RepeatedEvent(CONTROL_DT, self._update_internal_state)


    # PROPERTIES/GETTERS --------------------------------------------------------------------------

    @property
    def robot_type(self) -> RobotType:
        """
        Returns:
            the type of robot this object represents
        """
        return self._robot_type

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

    @property
    def joints(self) -> dict[JointType, Joint]:
        """
        Returns:
            a mapping of each JointType on this robot to the corresponding Joint
        """
        return self._joints

    @property
    def joint_set(self) -> set[Joint]:
        """
        Returns:
            the set of joints on this robot
        """
        return set(self.joints.values())

    def get_joint(self, joint_type: JointType) -> Joint:
        """
        Get the joint corresponding to the given joint type

        Args:
            joint_type: the type of joint to get

        Returns:
            the joint of the given joint type

        Raises:
            KeyError: if the given joint type is not present on this Robot
        """
        return self._joints[joint_type]

    def get_current_joint_positions(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each JointType on this robot and its corresponding joint position
        """
        joint_poses = {}
        for joint in self.joint_set:
            joint_poses[joint.joint_type] = joint.pos
        return joint_poses

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
        if self.robot_type == RobotType.G1:
            return self._arm_cmd.motor_cmd[G1_ARM_SDK_WEIGHT_MOTOR_IDX].q
        if self.robot_type == RobotType.R1:
            return self._arm_cmd.mode_pr
        raise UnknownRobotException(f"Do not know how to get arm sdk weight from {self.robot_type}")


    # INITIALIZATION ------------------------------------------------------------------------------

    def initialize(self) -> None:
        """
        Initialize this robot
        """
        self._loco_client.Init()
        self._state_sub.Init()
        self._arm_pub.Init()
        self._state_update_event.start()

    def enable_low_level_arm_control(self) -> None:
        """
        Sets the relevant value so the upper body will start listening to low level control
        """
        # can go straight to 100 if we're moving the motors to their current position
        self._doing_low_level_arms = True
        self.set_upper_body_position(self.get_upper_body_joint_positions())

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
        self._state_update_event.stop()

    def shutdown(self, enter_damping: bool = True) -> None:
        """
        Shut down the robot

        Args:
            enter_damping: if True, will move the robot to damping mode
        """
        if self._doing_low_level_arms:
            self.release_low_level_arm_control()
        if enter_damping:
            self.e_stop()
        else:
            self._state_update_event.stop()

    def release_low_level_arm_control(self, duration: float = 2.) -> None:
        """
        Slowly ramps down the relevant value so the upper body will no longer listen to low level control

        Args:
            duration: how long it should take to release the arm control (secs)
        """
        self._doing_low_level_arms = False
        steps = max(1, int(duration / CONTROL_DT))
        for weight in np.linspace(self._get_current_arm_sdk_weight(), 0.0, num=steps + 1):
            self._set_arm_sdk_weight(weight)
            self._send_arm_command()
        self._set_arm_sdk_weight(0.0)


    # UPDATE CYCLE --------------------------------------------------------------------------------

    def _update_internal_state(self) -> None:
        """
        Update all tracked internal states of this robot

        This function is called on a timer every CONTROL_DT seconds.
        To be thread-safe, nothing else should update these variables - only read them
        """
        state = self._get_low_level_state()
        self._arm_cmd.mode_machine = state.mode_machine
        self._update_joint_states(state)
        fsm_id = self._loco_client.GetFsmId()[1]

        try:
            self._robot_fsm_state = RobotFSMState(fsm_id)
        except ValueError:
            self._robot_fsm_state = RobotFSMState.Unknown

        if self._doing_low_level_arms:
            self.set_upper_body_position(self.get_upper_body_joint_positions())

    def _get_low_level_state(self) -> LowState_:
        """
        Returns:
            the current low-level robot state
        """
        state: LowState_ | None = self._state_sub.Read()
        while state is None:
            time.sleep(CONTROL_DT)
            state = self._state_sub.Read()

        self._last_lowstate_call_time = time.time()
        return state

    def _update_joint_states(self, state: LowState_) -> None:
        """
        Update the internal information for all joints

        Args:
            state: the result of the call to rt/lowstate
        """
        for joint in self._joints.values():
            joint.update_state(state)


    # ARM COMMANDS --------------------------------------------------------------------------------

    def _set_arm_sdk_weight(self, weight: float) -> None:
        """
        Set the weight of low-level control to the current command

        Args:
            weight: what percentage (0-1) the robot should listen to the low-level command. 1 is entirely low-level,
                0 is entirely high-level
        """
        if self.robot_type == RobotType.G1:
            self._arm_cmd.motor_cmd[G1_ARM_SDK_WEIGHT_MOTOR_IDX].q = weight
        elif self.robot_type == RobotType.R1:
            self._arm_cmd.mode_pr = int(np.clip(weight, 0.0, 1.0) * 100.0)
        else:
            raise UnknownRobotException(f"Do not know how to set arm sdk weight on {self.robot_type}")

    def _send_arm_command(self, block_for_control_dt: bool = True) -> None:
        """
        Send the stored arm command

        Assumed that the arm command is entirely set up except for the Crc

        Args:
            block_for_control_dt: if True, will sleep the current thread for CONTROL_DT secs after the command is sent
        """
        self._arm_cmd.crc = self._crc.Crc(self._arm_cmd)
        self._arm_pub.Write(self._arm_cmd)
        if block_for_control_dt:
            time.sleep(CONTROL_DT)

    def set_upper_body_position(self, target_poses: dict[JointType, float], max_vel: float = 3.) -> None:
        """
        Set the position of joints in the upper body, moving them there with up to max_vel speed.

        All given joints must be in the upper body, and all given positions must be in range for that joint.
        TODO this func does not set a limit on acceleration
        The speed of each joint will be determined so that they all arrive at the same time, and the fastest
        joint is moving at max_vel.
        This function blocks until the movement is complete - TODO: dont?
        Any joints not in the given dict will not be moved

        Args:
            target_poses: a mapping of upper body joints to their target positions (radians)
            max_vel: rad/s, the maximum angular velocity the fastest joint may move

        Raises:
            IllegalRobotStateException: if the robot is not in a state that is able to take joint commands, or if
                enable_low_level_arm_control has not been called
            IllegalRobotStateException: if a lower-body joint position is present in target_poses
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        if self._robot_fsm_state not in (RobotFSMState.G1Walking, RobotFSMState.R1Walking, RobotFSMState.R1Balancing):
            raise IllegalRobotStateException(f"Cannot control the arms when robot is in {self._robot_fsm_state}")
        if not self._doing_low_level_arms:
            raise IllegalRobotStateException(f"Please call enable_low_level_arm_control before controlling the arms")
        for joint_type, target_pos in target_poses.items():
            if not joint_type.is_upper_body:
                raise IllegalJointCommandException(f"Cannot set the position of {joint_type.name}")
            self.joints[joint_type].assert_pos_in_range(target_pos)

        self._set_arm_sdk_weight(1.)
        for joint_idx in range(len(self._arm_cmd.motor_cmd)):
            self._arm_cmd.motor_cmd[joint_idx].mode = 0  # by default ignore all joints - will override the joints we're using
            self._arm_cmd.motor_cmd[joint_idx].dq = 0.
            self._arm_cmd.motor_cmd[joint_idx].tau = 0.

        waypoints = self._create_waypoints_with_max_vel(target_poses, max_vel)
        for waypoint in waypoints:
            for joint_type, pos in waypoint.items():
                self._joints[joint_type].add_to_cmd(self._arm_cmd, pos)
            self._send_arm_command()

    def _create_waypoints_with_max_vel(self, target_poses: dict[JointType, float], max_vel: float) -> list[dict[JointType, float]]:
        """
        Create a list of joint waypoints moving from the current position to the given target_poses, subject to the
        given max_vel

        Args:
            target_poses: a mapping of upper body joints to their target positions (radians)
            max_vel: rad/s, the maximum angular velocity the fastest joint may move

        Returns:
            a list of waypoints for the robot to move to every CONTROL_DT secs, where each waypoint is represented as
                a mapping from joint type to its target position (rad) for that waypoint
        """
        if max_vel <= 0:
            raise ValueError("max_vel must be positive")

        starting_poses = self.get_current_joint_positions()
        max_delta = max(
            abs(target_pos - starting_poses[joint_type])
            for joint_type, target_pos in target_poses.items()
        )
        duration = max_delta / max_vel
        num_steps = max(1, math.ceil(duration / CONTROL_DT))

        return [
            {
                joint_type: starting_poses[joint_type] + (target_pos - starting_poses[joint_type]) * (step / num_steps)
                for joint_type, target_pos in target_poses.items()
            }
            for step in range(1, num_steps + 1)
        ]

