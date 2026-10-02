import math
import threading
import time
from enum import Enum, auto
from typing import Generator

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.utils.crc import CRC

from top_tier.general.repeated_event import RepeatedEvent
from top_tier.hardware.extensions import R1LocoClient, G1LocoClient
from top_tier.hardware.joint import Joint, JointType
from top_tier.general.constants import NETWORK_INTERFACE, CONTROL_DT, G1_ARM_SDK_WEIGHT_MOTOR_IDX
from top_tier.general.exceptions import IllegalRobotStateException, IllegalJointCommandException, UnknownRobotException
from top_tier.general.repeated_event import RepeatMode

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


class ControlMode(Enum):
    """
    Control of the arms is dictated by the Unitree remote
    """
    HighLevel = auto()

    """
    Takes in a set of waypoints, and the robot will interpolate between them.
    Allows for sparse control
    """
    Waypoints = auto()

    """
    Set a target that the robot will move to (subject to the max velocity).
    Allows for smoother control, but only if continuous data is provided
    """
    TrackTarget = auto()

    """
    Allows setting of waypoints, but any joints not set by the waypoint will follow the given target
    Still requires continuous data, but allows for presets
    """
    Mixed = auto()

    @property
    def is_low_level(self) -> bool:
        """
        Returns:
            Whether this control mode is low-level control
        """
        return self in (ControlMode.Waypoints, ControlMode.TrackTarget)


class Robot:
    """
    Base class for all types of robots
    """
    def __init__(self, robot_type: RobotType, is_channel_initialized: bool = False):
        """
        Create a new instance of a robot and do all relevant setup

        Args:
            robot_type: the type of robot this instance represents
            is_channel_initialized: whether ChannelFactoryInitialize has already been called
        """
        self._robot_type = robot_type

        if not is_channel_initialized:
            ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        self._loco_client = G1LocoClient() if robot_type == RobotType.G1 else R1LocoClient()
        self._loco_client.SetTimeout(1.0)

        self._joints = self._robot_type.get_joint_map()
        self._robot_fsm_state = RobotFSMState.ZeroTorque
        self._control_mode = ControlMode.HighLevel

        self._state_sub = ChannelSubscriber(
            "rt/lowstate",
            LowState_,
        )
        self._last_lowstate_call_time = -1.
        self._state_update_event = RepeatedEvent(CONTROL_DT, RepeatMode.START_TO_START, self._update_internal_state)

        self._arm_pub = ChannelPublisher(
            "rt/arm_sdk",
            LowCmd_,
        )
        self._arm_cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()
        self._arm_cmd_lock = threading.Lock()

        self._current_arm_motion_gen: Generator[None, None, None] | None = None
        self._waypoint_joints: set[JointType] = set()
        self._current_motion_num = 0
        self._max_vel = 50.

        self._target_pose: dict[JointType, float] = {}
        self._target_kff: dict[JointType, float] = {}


    # MISC PROPERTIES -----------------------------------------------------------------------------

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

    # JOINTS --------------------------------------------------------------------------------------

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

    def has_joint(self, joint_type: JointType) -> bool:
        """
        Determine if this robot has the given joint type

        Args:
            joint_type: the type of joint to check

        Returns:
            True if this robot has the given joint type, False otherwise
        """
        return joint_type in self.joints

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

    def get_current_joint_velocities(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each JointType on this robot and its corresponding joint position
        """
        joint_vels = {}
        for joint in self.joint_set:
            joint_vels[joint.joint_type] = joint.vel
        return joint_vels

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

    def set_control_mode(self, control_mode: ControlMode) -> None:
        """
        Change the control mode for the upper half of the body

        See ControlMode for details

        Note that this function will block if switching between high and low level control

        Args:
            control_mode: the mode to set
        """
        if self._control_mode == control_mode:
            return
        old_control_mode = self._control_mode
        self._control_mode = control_mode
        if not old_control_mode.is_low_level and control_mode.is_low_level:
            # enable low level control
            # can go straight to 100 if we're moving the motors to their current position
            self._fill_arm_cmd(self.get_upper_body_joint_positions())
            self._send_arm_command()
        elif old_control_mode.is_low_level and not control_mode.is_low_level:
            self.release_low_level_arm_control()

        self._current_arm_motion_gen = None
        self._waypoint_joints = set()
        self._target_pose = {}
        self._target_kff = {}

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
        if self._control_mode.is_low_level:
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
        self._control_mode = ControlMode.HighLevel
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

        self._update_arms()

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

    def _update_arms(self) -> None:
        """
        Update the position of the arms, depending on the current control mode
        """
        with self._arm_cmd_lock:
            if self._control_mode == ControlMode.Waypoints:
                self._update_arms_waypoints()
            elif self._control_mode == ControlMode.TrackTarget:
                self._update_arms_track_target()
            elif self._control_mode == ControlMode.Mixed:
                self._update_arms_mixed()

    def set_max_velocity(self, max_vel: float) -> None:
        """
        Set the max velocity for the TARGET TRACKING ONLY

        A nonpositive maximum velocity means no speed limit - joints will always move instantly to their target/waypoint

        Args:
            max_vel: the maximum angular velocity (rad/s) a joint can move at
        """
        if max_vel <= 0:
            max_vel = -1.  # just for consistency
        self._max_vel = max_vel

    def move_to_waypoints(self, waypoints: list[dict[JointType, float]] | dict[JointType, float], max_vel: float | None = None, block: bool = True, override: bool = True) -> bool:
        """
        Command the upper body to move through a set of waypoints

        Only works when the control mode is Waypoints or Mixed
        All given joints must be in the upper body, and all given positions must be in range for that joint.
        TODO this func does not set a limit on acceleration
        The speed of each joint will be determined so that they all arrive at the same time, and the fastest
        joint is moving at max_vel.
        Any joints not in the given dict will not be moved

        Args:
            waypoints: a list of waypoints for the robot to hit, where each waypoint is a mapping of upper body
                joints to their target positions (radians). Alternatively, a single waypoint represented like this
            max_vel: an override for the maximum velocity. if None, uses self.max_vel. Otherwise, sets the max velocity
                for this motion (rad/s)
            block: if True, this function will not exit until the movement is complete. Otherwise, the movement will run
                in the background
            override: what to do if there is a motion already running. If True, will stop it and start executing this
                motion instead. Otherwise, will discard this motion

        Returns:
            True if this motion successfully executed, False if it did not. False can occur if override is False and a
            motion is already running, or, if block, False can occur if this motion was overridden by something else. If
            not block, this function will assume that the motion completes, and returns True (regardless of if a future
            call overrides it).

        Raises:
            IllegalRobotStateException: if the robot is not in an FSM state that is able to take joint commands, or if
                the current control mode is not Waypoints
            IllegalRobotStateException: if a lower-body joint position is present in waypoints
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        if isinstance(waypoints, dict):
            waypoints = [waypoints]

        for wp in waypoints:
            self._verify_poses(wp, {ControlMode.Waypoints, ControlMode.Mixed})

        if self.is_arm_command_running and not override:
            return False

        with self._arm_cmd_lock:
            self._current_motion_num += 1
            this_motion_num = self._current_motion_num
            dense_waypoints = self._create_dense_waypoints(waypoints, self._max_vel if max_vel is None else max_vel)
            self._current_arm_motion_gen = self._set_arm_waypoint_gen(dense_waypoints, send_command=self._control_mode == ControlMode.Waypoints)

        if block:  # the motion will be executed by the update loop, just wait for it to be done
            while self.is_arm_command_running:
                time.sleep(CONTROL_DT)
                # dont think we need arm_cmd_lock here, since we're only checking the value (i.e. only need to reference
                # it in memory once) but if anything more complex happens here, should lock
                if self._current_motion_num != this_motion_num:  # has been overridden
                    return False
            with self._arm_cmd_lock:
                self._current_arm_motion_gen = None
        return True

    def _update_arms_waypoints(self) -> None:
        """
        Update the arm positions when we are in Waypoint mode
        """
        if not self.is_arm_command_running:
            return
        try:
            next(self._current_arm_motion_gen)
        except StopIteration:
            self._current_arm_motion_gen = None

    @property
    def is_arm_command_running(self) -> bool:
        """
        Returns:
            True if there is an arm command currently running, False otherwise
        """
        return self._current_arm_motion_gen is not None

    def _set_arm_waypoint_gen(self, waypoints: list[dict[JointType, float]], send_command: bool = True) -> Generator[None, None, None]:
        """
        Run through a set of waypoints, returning control of the current thread after each call

        Expects that this function will be called every CONTROL_DT seconds

        Args:
            waypoints: the list of waypoints to travel through
            send_command: whether to actually send the command
        """
        for waypoint in waypoints:
            self._waypoint_joints = set(waypoint.keys())
            self._fill_arm_cmd(waypoint)
            yield None
            if send_command:
                self._send_arm_command(False)

    def _create_dense_waypoints(self, sparse_waypoints: list[dict[JointType, float]], max_vel: float) -> list[dict[JointType, float]]:
        """
        Fills in a list of waypoints with a dense plan, so that when each waypoint is moved to at a rate of CONTROL_DT,
        the motions will be subject to self._max_vel

        Args:
            sparse_waypoints: a list of waypoints for the robot to hit, where each waypoint is a mapping of upper body
                joints to their target positions (radians)
            max_vel: the maximum angular velocity any joint may move at for this motion

        Returns:
            a list of waypoints for the robot to move to every CONTROL_DT secs, where each waypoint is represented as
                a mapping from joint type to its target position (rad) for that waypoint
        """
        if self._max_vel <= 0:
            return sparse_waypoints

        starting_pose = self.get_current_joint_positions()
        dense_waypoints = []
        for waypoint in sparse_waypoints:
            max_delta = max(
                abs(target_pos - starting_pose[joint_type])
                for joint_type, target_pos in waypoint.items()
            )
            duration = max_delta / max_vel
            num_steps = max(1, math.ceil(duration / CONTROL_DT))

            dense_waypoints.extend([
                {
                    joint_type: starting_pose[joint_type] + (target_pos - starting_pose[joint_type]) * (
                                step / num_steps)
                    for joint_type, target_pos in waypoint.items()
                }
                for step in range(1, num_steps + 1)
            ])
            starting_pose = waypoint

        return dense_waypoints

    def set_target_position(self, target_pose: dict[JointType, float | tuple[float, float]]) -> None:
        """
        Update the target position for TrackTarget control mode

        Each specified joint can also optionally include a feedforward value for that joint's PID controller.

        It is expected that this will be called many times a second, frequent enough that the motors do not have to
        stop and start over and over

        Args:
            target_pose: a mapping of joint types to their corresponding target position (rad). Each joint may instead
                map to a tuple of floats, in which case the first will be target position (rad) and the second will be
                the target PID feedforward value

        Raises:
            IllegalRobotStateException: if the robot is not in an FSM state that is able to take joint commands, or if
                the current control mode is not TrackTarget
            IllegalRobotStateException: if a lower-body joint position is present in pose
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        new_pose = {}
        new_kff = {}
        for joint_type, pos_and_kff in target_pose.items():
            if isinstance(pos_and_kff, tuple):
                pos, kff = pos_and_kff
            else:
                pos = pos_and_kff
                kff = 0.
            new_pose[joint_type] = pos
            new_kff[joint_type] = kff

        self._verify_poses(new_pose, {ControlMode.TrackTarget, ControlMode.Mixed})

        with self._arm_cmd_lock:
            self._target_pose = new_pose
            self._target_kff = new_kff

    def _clipped_pose(self) -> dict[JointType, float]:
        """
        Clip the target pose so that it obeys max_vel when called at an interval of CONTROL_DT

        Additionally, removes any joints from target pose that are currently being tracked by a waypoint.
        """
        current_pos = self.get_upper_body_joint_positions()
        target = current_pos if len(self._target_pose) == 0 else self._target_pose.copy()
        target = {j: t for j, t in target.items() if j not in self._waypoint_joints}
        if len(target) == 0:
            return {}
        delta_pos = {j: t - current_pos[j] for j, t in target.items()}
        max_dist = max(abs(d) for d in delta_pos.values())
        scale_factor = 0. if np.isclose(max_dist, 0.) else min(self._max_vel * CONTROL_DT / max_dist, 1.)
        return {j: current_pos[j] + d * scale_factor for j, d in delta_pos.items()}

    def _update_arms_track_target(self) -> None:
        """
        Update the arm positions when we are in TrackTarget mode
        """
        self._waypoint_joints = set()
        clipped_pos = self._clipped_pose()
        self._fill_arm_cmd(clipped_pos, self._target_kff)
        self._send_arm_command(False)

    def _update_arms_mixed(self) -> None:
        """
        Update the arm positions when we are in Mixed mode
        """
        if self.is_arm_command_running:
            try:
                next(self._current_arm_motion_gen)
            except StopIteration:
                self._waypoint_joints = set()
                self._current_arm_motion_gen = None

        clipped_pos = self._clipped_pose()
        self._fill_arm_cmd(clipped_pos, self._target_kff)
        self._send_arm_command(False)

    def _verify_poses(self, pose: dict[JointType, float], ideal_control_modes: set[ControlMode]) -> None:
        """
        Run all error handling on the current poses and control mode, verifying they are correct and throwing an error
        if they are not

        Args:
            pose: the joint positions to verify
            ideal_control_modes: the allowable control modes we can be in

        Raises:
            IllegalRobotStateException: if the robot is not in an FSM state that is able to take joint commands, or if
                the current control mode is not ideal_control_mode
            IllegalRobotStateException: if a lower-body joint position is present in pose
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        if self._robot_fsm_state not in (RobotFSMState.G1Walking, RobotFSMState.R1Walking, RobotFSMState.R1Balancing):
            raise IllegalRobotStateException(f"Cannot control the arms when robot is in {self._robot_fsm_state}")
        if self._control_mode not in ideal_control_modes:
            raise IllegalRobotStateException(f"Cannot do this operation while in {self._control_mode}")
        for joint_type, target_pos in pose.items():
            if not joint_type.is_upper_body:
                raise IllegalJointCommandException(f"Cannot set the position of {joint_type.name}")
            self.joints[joint_type].assert_pos_in_range(target_pos)

    def _reset_arm_cmd(self) -> None:
        """
        Resets the arm command, so it is ready for a new command to be filled in
        """
        self._set_arm_sdk_weight(1.)
        for joint_idx in range(len(self._arm_cmd.motor_cmd)):
            self._arm_cmd.motor_cmd[joint_idx].mode = 0  # by default ignore all joints - will override the joints we're using
            self._arm_cmd.motor_cmd[joint_idx].dq = 0.
            self._arm_cmd.motor_cmd[joint_idx].tau = 0.

    def _fill_arm_cmd(self, pose: dict[JointType, float], kffs: dict[JointType, float] | None = None) -> None:
        """
        Fill in the data for an arm command, matching the given pose and kff values

        Args:
            pose: a mapping of joint type to the position to set that joint, for each joint to set
            kffs: a mapping of joint type to its corresponding feedforward value. Will only apply if the joint is
                present in pose
        """
        self._reset_arm_cmd()
        if kffs is None:
            kffs = {}
        for joint_type, pos in pose.items():
            kff = kffs.get(joint_type, 0.)
            self._joints[joint_type].add_to_cmd(self._arm_cmd, pos, kff=kff)

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
