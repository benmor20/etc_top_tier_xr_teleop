import copy
import math
import threading
import time
from abc import abstractmethod, ABC
from typing import Callable, Generator, TypeVar, Generic

import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from top_tier.general.constants import CONTROL_DT
from top_tier.general.exceptions import IllegalJointCommandException, IllegalRobotStateException
from top_tier.general.repeated_event import RepeatedEvent, RepeatMode
from top_tier.hardware.joint import JointType, Joint
from top_tier.hardware.msg_types import DeviceState, DeviceCmd


DEVICE_T = TypeVar("DEVICE_T")
STATE_T = TypeVar("STATE_T", bound=DeviceState)
CMD_T = TypeVar("CMD_T", bound=DeviceCmd)


class HardwareDevice(ABC, Generic[DEVICE_T, STATE_T, CMD_T]):
    def __init__(self, device_type: DEVICE_T, state_name: str, cmd_name: str):
        self._device_type = device_type

        self._state_sub = ChannelSubscriber(state_name, self._state_type)
        self._current_state = self._state_default_factory()
        self._state_lock = threading.Lock()

        self._cmd_pub = ChannelPublisher(cmd_name, self._cmd_type)
        self._current_cmd = self._cmd_default_factory()
        self._cmd_lock = threading.Lock()

        self._waypoint_max_vel = 3.
        self._current_motion_gen: Generator[None, None, None] | None = None
        self._waypoint_joints: set[JointType] = set()
        self._current_motion_num = 0
        self._waypoint_callback: Callable[[bool], None] | None = None

        self._tracking_max_vel = 50.
        self._target_pose: dict[JointType, float] = {}
        self._position_to_hold: dict[JointType, float] = {}
        self._target_kff: dict[JointType, float] = {}

        self._update_event = RepeatedEvent(CONTROL_DT, RepeatMode.START_TO_START, self._update)
        self._active_control = False
        self._joints = self._get_joint_map(device_type)

    # ABSTRACT ------------------------------------------------------------------------------------

    @abstractmethod
    def _get_joint_map(self, device_type: DEVICE_T) -> dict[JointType, Joint]:
        """
        Construct a mapping of joint types to the joints they represent for this device

        Args:
            device_type: the type of device this is

        Returns:
            a dict mapping every joint type on this device to its corresponding joint
        """
        pass

    @property
    @abstractmethod
    def _state_type(self) -> type[STATE_T]:
        """
        Returns:
            the state class to pass to the ChannelSubscriber
        """
        pass

    @property
    @abstractmethod
    def _cmd_type(self) -> type[CMD_T]:
        """
        Returns:
            the command class to pass to the ChannelPublisher
        """
        pass

    @property
    @abstractmethod
    def _state_default_factory(self) -> Callable[[], STATE_T]:
        """
        Returns:
            a factory function that creates a new state
        """
        pass

    @property
    @abstractmethod
    def _cmd_default_factory(self) -> Callable[[], CMD_T]:
        """
        Returns:
            a factory function that creates a new command
        """
        pass

    # PROPERTIES ----------------------------------------------------------------------------------

    @property
    def device_type(self) -> DEVICE_T:
        """
        Returns:
            the type of device this is
        """
        return self._device_type

    @property
    def is_waypoint_command_running(self) -> bool:
        """
        Returns:
            True if there is an waypoint command currently running, False otherwise
        """
        return self._current_motion_gen is not None

    @property
    def is_active(self) -> bool:
        """
        Returns:
            Whether this device is actively being controlled
        """
        return self._active_control

    # JOINTS --------------------------------------------------------------------------------------

    @property
    def joints(self) -> dict[JointType, Joint]:
        """
        Returns:
            a mapping of each JointType on this device to the corresponding Joint
        """
        with self._state_lock:
            joints = copy.deepcopy(self._joints)
        return joints

    def has_joint(self, joint_type: JointType) -> bool:
        """
        Determine if this device has the given joint type

        Args:
            joint_type: the type of joint to check

        Returns:
            True if this device has the given joint type, False otherwise
        """
        return joint_type in self._joints

    def get_joint(self, joint_type: JointType) -> Joint:
        """
        Get the joint corresponding to the given joint type

        Args:
            joint_type: the type of joint to get

        Returns:
            the joint of the given joint type

        Raises:
            KeyError: if the given joint type is not present on this device
        """
        with self._state_lock:
            joint = copy.copy(self._joints[joint_type])
        return joint

    def get_current_joint_positions(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each JointType on this device and its corresponding joint position
        """
        joint_poses = {}
        with self._state_lock:
            for joint in self._joints.values():
                joint_poses[joint.joint_type] = joint.pos
        return joint_poses

    def is_joint_controlled(self, joint_type: JointType) -> bool:
        """
        Determines if a joint is allowed to be controlled by this device

        By default, assumes every joint this device has is controllable. Subclasses should override if needed

        Args:
            joint_type: the joint to check

        Returns:
            a bool, whether the joint can be controlled by this device
        """
        return self.has_joint(joint_type)

    def get_controlled_joint_positions(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each JointType on this device which can be controlled and its corresponding joint position
        """
        return {jt: p for jt, p in self.get_current_joint_positions().items() if self.is_joint_controlled(jt)}

    def get_current_joint_velocities(self) -> dict[JointType, float]:
        """
        Returns:
            a mapping of each JointType on this device and its corresponding joint position
        """
        joint_vels = {}
        with self._state_lock:
            for joint in self._joints.values():
                joint_vels[joint.joint_type] = joint.vel
        return joint_vels

    # LOGISTICS -----------------------------------------------------------------------------------

    def initialize(self) -> None:
        """
        Initialize this device
        """
        self._state_sub.Init()
        self._cmd_pub.Init()
        self._update_event.start()

    def activate_control(self) -> None:
        """
        Activates this device, marking it as ready for control
        """
        self._active_control = True
        try:
            self._position_to_hold = self.get_controlled_joint_positions()
            self.set_target_position({})
        except:
            self._active_control = False
            raise

    def shutdown(self) -> None:
        """
        Shut down this hardware device

        Should be called if the program is exited for any reason
        """
        self._update_event.stop()

    def deactivate_control(self) -> None:
        """
        Deactivate this device, stopping all active control
        """
        with self._cmd_lock:
            self._active_control = False
            self._current_motion_gen = None
            self._waypoint_callback = None
            self._waypoint_cancelled_callback = None
            self._waypoint_joints = set()
            self._target_pose = {}
            self._target_kff = {}
            self._position_to_hold = {}

    def _update(self) -> None:
        """
        Poll the current state

        Make sure any subclasses that override this function make the super() call LAST. _create_and_send_command may
        block for a long time (such as during release - the lock is held for a while
        """
        state = self._state_sub.Read(CONTROL_DT)
        if state is not None:
            with self._state_lock:
                self._current_state = state
                for joint in self._joints.values():
                    joint.update_state(state)

        self._create_and_send_command()  # will check is_active

    # CONTROL -------------------------------------------------------------------------------------

    def set_max_waypoint_velocity(self, max_vel: float) -> None:
        """
        Set the maximum velocity for waypoint motions

        Args:
            max_vel: the new max velocity for waypoint movements (rad/s)
        """
        self._waypoint_max_vel = max_vel

    def set_max_tracking_velocity(self, max_vel: float) -> None:
        """
        Set the maximum velocity for target tracking motions

        Args:
            max_vel: the new max velocity for target tracking movements (rad/s)
        """
        self._tracking_max_vel = max_vel

    def move_to_waypoints(self, waypoints: list[dict[JointType, float]] | dict[JointType, float], block: bool = True, override: bool = True, callback: Callable[[bool], None] | None = None) -> bool:
        """
        Command the upper body to move through a set of waypoints

        All given joints must be controllable, and all given positions must be in range for that joint.
        TODO this func does not set a limit on acceleration
        The speed of each joint will be determined so that they all arrive at the same time, and the fastest
        joint is moving at max_vel.
        Any joints not in the given dict will not be moved

        Args:
            waypoints: a list of waypoints for the device to hit, where each waypoint is a mapping of upper body
                joints to their target positions (radians). Alternatively, a single waypoint represented like this
            block: if True, this function will not exit until the movement is complete. Otherwise, the movement will run
                in the background
            override: what to do if there is a motion already running. If True, will stop it and start executing this
                motion instead. Otherwise, will discard this motion
            callback: a function to call when this series of waypoints finishes. Will be passed a bool (True if these
                waypoints successfully finished). Note that this function is called with cmd_lock active

        Returns:
            True if this motion successfully executed, False if it did not. False can occur if override is False and a
            motion is already running, or, if block, False can occur if this motion was overridden by something else. If
            not block, this function will assume that the motion completes, and returns True (regardless of if a future
            call overrides it).

        Raises:
            IllegalJointCommandException: if a lower-body joint position is present in waypoints
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        if isinstance(waypoints, dict):
            waypoints = [waypoints]

        for wp in waypoints:
            self._verify_pos(wp)

        if self.is_waypoint_command_running and not override:
            return False

        with self._cmd_lock:
            self._current_motion_num += 1
            this_motion_num = self._current_motion_num
            dense_waypoints = self._create_dense_waypoints(waypoints)
            if self._current_motion_gen is not None:
                self._on_waypoint_done(False)
            self._current_motion_gen = self._set_waypoint_gen(dense_waypoints)
            self._waypoint_callback = callback

        if block:  # the motion will be executed by the update loop, just wait for it to be done
            while self.is_waypoint_command_running:
                time.sleep(CONTROL_DT)
                # dont think we need cmd_lock here, since we're only checking the value (i.e. only need to reference
                # it in memory once) but if anything more complex happens here, should lock
                if self._current_motion_num != this_motion_num:  # has been overridden
                    return False
        return True

    def set_target_position(self, target_pose: dict[JointType, float | None | tuple[float, float]]) -> None:
        """
        Update the target position target tracking

        Each specified joint can also optionally include a feedforward value for that joint's PID controller.

        It is expected that this will be called many times a second, frequent enough that the motors do not have to
        stop and start over and over

        Any joint type not passed (and not being controlled by a waypoint) will hold its current position. Any joint
        type which maps to None will fall loose

        Args:
            target_pose: a mapping of joint types to their corresponding target position (rad). Each joint may instead
                map to a tuple of floats, in which case the first will be target position (rad) and the second will be
                the target PID feedforward value

        Raises:
            IllegalJointCommandException: if a lower-body joint position is present in pose
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

        self._verify_pos(new_pose)

        with self._cmd_lock:
            self._target_pose = new_pose
            self._target_kff = new_kff

    def _verify_pos(self, pos: dict[JointType, float | None]):
        """
        Run all error handling on the current poses and control mode, verifying they are correct and throwing an error
        if they are not

        Args:
            pos: the joint positions to verify

        Raises:
            IllegalRobotStateException: if this device has not been activated
            IllegalJointCommandException: if a joint has been set which is not present on this device
            JointOutOfBoundsException: if any pos is out of range for its corresponding joint
        """
        if not self.is_active:
            raise IllegalRobotStateException("Cannot control a device which has not been activated")
        for joint_type, target_pos in pos.items():
            if not self.is_joint_controlled(joint_type):
                raise IllegalJointCommandException(f"Cannot set the position of {joint_type.name} on this {self.__class__.__name__}")
            if target_pos is not None:
                self._joints[joint_type].assert_pos_in_range(target_pos)

    def _set_waypoint_gen(self, waypoints: list[dict[JointType, float]]) -> Generator[None, None, None]:
        """
        Run through a set of waypoints, returning control of the current thread after each call

        Expects that this function will be called every CONTROL_DT seconds
        Assumes that this is called with the _cmd_lock locked

        Args:
            waypoints: the list of waypoints to travel through
        """
        prev_joints: set[JointType] = set()
        for waypoint in waypoints:
            self._waypoint_joints = set(waypoint.keys())
            joints_to_hold = prev_joints - self._waypoint_joints
            current_pos = self.get_current_joint_positions()
            self._position_to_hold.update({j: current_pos[j] for j in joints_to_hold})

            self._fill_cmd(waypoint)
            prev_joints = copy.copy(self._waypoint_joints)
            yield None
        self._on_waypoint_done(True)

    def _on_waypoint_done(self, successful_finish: bool) -> None:
        """
        Do relevant commands when a series of waypoints has finished running

        Calls the waypoint callback and resets the position to hold

        Args:
            successful_finish: whether the waypoint sequence finished, or was cancelled early
        """
        self._position_to_hold = self.get_controlled_joint_positions()
        if self._waypoint_callback is not None:
            self._waypoint_callback(successful_finish)

    def _create_dense_waypoints(self, sparse_waypoints: list[dict[JointType, float]]) -> list[dict[JointType, float]]:
        """
        Fills in a list of waypoints with a dense plan, so that when each waypoint is moved to at a rate of CONTROL_DT,
        the motions will be subject to self._waypoint_max_vel

        Args:
            sparse_waypoints: a list of waypoints for the device to hit, where each waypoint is a mapping of upper body
                joints to their target positions (radians)

        Returns:
            a list of waypoints for the device to move to every CONTROL_DT secs, where each waypoint is represented as
                a mapping from joint type to its target position (rad) for that waypoint
        """
        if self._waypoint_max_vel <= 0:
            return sparse_waypoints

        starting_pose = self.get_current_joint_positions()
        dense_waypoints = []
        for waypoint in sparse_waypoints:
            max_delta = max(
                abs(target_pos - starting_pose[joint_type])
                for joint_type, target_pos in waypoint.items()
            )
            duration = max_delta / self._waypoint_max_vel
            num_steps = max(1, math.ceil(duration / CONTROL_DT))

            dense_waypoints.extend([
                {
                    joint_type: starting_pose[joint_type] + (target_pos - starting_pose[joint_type]) * (step / num_steps)
                    for joint_type, target_pos in waypoint.items()
                }
                for step in range(1, num_steps + 1)
            ])
            starting_pose.update(waypoint)

        return dense_waypoints

    def _clipped_pose(self) -> dict[JointType, float | None]:
        """
        Clip the target pose so that it obeys target_max_vel when called at an interval of CONTROL_DT

        The returned pose will fill in any missing positions with a static position, and will remove any poses that
        are being controlled by waypoint. Any remaining joint types which map to None will still map to None

        Returns:
            A pose between the current one and the target one, which obeys _tracking_max_vel
        """
        current_pos = self.get_controlled_joint_positions()
        target = {**self._position_to_hold, **self._target_pose}
        target = {j: t for j, t in target.items() if j not in self._waypoint_joints}
        if len(target) == 0 or all(pos is None for pos in target.values()):
            return target
        delta_pos = {j: (None if t is None else t - current_pos[j]) for j, t in target.items()}
        max_dist = max(abs(d) for d in delta_pos.values() if d is not None)
        scale_factor = 0. if np.isclose(max_dist, 0.) else min(self._tracking_max_vel * CONTROL_DT / max_dist, 1.)
        return {j: (None if d is None else current_pos[j] + d * scale_factor) for j, d in delta_pos.items()}

    def _reset_cmd(self) -> None:
        """
        Resets the command, so it is ready for a new command to be filled in
        """
        for joint_idx in range(len(self._current_cmd.motor_cmd)):
            self._current_cmd.motor_cmd[joint_idx].mode = 0  # by default ignore all joints - will override the joints we're using
            self._current_cmd.motor_cmd[joint_idx].q = 0.
            self._current_cmd.motor_cmd[joint_idx].dq = 0.
            self._current_cmd.motor_cmd[joint_idx].tau = 0.

    def _fill_cmd(self, pose: dict[JointType, float | None], kffs: dict[JointType, float] | None = None) -> None:
        """
        Fill in the data for a command, matching the given pose and kff values

        Args:
            pose: a mapping of joint type to the position to set that joint, for each joint to set
            kffs: a mapping of joint type to its corresponding feedforward value. Will only apply if the joint is
                present in pose
        """
        if kffs is None:
            kffs = {}
        for joint_type, pos in pose.items():
            kff = kffs.get(joint_type, 0.)
            if pos is None:
                self._joints[joint_type].add_to_cmd_slack(self._current_cmd, kff=kff)
            else:
                self._joints[joint_type].add_to_cmd(self._current_cmd, pos, kff=kff)

    def _send_command(self, block_for_control_dt: bool = True) -> None:
        """
        Send the stored command

        Assumes that this function is called with the _cmd_lock locked

        Args:
            block_for_control_dt: if True, will sleep the current thread for CONTROL_DT secs after the command is sent
        """
        self._cmd_pub.Write(self._current_cmd)
        if block_for_control_dt:
            time.sleep(CONTROL_DT)

    def _create_and_send_command(self) -> None:
        """
        Update the current joint command and send it to this device
        """
        with self._cmd_lock:
            if not self._active_control:
                return
            self._reset_cmd()
            if self.is_waypoint_command_running:
                try:
                    next(self._current_motion_gen)
                except StopIteration:
                    self._waypoint_joints = set()
                    self._current_motion_gen = None
                    self._waypoint_callback = None

            clipped_pos = self._clipped_pose()
            self._fill_cmd(clipped_pos, self._target_kff)
            self._send_command(False)
