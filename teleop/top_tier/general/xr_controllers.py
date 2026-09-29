from dataclasses import dataclass, field
from enum import auto, Enum

import numpy as np

from televuer import TeleData
from top_tier.general.constants import XR_CONTROLLER_TRIGGER_THRESHOLD


class XRControllerButton(Enum):
    A = auto()
    B = auto()
    X = auto()
    Y = auto()
    LeftTrigger = auto()
    LeftSqueeze = auto()
    LeftJoystick = auto()
    RightTrigger = auto()
    RightSqueeze = auto()
    RightJoystick = auto()


class XRControllerFloat(Enum):
    LeftTrigger = auto()
    LeftSqueeze = auto()
    LeftJoystickX = auto()
    LeftJoystickY = auto()
    RightTrigger = auto()
    RightSqueeze = auto()
    RightJoystickX = auto()
    RightJoystickY = auto()


class XRControllerMatrix(Enum):
    HeadPose = auto()
    LeftHandPose = auto()
    RightHandPose = auto()


XRControllerValue = XRControllerButton | XRControllerFloat | XRControllerMatrix


@dataclass
class XRControllerState:
    """
    State representation of the two XR Controllers
    """
    is_valid: bool = False  # true if this data comes from actual XR data, false if constructed
    
    head_pose: np.ndarray = field(default_factory=lambda: np.eye(4))  # (4,4) SE(3) pose of head matrix
    left_hand_pose: np.ndarray = field(default_factory=lambda: np.eye(4))  # (4,4) SE(3) pose of left wrist of arm
    right_hand_pose: np.ndarray = field(default_factory=lambda: np.eye(4))  # (4,4) SE(3) pose of right wrist of arm

    left_trigger: float = 0.0  # float trigger pull depth, 0.0 means no press
    left_squeeze: float = 0.0  # (0.0 → 1.0) grip pull depth, 0.0 means no press
    left_joystick_pressed: bool = False  # True if joystick button is pressed
    left_joystick_pos: np.ndarray = field(default_factory=lambda: np.zeros(2))  # 2D vector (x, y), normalized

    right_trigger: float = 0.0  # float trigger pull depth, 0.0 means no press
    right_squeeze: float = 0.0  # (0.0 → 1.0) grip pull depth, 0.0 means no press
    right_joystick_pressed: bool = False  # True if joystick button is pressed
    right_joystick_pos: np.ndarray = field(default_factory=lambda: np.zeros(2))  # 2D vector (x, y), normalized

    a_button: bool = False  # True if A button is pressed
    b_button: bool = False  # True if B button is pressed
    x_button: bool = False  # True if X button is pressed
    y_button: bool = False  # True if Y button is pressed

    @property
    def left_trigger_pressed(self) -> bool:
        """
        Returns:
            True if the left trigger is sufficiently pressed
        """
        return self.left_trigger > XR_CONTROLLER_TRIGGER_THRESHOLD

    @property
    def left_squeeze_pressed(self) -> bool:
        """
        Returns:
            True if the left squeeze is sufficiently pressed
        """
        return self.left_squeeze > XR_CONTROLLER_TRIGGER_THRESHOLD

    @property
    def right_trigger_pressed(self) -> bool:
        """
        Returns:
            True if the right trigger is sufficiently pressed
        """
        return self.right_trigger > XR_CONTROLLER_TRIGGER_THRESHOLD

    @property
    def right_squeeze_pressed(self) -> bool:
        """
        Returns:
            True if the right squeeze is sufficiently pressed
        """
        return self.right_squeeze > XR_CONTROLLER_TRIGGER_THRESHOLD

    @staticmethod
    def from_tele_data(td: TeleData) -> 'XRControllerState':
        """
        Creates an XRControllerState from the given TeleData

        Args:
            td: the TeleData to convert to XRControllerState

        Returns:
            an XRControllerState with the same data as td
        """
        return XRControllerState(
            is_valid=td.motion_data_ready,
            head_pose=td.head_pose,
            left_hand_pose=td.left_wrist_pose,
            right_hand_pose=td.right_wrist_pose,
            left_trigger=(10. - td.left_ctrl_triggerValue) / 10.,
            left_squeeze=td.left_ctrl_squeezeValue,
            left_joystick_pressed=td.left_ctrl_thumbstick == 1,
            left_joystick_pos=td.left_ctrl_thumbstickValue * np.array([1., -1.]),
            right_trigger=(10. - td.right_ctrl_triggerValue) / 10.,
            right_squeeze=td.right_ctrl_squeezeValue,
            right_joystick_pressed=td.right_ctrl_thumbstick == 1,
            right_joystick_pos=td.right_ctrl_thumbstickValue * np.array([1., -1.]),
            a_button=td.right_ctrl_aButton == 1,
            b_button=td.right_ctrl_bButton == 1,
            x_button=td.left_ctrl_aButton == 1,
            y_button=td.left_ctrl_bButton == 1,
        )
    
    def get_button(self, button: XRControllerButton) -> bool:
        """
        Get the value of the given button
        
        Args:
            button: the button to get the state of
        
        Returns:
            True if the button is pressed, False otherwise
        """
        if button == XRControllerButton.A:
            return self.a_button
        if button == XRControllerButton.B:
            return self.b_button
        if button == XRControllerButton.X:
            return self.x_button
        if button == XRControllerButton.Y:
            return self.y_button
        if button == XRControllerButton.LeftTrigger:
            return self.left_trigger_pressed
        if button == XRControllerButton.LeftSqueeze:
            return self.left_squeeze_pressed
        if button == XRControllerButton.LeftJoystick:
            return self.left_joystick_pressed
        if button == XRControllerButton.RightTrigger:
            return self.right_trigger_pressed
        if button == XRControllerButton.RightSqueeze:
            return self.right_squeeze_pressed
        if button == XRControllerButton.RightJoystick:
            return self.right_joystick_pressed
        raise ValueError(f"Unknown XRControllerButton: {button}")

    def get_float(self, trigger: XRControllerFloat) -> float:
        """
        Get the value of the given trigger on the controller

        Args:
            trigger: the trigger to get the value of

        Returns:
            the value of the given trigger
        """
        if trigger == XRControllerFloat.LeftTrigger:
            return self.left_trigger
        if trigger == XRControllerFloat.LeftSqueeze:
            return self.left_squeeze
        if trigger == XRControllerFloat.LeftJoystickX:
            return self.left_joystick_pos[0]
        if trigger == XRControllerFloat.LeftJoystickY:
            return self.left_joystick_pos[1]
        if trigger == XRControllerFloat.RightTrigger:
            return self.right_trigger
        if trigger == XRControllerFloat.RightSqueeze:
            return self.right_squeeze
        if trigger == XRControllerFloat.RightJoystickX:
            return self.right_joystick_pos[0]
        if trigger == XRControllerFloat.LeftTrigger:
            return self.right_joystick_pos[1]
        raise ValueError(f"Unknown XRControllerFloat: {trigger}")

    def get_matrix(self, matrix: XRControllerMatrix) -> np.ndarray:
        """
        Get the value of a given matrix tracked by this state

        Args:
            matrix: the matrix to get the value of

        Returns:
            the value of the given matrix
        """
        if matrix == XRControllerMatrix.HeadPose:
            return self.head_pose
        if matrix == XRControllerMatrix.LeftHandPose:
            return self.left_hand_pose
        if matrix == XRControllerMatrix.RightHandPose:
            return self.right_hand_pose
        raise ValueError(f"Unknown XRControllerMatrix: {matrix}")


class XRControllers:
    """
    Tracks the state of the XR Controllers, giving useful helper functions
    """
    def __init__(self):
        """
        Create a new XRControllers wrapper
        """
        self._last_state = XRControllerState()
        self._current_state = XRControllerState()

    def get_button(self, button: XRControllerButton) -> bool:
        """
        Get the value of the given button

        Args:
            button: the button to get the state of

        Returns:
            True if the button is pressed, False otherwise
        """
        return self._current_state.get_button(button)

    def was_button_just_pressed(self, button: XRControllerButton) -> bool:
        """
        Determine if the button was just pressed

        Args:
            button: the button to get the state of

        Returns:
            True if the button has just been pressed, False otherwise
        """
        return self.get_button(button) and not self._last_state.get_button(button)

    def was_button_just_released(self, button: XRControllerButton) -> bool:
        """
        Determine if the button was just released

        Args:
            button: the button to get the state of

        Returns:
            True if the button has just been released, False otherwise
        """
        return not self.get_button(button) and self._last_state.get_button(button)

    def get_float(self, trigger: XRControllerFloat) -> float:
        """
        Get the value of the given trigger on the controller

        Args:
            trigger: the trigger to get the value of

        Returns:
            the value of the given trigger
        """
        return self._current_state.get_float(trigger)

    def get_matrix(self, matrix: XRControllerMatrix) -> np.ndarray:
        """
        Get the value of a given matrix tracked by this state

        Args:
            matrix: the matrix to get the value of

        Returns:
            the value of the given matrix
        """
        return self._current_state.get_matrix(matrix)

    def update(self, tele_data: TeleData) -> None:
        """
        Update the state of this controller with new teledata

        Args:
            tele_data: the data to update this controller with
        """
        # print("Updating!")
        self._last_state = self._current_state
        self._current_state = XRControllerState.from_tele_data(tele_data)
        # if np.allclose(self._current_state.get_matrix(XRControllerMatrix.HeadPose), self._last_state.get_matrix(XRControllerMatrix.HeadPose)):
        #     print(f"Similar head poses!, pose is: {self.get_matrix(XRControllerMatrix.HeadPose)}")
