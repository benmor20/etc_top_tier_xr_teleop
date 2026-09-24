from enum import IntEnum, auto


class JointType(IntEnum):
    # Head
    HeadPitch = auto()
    HeadYaw = auto()

    # Left arm
    LeftShoulderPitch = auto()
    LeftShoulderRoll = auto()
    LeftShoulderYaw = auto()
    LeftElbow = auto()
    LeftWristRoll = auto()
    LeftWristPitch = auto()
    LeftWristYaw = auto()

    # Right arm
    RightShoulderPitch = auto()
    RightShoulderRoll = auto()
    RightShoulderYaw = auto()
    RightElbow = auto()
    RightWristRoll = auto()
    RightWristPitch = auto()
    RightWristYaw = auto()

    # Waist
    WaistYaw = auto()
    WaistRoll = auto()
    WaistPitch = auto()

    # Left leg
    LeftHipPitch = auto()
    LeftHipRoll = auto()
    LeftHipYaw = auto()
    LeftKnee = auto()
    LeftAnklePitch = auto()
    LeftAnkleRoll = auto()

    # Right leg
    RightHipPitch = auto()
    RightHipRoll = auto()
    RightHipYaw = auto()
    RightKnee = auto()
    RightAnklePitch = auto()
    RightAnkleRoll = auto()


class Joint:
    """
    A class to represent different joints on the robot, ensuring safe use of them
    """

    def __init__(self, joint_type: JointType, joint_id: int, lower_limit: float, upper_limit: float, Kp: float, Kd: float):
        """
        Creates a new joint

        Args:
            joint_type: the type of joint this represents
            joint_id: the ID of the joint on the robot
            lower_limit: radians, the lowest angle the joint can turn to
            upper_limit: radians, the highest angle the joint can turn to
            Kp: the proportional coefficient of the PID control for the joint
            Kd: the derivative component of the PID control for the joint
        """
        self._joint_type = joint_type
        self._joint_id = joint_id
        self._lower_limit = lower_limit
        self._upper_limit = upper_limit
        self._Kp = Kp
        self._Kd = Kd
        self._Kff = 0.  # referred to as tau in the commands, really just the feedforward component of PID

        self._q = 0.
        self._dq = 0.

    @property
    def joint_type(self) -> JointType:
        """
        Returns:
            the type of joint this joint is
        """
        return self._joint_type

    @property
    def joint_id(self) -> int:
        """
        Returns:
            the joint id
        """
        return self._joint_id

    @property
    def lower_limit(self) -> float:
        """
        Returns:
            radians, the lowest angle this joint can turn to
        """
        return self._lower_limit

    @property
    def upper_limit(self) -> float:
        """
        Returns:
            radians, the highest angle this joint can turn to
        """
        return self._upper_limit

    @property
    def Kp(self) -> float:
        """
        Returns:
            the proportional coefficient for this joint's PID controller
        """
        return self._Kp

    @property
    def Kd(self) -> float:
        """
        Returns:
            the derivative coefficient for this joint's PID controller
        """
        return self._Kd
