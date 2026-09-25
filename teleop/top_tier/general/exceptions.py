

class RobotException(Exception):
    """
    Generic base exception for errors relating to the robot
    """


class UnknownRobotException(RobotException):
    """
    Raised if a robot-specific action is taken with an unknown robot type
    """


class IllegalRobotStateException(RobotException):
    """
    For doing an operation when the robot is in a state that does not support it
    """


class IllegalJointCommandException(RobotException):
    """
    For if a command is attempted to be set that is not allowed
    (i.e. trying to set the position of a leg joint with an arm command)
    """


class JointOutOfBoundsException(RobotException):
    """
    Raised if a joint is told to move outside its range
    """
