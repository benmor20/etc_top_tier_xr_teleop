from enum import Enum, auto

from hardware.joint import JointType, Joint


def _construct_dex3_joints(left_hand: bool = True) -> dict[JointType, Joint]:
    """
    Construct a dictionary of dex3 joints for the given hand

    Args:
        left_hand: if True, constructs a dict for the left hand. Otherwise, right hand

    Returns:
        a dict mapping joint types of the requested hand to the joints they represent
    """
    joint_order = (
        [
            JointType.LeftThumbYaw,
            JointType.LeftThumbJoint1,
            JointType.LeftThumbJoint2,
            JointType.LeftIndexFingerJoint1,
            JointType.LeftIndexFingerJoint2,
            JointType.LeftMiddleFingerJoint1,
            JointType.LeftMiddleFingerJoint2,
        ] if left_hand else [
            JointType.RightThumbYaw,
            JointType.RightThumbJoint1,
            JointType.RightThumbJoint2,
            JointType.RightIndexFingerJoint1,
            JointType.RightIndexFingerJoint2,
            JointType.RightMiddleFingerJoint1,
            JointType.RightMiddleFingerJoint2,
        ])
    return {
        joint_order[0]: Joint(joint_order[0], 0, -1.047, 1.047, 1.5, 0.2),
        joint_order[1]: Joint(joint_order[1], 1, -1.047, 0.6108, 1.5, 0.2),
        joint_order[2]: Joint(joint_order[2], 2, 0., 1.745, 1.5, 0.2),
        joint_order[3]: Joint(joint_order[3], 3, -1.5707, 0., 1.5, 0.2),
        joint_order[4]: Joint(joint_order[4], 4, -1.745, 0., 1.5, 0.2),
        joint_order[5]: Joint(joint_order[5], 5, -1.5707, 0., 1.5, 0.2),
        joint_order[6]: Joint(joint_order[6], 5, -1.745, 0., 1.5, 0.2),
    }


# Currently only supports Dex3, but allows for us to expand to different hands easily
class HandType(Enum):
    Dex3 = auto()

    def get_joint_map(self, left_hand: bool) -> dict[JointType, Joint]:
        """
        Get the joint map corresponding to this hand type

        Args:
            left_hand: whether to construct the joint map for the left or right hand

        Returns:
            a dict mapping joint types of the requested hand to the joints they represent
        """
        return _construct_dex3_joints(left_hand)


class Hand:
    """
    Base class for all types of hands

    Enables tracking and control of various hand types
    """
    def __init__(self, hand_type: HandType, left_hand: bool):
        """
        Create a new hand and do all relevant setup

        Assumes ChannelFactoryInitialize has already been called

        Args:
            hand_type: the type of hand to create
            left_hand: whether this hand is on the robot's left or right
        """
        self._hand_type = hand_type
        self._is_left_hand = left_hand

        # TODO Hand class
