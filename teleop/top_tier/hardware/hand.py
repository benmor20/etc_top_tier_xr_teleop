from enum import Enum, auto
from typing import Callable

from unitree_sdk2py.idl import unitree_hg_msg_dds__HandState_, unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, HandCmd_

from hardware.hardware_device import HardwareDevice, DEVICE_T
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


class Hand(HardwareDevice[tuple[HandType, bool], HandState_, HandCmd_]):
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
        side_str = "left" if left_hand else "right"
        state_name = f"rt/dex3/{side_str}/state"
        cmd_name = f"rt/dex3/{side_str}/cmd"
        super().__init__((hand_type, left_hand), state_name, cmd_name)

    # ABSTRACT METHODS ----------------------------------------------------------------------------

    @property
    def _state_type(self) -> type[HandState_]:
        return HandState_

    @property
    def _cmd_type(self) -> type[HandCmd_]:
        return HandCmd_

    @property
    def _state_default_factory(self) -> Callable[[], HandState_]:
        return unitree_hg_msg_dds__HandState_

    @property
    def _cmd_default_factory(self) -> Callable[[], HandCmd_]:
        return unitree_hg_msg_dds__HandCmd_

    def _get_joint_map(self, device_type: tuple[HandType, bool]) -> dict[JointType, Joint]:
        return device_type[0].get_joint_map(device_type[1])

    # MISC PROPERTIES -----------------------------------------------------------------------------

    @property
    def hand_type(self) -> HandType:
        """
        Returns:
            what type of hand this instance represents
        """
        return self.device_type[0]

    @property
    def is_left_hand(self) -> bool:
        """
        Returns:
            True of this hand is a left hand, False otherwise
        """
        return self.device_type[1]
