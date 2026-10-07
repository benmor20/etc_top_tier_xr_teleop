import time
from enum import Enum

import numpy as np
from pynput import keyboard
from pynput.keyboard import KeyCode
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_


# From dex3_hand_probe
# All open, thumb yaw at limit
# [ 1.02534223 -0.99715334 -0.03664392 -0.04053353 -0.01297298 -0.04780016
#  -0.01713444]
# Thumb closed, yaw at other limit, others open
# [-1.07003045  0.65492386  1.39915001 -0.04667382 -0.01324224 -0.03076498
#  -0.0174377 ]
# Thumb open at middle yaw, others closed
# [-0.0038186  -1.0049423  -0.02236757 -1.61517811 -1.78804016 -1.64871001
#  -1.7736367 ]

class JointType(Enum):
    ThumbYaw = 0
    ThumbJoint1 = 1
    ThumbJoint2 = 2
    IndexJoint1 = 3
    IndexJoint2 = 4
    MiddleJoint1 = 5
    MiddleJoint2 = 6


LOWER = np.array([-1., -1., 0., -1.6, -1.8, -1.6, -1.8])
UPPER = np.array([1., .65, 1.45, 0., 0., 0., 0.])
KP = 1.5
KD = 0.2


CLOSED = np.concatenate(([0.], UPPER[1:3], LOWER[3:]))
OPEN = np.concatenate(([0.], LOWER[1:3], UPPER[3:]))


def get_motor_mode(id: int, status: int = 0x01, timeout: int = 0):
    motor_mode = 0
    motor_mode |= id & 0x0F
    motor_mode |= (status & 0x07) << 4
    motor_mode |= (timeout & 0x01) << 7
    return motor_mode


def main():
    ChannelFactoryInitialize(0, "enp0s31f6")
    pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
    pub.Init()

    cmd = unitree_hg_msg_dds__HandCmd_()
    for joint_type in JointType:
        joint_idx = joint_type.value
        cmd.motor_cmd[joint_idx].mode = get_motor_mode(joint_idx)
        cmd.motor_cmd[joint_idx].q = 0.
        cmd.motor_cmd[joint_idx].dq = 0.
        cmd.motor_cmd[joint_idx].tau = 0.
        cmd.motor_cmd[joint_idx].kp = KP
        cmd.motor_cmd[joint_idx].kd = KD

    sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
    sub.Init()
    state = None
    while state is None:
        state = sub.Read()
        time.sleep(1. / 250.)
    current_pos = np.array([m.q for m in state.motor_state])

    run = True
    target = OPEN
    def open():
        nonlocal target
        target = OPEN
    def close():
        nonlocal target
        target = CLOSED
    def exit():
        nonlocal run
        run = False
    commands = {"e": exit, "o": open, "c": close}
    def on_press(key: KeyCode):
        for code, func in commands.items():
            if key.char == code:
                func()
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while run:
        delta = target - current_pos
        max_dist = np.max(np.abs(delta))
        scale = 0. if np.isclose(max_dist, 0.) else min(30. / 250. / max_dist, 1.)
        clipped = current_pos + delta * scale
        assert np.all(clipped >= LOWER), f"{clipped}, {LOWER}"
        assert np.all(clipped <= UPPER), f"{clipped}, {UPPER}"
        for joint_type in JointType:
            joint_idx = joint_type.value
            cmd.motor_cmd[joint_idx].q = clipped[joint_idx]
        # print(target, scale, clipped)
        pub.Write(cmd)
        current_pos = clipped
        time.sleep(1. / 250.)


if __name__ == '__main__':
    main()
