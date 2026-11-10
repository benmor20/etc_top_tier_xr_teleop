import sys
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


# LOWER = np.array([-1., -1., 0., -1.6, -1.8, -1.6, -1.8])
# UPPER = np.array([1., .65, 1.5, 0., 0., 0., 0.])

LOWER = np.deg2rad(np.array([-60., -60., 0., -90., -100., -90., -100.]))
UPPER = np.deg2rad(np.array([60., 35., 100., 0., 0., 0., 0.]))
KP = 1.5
KD = 0.2


def get_motor_mode(id: int, status: int = 0x01, timeout: int = 0):
    motor_mode = 0
    motor_mode |= id & 0x0F
    motor_mode |= (status & 0x07) << 4
    motor_mode |= (timeout & 0x01) << 7
    return motor_mode


def main():
    if len(sys.argv) == 1:
        hand = "left"
    else:
        hand = sys.argv[1].lower()
        if hand == "l":
            hand = "left"
        elif hand == "r":
            hand = "right"
        if hand not in ("left", "right"):
            raise ValueError("Usage: python dex3_hand_probe.py (left|right)")
    print(f"{hand=}")

    if hand == "right":
        lower, upper = -UPPER, -LOWER
    else:
        lower, upper = LOWER, UPPER

    closed = np.concatenate(([0.], upper[1:3], lower[3:]))
    opened = np.concatenate(([0.], lower[1:3], upper[3:]))
    if hand == "right":
        closed, opened = opened, closed  # maybe this is backwards for left too idk
    gripped = np.concatenate(([0.], closed[1:5], opened[5:]))

    normal_kff = np.zeros(gripped.shape)
    fire_kff = normal_kff.copy()
    fire_kff_idxs = [5, 6]
    fire_kff[fire_kff_idxs] = 0.1
    kff = normal_kff

    ChannelFactoryInitialize(0, "wlp0s20f3")
    pub = ChannelPublisher(f"rt/dex3/{hand}/cmd", HandCmd_)
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

    run = True
    target = opened
    def open():
        nonlocal target, kff
        target = opened
        kff = normal_kff
    def close():
        nonlocal target, kff
        target = closed
        kff = normal_kff
    def grip():
        nonlocal target, kff
        target = gripped
        kff = normal_kff
    def fire():
        nonlocal target, kff
        target = closed
        kff = fire_kff
    def increase():
        nonlocal fire_kff
        fire_kff[fire_kff_idxs] *= 2.
    def decrease():
        nonlocal fire_kff
        fire_kff[fire_kff_idxs] /= 2.
    def exit():
        nonlocal run
        run = False
    commands = {"e": exit, "o": open, "c": close, "g": grip, "f": fire, "r": increase, "v": decrease}
    def on_press(key: KeyCode):
        if not isinstance(key, KeyCode):
            return
        for code, func in commands.items():
            if key.char == code:
                func()
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    sub = ChannelSubscriber(f"rt/dex3/{hand}/state", HandState_)
    sub.Init()
    state = None
    while state is None and run:
        state = sub.Read(0.1)
        time.sleep(1. / 250.)
    if not run:
        return
    current_pos = np.array([m.q for m in state.motor_state])
    current_pos = np.clip(current_pos, lower, upper)

    while run:
        delta = target - current_pos
        max_dist = np.max(np.abs(delta))
        scale = 0. if np.isclose(max_dist, 0.) else min(30. / 250. / max_dist, 1.)
        clipped = current_pos + delta * scale
        assert np.all(clipped >= lower), f"{clipped}, {lower}"
        assert np.all(clipped <= upper), f"{clipped}, {upper}"
        for joint_type in JointType:
            joint_idx = joint_type.value
            cmd.motor_cmd[joint_idx].q = clipped[joint_idx]
            cmd.motor_cmd[joint_idx].tau = kff[joint_idx]
        # print(target, scale, clipped)
        pub.Write(cmd)
        current_pos = clipped
        time.sleep(1. / 250.)


if __name__ == '__main__':
    main()
