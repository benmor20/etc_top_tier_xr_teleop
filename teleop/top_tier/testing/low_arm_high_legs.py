#!/usr/bin/env python3
import json
import time

import sys
from pynput import keyboard
import numpy as np
from pynput.keyboard import Key, KeyCode
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient as R1LocoClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient as G1LocoClient
from unitree_sdk2py.utils.crc import CRC


NETWORK_INTERFACE = "enp0s31f6"

ARM_SDK_TOPIC = "rt/arm_sdk"
LOWSTATE_TOPIC = "rt/lowstate"

# R1-A5 arm/head/waist joints
CONTROL_JOINTS = [
    15, 16, 17, 18, 19,     # left arm
    22, 23, 24, 25, 26,     # right arm
    13,                     # waist yaw
    29, 30,                 # head
]

KP = [
    50.0, 50.0, 40., 40., 30.,
    50.0, 50.0, 40., 40., 30.,
    50.,
    15., 15.
]
KD = [
    2.0, 2.0, 2., 2., 2.,
    2.0, 2.0, 2., 2., 2.,
    3.,
    1., 1.
]


def main():
    if len(sys.argv) < 2 or sys.argv[1].lower() not in ("g1", "r1"):
        print(f"Usage: python {sys.argv[0]} (g1|r1)")
        return
    is_g1 = sys.argv[1].lower() == "g1"

    ChannelFactoryInitialize(0, NETWORK_INTERFACE)
    c = G1LocoClient() if is_g1 else R1LocoClient()
    c.SetTimeout(1.0)
    c.Init()

    state_sub = ChannelSubscriber(
        LOWSTATE_TOPIC,
        LowState_,
    )
    state_sub.Init()

    print("Waiting for lowstate...")

    state = None

    while state is None:
        state = state_sub.Read()
        time.sleep(0.01)

    print("Got lowstate.")

    arm_pub = ChannelPublisher(
        ARM_SDK_TOPIC,
        LowCmd_,
    )
    arm_pub.Init()
    print("Initialized arm pub")

    cmd = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()

    if is_g1:
        cmd.motor_cmd[29].q = 1.
    else:
        cmd.mode_pr = 100

    cmd.mode_machine = state.mode_machine
    print("Created cmd")

    # Hold current positions
    for idx, joint_id in enumerate(CONTROL_JOINTS):

        cmd.motor_cmd[joint_id].mode = 1
        cmd.motor_cmd[joint_id].kp = KP[idx]
        cmd.motor_cmd[joint_id].kd = KD[idx]

        cmd.motor_cmd[joint_id].q = state.motor_state[joint_id].q
        cmd.motor_cmd[joint_id].dq = 0.0
        cmd.motor_cmd[joint_id].tau = 0.0
    print("Said to hold joints")

    target_key = KeyCode.from_char("m")
    key_was_pressed = False
    key_is_pressed = False
    def on_press(key):
        if key == target_key:
            nonlocal key_is_pressed
            key_is_pressed = True
        else:
            print(f"Unknown key: {key}")
    def on_release(key):
        if key == target_key:
            nonlocal key_is_pressed
            key_is_pressed = False
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    print("Setup listener")

    try:
        while True:

            latest = state_sub.Read()

            if latest is not None:
                cmd.mode_machine = latest.mode_machine

            cmd.crc = crc.Crc(cmd)
            arm_pub.Write(cmd)

            if key_is_pressed and not key_was_pressed:
                print("Moving forward...")
                c.Move(0.5, 0.0, 0.0, False)
            key_was_pressed = key_is_pressed

            time.sleep(1.0 / 250.0)

    except KeyboardInterrupt:
        print("Shutting down...")
        steps = 500
        for weight in np.linspace(cmd.mode_pr / 100.0, 0.0, num=steps + 1):
            cmd.mode_pr = int(np.clip(weight, 0.0, 1.0) * 100.0)
            cmd.crc = crc.Crc(cmd)
            arm_pub.Write(cmd)
            try:
                time.sleep(1. / 250.)
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
