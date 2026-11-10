import sys
import time

import numpy as np
from pynput import keyboard
from pynput.keyboard import KeyCode
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_


def main():
    if len(sys.argv) == 1:
        hand = "left"
    else:
        hand = sys.argv[1].lower()
        if hand == "l": hand = "left"
        elif hand == "r": hand = "right"
        if hand not in ("left", "right"):
            raise ValueError("Usage: python dex3_hand_probe.py (left|right)")
    print(f"{hand=}")

    ChannelFactoryInitialize(0, "wlp0s20f3")
    sub = ChannelSubscriber(f"rt/dex3/{hand}/state", HandState_)
    sub.Init()
    pos = np.zeros((7,))
    run = True

    def exit():
        nonlocal run
        run = False
    def log():
        print(pos)
    commands = {"e": exit, "p": log}
    def on_press(key: KeyCode):
        if not isinstance(key, KeyCode):
            return
        for code, func in commands.items():
            if key.char == code:
                func()
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while run:
        time.sleep(1. / 250.)
        msg = sub.Read(0.1)
        if msg is None:
            continue
        pos = np.array([m.q for m in msg.motor_state])


if __name__ == '__main__':
    main()
