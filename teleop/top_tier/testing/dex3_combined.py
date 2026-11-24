import argparse
import time
from enum import Enum
import numpy as np
from pynput import keyboard
from pynput.keyboard import KeyCode
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, HandCmd_
from unitree_sdk2py.idl import unitree_hg_msg_dds__HandCmd_


def hand_type(value: str | None) -> str:
    """Normalize l/left and r/right (case-insensitive) to 'left' or 'right'."""
    if value is None:
        return "right"
    v = value.lower()
    if v in ("left", "l"):
        return "left"
    if v in ("right", "r"):
        return "right"
    raise argparse.ArgumentTypeError(
        f"invalid choice: {value!r} (choose from 'left', 'l', 'right', 'r')"
    )


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Example CLI.")  # -h/--help is built in

    parser.add_argument(
        "-H", "--hand",
        type=hand_type,
        metavar="{left,l,right,r}",
        help="which hand to use: left/l or right/r",
    )
    parser.add_argument(
        "-c", "--continuous-feed",
        action="store_true",
        help="enable continuous feed (default: prints only when P is pressed)",
    )
    parser.add_argument(
        "-n", "--network-interface",
        type=str,
        metavar="IFACE",
        help="network interface to connect to robot (overrides --ethernet and --wireless)",
    )
    parser.add_argument(
        "-e", "--ethernet",
        action="store_true",
        help="use ethernet/wired connection to robot (ignored if --network-interface is given)",
    )
    parser.add_argument(
        "-w", "--wireless",
        action="store_true",
        help="use wireless connection to robot (ignored if --network-interface or --ethernet is given)",
    )
    parser.add_argument(
        "-m", "--movement",
        action="store_true",
        help="control the motion of the hand"
    )

    args = parser.parse_args(argv)

    # Precedence: -n > -e > -w
    if not args.network_interface:
        if args.ethernet or not args.wireless:
            args.network_interface = "enp0s31f6"
        else:
            args.network_interface = "wlp0s20f3"

    return args


class JointType(Enum):
    ThumbYaw = 0
    ThumbJoint1 = 1
    ThumbJoint2 = 2
    IndexJoint1 = 3
    IndexJoint2 = 4
    MiddleJoint1 = 5
    MiddleJoint2 = 6


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
    args = parse_args()
    hand = hand_type(args.hand)

    if hand == "right":
        lower, upper = -UPPER, -LOWER
    else:
        lower, upper = LOWER, UPPER
    closed = np.concatenate(([0.], upper[1:3], lower[3:]))
    opened = np.concatenate(([0.], lower[1:3], upper[3:]))
    if hand == "right":
        closed, opened = opened, closed  # maybe this is backwards for left too idk
    gripped = np.concatenate(([0.], closed[1:5], opened[5:]))

    print(f"Initializing {hand} hand...")

    ChannelFactoryInitialize(0, args.network_interface)
    sub = ChannelSubscriber(f"rt/dex3/{hand}/state", HandState_)
    sub.Init()
    pub = ChannelPublisher(f"rt/dex3/{hand}/cmd", HandCmd_)
    pub.Init()
    pos = np.zeros((7,))
    run = True

    cmd = unitree_hg_msg_dds__HandCmd_()
    for joint_type in JointType:
        joint_idx = joint_type.value
        cmd.motor_cmd[joint_idx].mode = get_motor_mode(joint_idx)
        cmd.motor_cmd[joint_idx].q = 0.
        cmd.motor_cmd[joint_idx].dq = 0.
        cmd.motor_cmd[joint_idx].tau = 0.
        cmd.motor_cmd[joint_idx].kp = KP
        cmd.motor_cmd[joint_idx].kd = KD

    target = opened
    normal_kff = np.zeros(gripped.shape)
    fire_kff = normal_kff.copy()
    fire_kff_idxs = [5, 6]
    fire_kff[fire_kff_idxs] = 0.1
    kff = normal_kff
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
    def print_hand():
        print(pos)
    def exit():
        nonlocal run
        run = False

    commands = {"e": exit, "o": open, "c": close, "g": grip, "f": fire, "r": increase, "v": decrease, "p": print_hand}
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
        if args.continuous_feed:
            print(pos)

        if args.movement:
            delta = target - pos
            max_dist = np.max(np.abs(delta))
            scale = 0. if np.isclose(max_dist, 0.) else min(30. / 250. / max_dist, 1.)
            clipped = pos + delta * scale
            for joint_type in JointType:
                joint_idx = joint_type.value
                cmd.motor_cmd[joint_idx].q = clipped[joint_idx]
                cmd.motor_cmd[joint_idx].tau = kff[joint_idx]
            pub.Write(cmd)
            time.sleep(1. / 250.)


if __name__ == "__main__":
    main()
