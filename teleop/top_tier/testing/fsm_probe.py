import json
from datetime import datetime
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient

NETWORK_INTERFACE = "enp0s31f6"


def get_timestamp() -> str:
    current_time = datetime.now()
    return str(current_time).split(" ")[1]


def main():
    ChannelFactoryInitialize(0, NETWORK_INTERFACE)
    c = LocoClient()  # this works regardless of g1 or r1 LocoClient, evidently
    c.SetTimeout(1.0)
    c.Init()

    prev_code = None

    while True:
        code = c._Call(7001, "{}")
        if code != prev_code:
            print(f"{get_timestamp()} - {code}")
            prev_code = code
        time.sleep(0.01)


if __name__ == '__main__':
    main()
