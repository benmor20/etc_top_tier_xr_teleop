import json
import sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient


NETWORK_INTERFACE = "enp0s31f6"


def main():
    ChannelFactoryInitialize(0, NETWORK_INTERFACE)

    c = LocoClient()
    c.SetTimeout(10.0)
    c.Init()

    # c.Move(1.0, 0.0, 0.0)
    ret = c.SetVelocity(1.0, 0.0, 0.0, 1.0)
    print(f"Code is: {ret}")
    # p = {}
    # velocity = [0.15, 0., 0.]
    # p["velocity"] = velocity
    # p["duration"] = 2.0
    # parameter = json.dumps(p)
    # code, data = c._Call(7105, parameter)
    # print(f"SetVelocity {code=}, {data=}")
    # time.sleep(3)



if __name__ == '__main__':
    main()
