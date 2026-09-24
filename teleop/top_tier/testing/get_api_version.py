import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient

from top_tier.general.constants import NETWORK_INTERFACE

ChannelFactoryInitialize(0, NETWORK_INTERFACE)

c = LocoClient()
c.SetTimeout(1.0)
c.Init()

server_code, server_version = c.GetServerApiVersion()
print("server code:", server_code)
print("ai_sport server api version:", server_version)

version = c.GetApiVersion()
print("ai_sport api version:", version)
