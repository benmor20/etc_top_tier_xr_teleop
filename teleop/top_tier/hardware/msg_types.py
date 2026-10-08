from typing import Protocol
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorState_, MotorCmd_


class DeviceState(Protocol):
    motor_state: list[MotorState_]


class DeviceCmd(Protocol):
    motor_cmd: list[MotorCmd_]
