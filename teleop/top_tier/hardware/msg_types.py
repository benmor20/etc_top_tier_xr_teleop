from typing import Protocol, Any
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorState_, MotorCmd_


class DeviceState(Protocol):
    motor_state: list[MotorState_]


class DeviceCmd(Protocol):
    motor_cmd: list[MotorCmd_]

class LocoClient(Protocol):
    def Init(self):
        pass
    def GetFsmId(self) -> tuple[int, Any]:
        pass
    def SetFsmId(self, fsm_id: int) -> None:
        pass
    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0) -> int:
        pass
    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False) -> None:
        pass
