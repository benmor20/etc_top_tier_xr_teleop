import json

from unitree_sdk2py.r1.loco.r1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient as G1Client
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient as R1Client


class R1LocoClient(R1Client):
    """
    Extension class on the R1's LocoClient, to implement some extra functions
    """
    def __init__(self):
        super().__init__()

    # 7001
    def GetFsmId(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")


class G1LocoClient(G1Client):
    """
    Extension class on the G1's LocoClient, in case extra functions need to be added
    """
    def __init__(self):
        super().__init__()
