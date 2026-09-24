import sys
import time
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient as R1LocoClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient as G1LocoClient

from top_tier.general.constants import NETWORK_INTERFACE

INTERVAL = 0.05


def force_state(client, stop_event, log, state: int):
    while not stop_event.is_set():
        try:
            code = client.SetFsmId(state)
            log.append((time.time(), code))
        except Exception as e:
            log.append((time.time(), f"EXC: {e}"))
        time.sleep(INTERVAL)

def main():
    if len(sys.argv) < 3 or sys.argv[1].lower() not in ("g1", "r1") or not sys.argv[2].isdigit():
        print(f"Usage: python {sys.argv[0]} (g1|r1) (fsm_state)")
        return
    is_g1 = sys.argv[1].lower() == "g1"
    fsm_state = int(sys.argv[2])

    ChannelFactoryInitialize(0, NETWORK_INTERFACE)
    client = G1LocoClient() if is_g1 else R1LocoClient()
    client.SetTimeout(1.0)
    client.Init()

    stop_event = threading.Event()
    log = []
    t = threading.Thread(target=force_state, args=(client, stop_event, log, fsm_state), daemon=True)

    print(f"Forcing SetFsmId({fsm_state}) every {INTERVAL}s. Ctrl+C to stop.")
    t.start()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass
    stop_event.set()
    t.join(timeout=1.0)

    codes = [c for _, c in log]
    nonzero = [c for c in codes if c != 0]
    print(f"\nSent {len(codes)} calls. {len(nonzero)} returned nonzero/error: {set(nonzero) if nonzero else 'none'}")

if __name__ == "__main__":
    main()
