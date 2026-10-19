import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def main():
    ChannelFactoryInitialize(0, "enp0s31f6")
    audio_client = AudioClient()
    audio_client.SetTimeout(1.0)
    audio_client.Init()

    audio_client.LedControl(0, 0, 0)
    time.sleep(5)
    audio_client.LedControl(0, 0, 255)


if __name__ == '__main__':
    main()
