from typing import Callable

from pynput import keyboard
from pynput.keyboard import KeyCode, Key
from unitree_sdk2py.utils.singleton import Singleton


_KeyTypeInternal = Key | KeyCode
_KeyType = _KeyTypeInternal | str
_KEY_STR_TO_KEY = {
    "alt": Key.alt,
    "alt_l": Key.alt_l,
    "alt_r": Key.alt_r,
    "alt_gr": Key.alt_gr,
    "backspace": Key.backspace,
    "caps_lock": Key.caps_lock,
    "cmd": Key.cmd,
    "cmd_l": Key.cmd_l,
    "cmd_r": Key.cmd_r,
    "ctrl": Key.ctrl,
    "ctrl_l": Key.ctrl_l,
    "ctrl_r": Key.ctrl_r,
    "delete": Key.delete,
    "down": Key.down,
    "end": Key.end,
    "enter": Key.enter,
    "esc": Key.esc,
    "f1": Key.f1,
    "f2": Key.f2,
    "f3": Key.f3,
    "f4": Key.f4,
    "f5": Key.f5,
    "f6": Key.f6,
    "f7": Key.f7,
    "f8": Key.f8,
    "f9": Key.f9,
    "f10": Key.f10,
    "f11": Key.f11,
    "f12": Key.f12,
    "f13": Key.f13,
    "f14": Key.f14,
    "f15": Key.f15,
    "f16": Key.f16,
    "f17": Key.f17,
    "f18": Key.f18,
    "f19": Key.f19,
    "f20": Key.f20,
    "home": Key.home,
    "left": Key.left,
    "page_down": Key.page_down,
    "page_up": Key.page_up,
    "right": Key.right,
    "shift": Key.shift,
    "shift_l": Key.shift_l,
    "shift_r": Key.shift_r,
    "space": Key.space,
    "tab": Key.tab,
    "up": Key.up,
    "media_play_pause": Key.media_play_pause,
    "media_volume_mute": Key.media_volume_mute,
    "media_volume_down": Key.media_volume_down,
    "media_volume_up": Key.media_volume_up,
    "media_previous": Key.media_previous,
    "media_next": Key.media_next,
    "insert": Key.insert,
    "menu": Key.menu,
    "num_lock": Key.num_lock,
    "pause": Key.pause,
    "print_screen": Key.print_screen,
    "scroll_lock": Key.scroll_lock,
}


def to_key(key: _KeyType) -> Key | KeyCode:
    if isinstance(key, Key) or isinstance(key, KeyCode):
        return key
    return _KEY_STR_TO_KEY[key] if key in _KEY_STR_TO_KEY else KeyCode.from_char(key)

def key_to_str(key: _KeyType) -> str:
    if isinstance(key, str):
        return key.lower()
    if isinstance(key, Key):
        return str(key).split(".")[1]
    if isinstance(key, KeyCode):
        if key.char is None:
            raise ValueError(f"Given KeyCode has no attached char: {key}")
        return key.char
    raise ValueError(f"Unknown key type: {type(key)}")


class _KeyboardListener(Singleton):
    """
    Central class for tracking keyboard events and using callbacks
    """

    def __init__(self):
        super().__init__()
        self._key_press_callbacks: dict[str, list[Callable[[], None]]] = {}
        self._key_release_callbacks: dict[str, list[Callable[[], None]]] = {}
        self._pressed_keys: set[str] = set()
        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.start()

    @staticmethod
    def _on_event(key: _KeyTypeInternal, callback_funcs: dict[str, list[Callable[[], None]]]) -> None:
        """
        Called when a keyboard event is triggered

        Args:
            key: which key triggered the event
            callback_funcs: a mapping of key strings to a list of functions to call when the event is triggered by
                that key
        """
        key_str = key_to_str(key)
        if key_str in callback_funcs:
            for callback in callback_funcs[key_str]:
                callback()

    def _on_press(self, key: _KeyTypeInternal) -> None:
        """
        Callback function for when a key is pressed. Triggers all given callbacks for that key, and also notes that
        the key is currently pressed.

        Args:
            key: the key that was pressed
        """
        self._on_event(key, self._key_press_callbacks)
        self._pressed_keys.add(key_to_str(key))

    def _on_release(self, key: _KeyTypeInternal) -> None:
        """
        Callback function for when a key is released. Triggers all given callbacks for that key, and also notes that
        the key is no longer pressed.

        Args:
            key: the key that was released
        """
        self._on_event(key, self._key_release_callbacks)
        self._pressed_keys.remove(key_to_str(key))

    def add_listener(self, key: _KeyType, callback: Callable[[], None], on_press: bool = True) -> None:
        """
        Add a callback function to be called when the given key is pressed

        Keep the reference to the callback if you intend to remove it later

        Args:
            key: the key which will trigger this callback
            callback: the function to call when the event is triggered by this key
            on_press: if True, press events will trigger the callback. Else, release events will trigger the callback
        """
        callback_funcs = self._key_press_callbacks if on_press else self._key_release_callbacks
        key_str = key_to_str(key)
        if key_str not in callback_funcs:
            callback_funcs[key_str] = []
        callback_funcs[key_str].append(callback)

    def remove_listener(self, key: _KeyType, callback: Callable[[], None], on_press: bool = True) -> bool:
        """
        Remove a callback function so it no longer gets called when the event triggers

        Should be the same instance of the callback that was added

        Args:
            key: the key to remove this callback from
            callback: the callback function to remove
            on_press: if True, removes this callback from press events. Else, removes it from release events

        Returns:
            a bool, True if the callback was successfully removed, False if it never existed (for that key/event type)
        """
        callback_funcs = self._key_press_callbacks if on_press else self._key_release_callbacks
        key_str = key_to_str(key)
        if key_str in callback_funcs and callback in callback_funcs[key_str]:
            callback_funcs[key_str].remove(callback)
            return True
        return False

    def is_key_pressed(self, key: _KeyType) -> bool:
        """
        Find out if a certain key is currently pressed

        Args:
            key: the key to check the status of

        Returns:
            True if the key is currently being pressed, False otherwise
        """
        return key_to_str(key) in self._pressed_keys


KeyboardListener = _KeyboardListener()
