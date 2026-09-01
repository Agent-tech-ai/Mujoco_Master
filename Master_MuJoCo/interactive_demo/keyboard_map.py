"""Viewer-focused keyboard handling with held keys and debounced taps."""

from __future__ import annotations

import ctypes
from ctypes import wintypes
import os
import time


CONTROL_KEYS = ("W", "S", "A", "D", "Q", "E", "H", "V", "C", "R", "1", "2", "3", "SPACE", "ESC")
WIN32_VIRTUAL_KEYS = {
    **{letter: (ord(letter),) for letter in "WSADQEH VCR".replace(" ", "")},
    "1": (0x31,), "2": (0x32,), "3": (0x33,),
    "SPACE": (0x20,), "ESC": (0x1B,),
}


class ViewerKeyboard:
    """Adapt the repository's proven ViewerKeyboard pattern to demo keys."""

    def __init__(self) -> None:
        self._pulse_until: dict[str, float] = {}
        self._previous: set[str] = set()
        self._native = os.name == "nt"
        self._user32 = None
        if self._native:
            self._user32 = ctypes.WinDLL("user32", use_last_error=True)
            self._user32.GetAsyncKeyState.argtypes = [ctypes.c_int]
            self._user32.GetAsyncKeyState.restype = ctypes.c_short
            self._user32.GetForegroundWindow.argtypes = []
            self._user32.GetForegroundWindow.restype = wintypes.HWND
            self._user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
            self._user32.GetWindowTextLengthW.restype = ctypes.c_int
            self._user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
            self._user32.GetWindowTextW.restype = ctypes.c_int

    @staticmethod
    def _logical_from_keycode(keycode: int) -> str | None:
        if ord("0") <= keycode <= ord("9"):
            return chr(keycode)
        if ord("a") <= keycode <= ord("z"):
            return chr(keycode).upper()
        if ord("A") <= keycode <= ord("Z"):
            return chr(keycode)
        if keycode == ord(" "):
            return "SPACE"
        if keycode == 256:
            return "ESC"
        return None

    def on_key(self, keycode: int) -> None:
        logical = self._logical_from_keycode(keycode)
        if logical in CONTROL_KEYS:
            self._pulse_until[logical] = time.monotonic() + (0.08 if self._native else 0.50)

    def _foreground_is_mujoco(self) -> bool:
        if not self._native or self._user32 is None:
            return False
        window = self._user32.GetForegroundWindow()
        if not window:
            return False
        length = self._user32.GetWindowTextLengthW(window)
        title = ctypes.create_unicode_buffer(max(1, length + 1))
        self._user32.GetWindowTextW(window, title, len(title))
        return "mujoco" in title.value.lower()

    def _native_down(self, logical: str) -> bool:
        return bool(
            self._native
            and self._user32 is not None
            and any(self._user32.GetAsyncKeyState(vk) & 0x8000 for vk in WIN32_VIRTUAL_KEYS[logical])
        )

    def sample(self) -> tuple[set[str], set[str]]:
        now = time.monotonic()
        focused = self._foreground_is_mujoco()
        down = {
            key for key in CONTROL_KEYS
            if (focused and self._native_down(key)) or now < self._pulse_until.get(key, 0.0)
        }
        rising = down - self._previous
        self._previous = down
        self._pulse_until = {key: until for key, until in self._pulse_until.items() if until > now}
        return down, rising
