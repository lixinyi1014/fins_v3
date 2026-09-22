"""手柄输入层：按键映射、死区、热插拔。

老程序每帧重建 pygame.joystick.Joystick(0) 并 init()，
不仅浪费，而且手柄未连接时直接抛未捕获异常。
这里只在设备变化时重建，掉线时把所有输入视为中位。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import pygame

#: 默认映射按 Xbox 布局（沿用原程序的编号）。
#: 不同手柄编号不同，可用 --mapping xxx.json 覆盖。
DEFAULT_MAPPING = {
    "buttons": {
        "ascend": 0,          # A
        "descend": 1,         # B
        "yaw_ccw": 2,         # X
        "yaw_cw": 3,          # Y
        "yaw_hold_toggle": 4,  # LB
        "fine_modifier": 5,   # RB，按住时舵机步进减半
        "estop": 6,           # Back / View
        "arm_toggle": 7,      # Start / Menu
        "servo_left_center": 8,   # 左摇杆按下
        "servo_right_center": 9,  # 右摇杆按下
    },
    "axes": {
        "left_horizontal": 0,
        "left_vertical": 1,
        "right_horizontal": 2,
        "right_vertical": 3,
    },
    "hat": 0,
    "deadzone": 0.25,
    "invert": {
        "left_vertical": False,
        "left_horizontal": True,
        "right_vertical": True,
        "right_horizontal": True,
    },
}


def load_mapping(path: str | None) -> dict:
    """读取映射覆盖文件，缺省用 DEFAULT_MAPPING。"""
    mapping = json.loads(json.dumps(DEFAULT_MAPPING))   # 深拷贝
    if not path:
        return mapping
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for section in ("buttons", "axes", "invert"):
        if section in data:
            mapping[section].update(data[section])
    for key in ("hat", "deadzone"):
        if key in data:
            mapping[key] = data[key]
    return mapping


@dataclass
class InputSnapshot:
    """一帧的手柄状态。所有字段在手柄掉线时都为中位/False。"""

    connected: bool = False
    name: str = "未连接"

    # 电平：按住持续有效
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    yaw_cw: bool = False
    yaw_ccw: bool = False
    ascend: bool = False
    descend: bool = False
    fine: bool = False

    # 边沿：按下瞬间一次
    pressed: set[str] = field(default_factory=set)

    # 摇杆，已去死区并按配置取反
    axes: dict[str, float] = field(default_factory=dict)

    def any_translation(self) -> bool:
        return any((self.forward, self.backward, self.left, self.right,
                    self.yaw_cw, self.yaw_ccw))


class GamepadReader:
    def __init__(self, mapping: dict, *, log=print) -> None:
        self.mapping = mapping
        self._log = log
        self._joystick: pygame.joystick.JoystickType | None = None
        self._instance_id: int | None = None
        self._button_names = {v: k for k, v in mapping["buttons"].items()}
        pygame.joystick.init()
        self._acquire()

    # ---- 设备管理 ----------------------------------------------------------
    def _acquire(self) -> None:
        if pygame.joystick.get_count() == 0:
            if self._joystick is not None:
                self._log("[手柄] 已断开，输入归零")
            self._joystick = None
            self._instance_id = None
            return
        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error as exc:
            self._log(f"[手柄] 初始化失败: {exc}")
            self._joystick = None
            return
        self._joystick = joystick
        self._instance_id = joystick.get_instance_id()
        self._log(f"[手柄] 已连接 {joystick.get_name()}："
                  f"{joystick.get_numaxes()} 轴 / "
                  f"{joystick.get_numbuttons()} 键 / "
                  f"{joystick.get_numhats()} 十字键")

    def handle_event(self, event: pygame.event.Event) -> None:
        """处理设备增删事件，实现热插拔。"""
        if event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
            self._acquire()

    @property
    def connected(self) -> bool:
        return self._joystick is not None

    # ---- 采样 --------------------------------------------------------------
    def _button(self, name: str) -> bool:
        index = self.mapping["buttons"].get(name)
        if index is None or self._joystick is None:
            return False
        if index >= self._joystick.get_numbuttons():
            return False
        return bool(self._joystick.get_button(index))

    def _axis(self, name: str) -> float:
        index = self.mapping["axes"].get(name)
        if index is None or self._joystick is None:
            return 0.0
        if index >= self._joystick.get_numaxes():
            return 0.0
        value = self._joystick.get_axis(index)
        if abs(value) < self.mapping["deadzone"]:
            return 0.0
        if self.mapping["invert"].get(name):
            value = -value
        return value

    def button_name(self, index: int) -> str | None:
        return self._button_names.get(index)

    def sample(self, pressed: set[str]) -> InputSnapshot:
        """采集当前电平状态，合并调用方收集到的边沿事件。"""
        if self._joystick is None:
            return InputSnapshot(pressed=pressed)

        snapshot = InputSnapshot(
            connected=True,
            name=self._joystick.get_name(),
            yaw_cw=self._button("yaw_cw"),
            yaw_ccw=self._button("yaw_ccw"),
            ascend=self._button("ascend"),
            descend=self._button("descend"),
            fine=self._button("fine_modifier"),
            pressed=pressed,
        )

        hat_index = self.mapping["hat"]
        if hat_index < self._joystick.get_numhats():
            hat_x, hat_y = self._joystick.get_hat(hat_index)
            snapshot.forward = hat_y > 0
            snapshot.backward = hat_y < 0
            snapshot.left = hat_x < 0
            snapshot.right = hat_x > 0

        for name in ("left_horizontal", "left_vertical",
                     "right_horizontal", "right_vertical"):
            snapshot.axes[name] = self._axis(name)
        return snapshot
