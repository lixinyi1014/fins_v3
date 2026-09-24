"""手柄标定向导：一步步问你按哪个键、推哪根杆，然后写出 my_pad.json。

用法（不接下位机、不接推进器，纯本地）：
    python pad_probe.py

窗口里会逐项提示，你照着做就行：
  * 提示按键时，按一下对应的键。
  * 提示推摇杆时，把杆推到底并保持住，识别到就自动进入下一项，然后松手。
  * 按 S 跳过当前项（你这只手柄没有那个键），按 R 重做上一项，按 ESC 放弃退出。

全部走完后会把 my_pad.json 写到本脚本同目录，
启动主程序时加 --mapping my_pad.json。

摇杆方向的约定：**推到右 / 推到上 = 正值 = 对应舵机脉宽增大**。
上机后如果哪一路舵机方向反了，把 my_pad.json 里那一项的 invert 取反即可。
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pygame

import fonts

# 采集顺序。括号里是 Xbox 手柄的建议按键，只是提示，按你自己习惯来也行。
BUTTON_STEPS = [
    ("ascend", "上浮 / 目标深度变浅", "建议 A"),
    ("descend", "下潜 / 目标深度变深", "建议 B"),
    ("yaw_ccw", "左转（偏航闭环开时 = 航向减）", "建议 X"),
    ("yaw_cw", "右转（偏航闭环开时 = 航向加）", "建议 Y"),
    ("yaw_hold_toggle", "偏航闭环 开 / 关", "建议 LB"),
    ("fine_modifier", "舵机微调（按住时步进减半）", "建议 RB"),
    ("estop", "急停", "建议 Back / View"),
    ("arm_toggle", "解锁 / 上锁", "建议 Start / Menu"),
    ("servo_left_center", "左两路舵机回中", "建议按下左摇杆"),
    ("servo_right_center", "右两路舵机回中", "建议按下右摇杆"),
]

AXIS_STEPS = [
    ("left_horizontal", "把【左摇杆】推到最右，保持住"),
    ("left_vertical", "把【左摇杆】推到最上，保持住"),
    ("right_horizontal", "把【右摇杆】推到最右，保持住"),
    ("right_vertical", "把【右摇杆】推到最上，保持住"),
]

BUTTON_LABELS = {key: what for key, what, _ in BUTTON_STEPS}
AXIS_LABELS = dict(AXIS_STEPS)

BASELINE_SETTLE_S = 1.5  # 识别到手柄后先跟随多久，再把静止基准定下来
AXIS_TRIGGER = 0.6    # 偏离静止值多少算"推到底了"
AXIS_DOMINANCE = 1.8  # 冠军轴的偏移要大过亚军这么多倍，否则算"推斜了"
AXIS_RELEASE = 0.25   # 回到静止值多少算"松手了"
REST_SUSPECT = 0.3    # 静止值超过这个数的轴多半是扳机，不作为摇杆候选

BACKGROUND = (20, 22, 26)
TEXT = (226, 229, 234)
MUTED = (140, 146, 156)
OK = (104, 200, 132)
WARN = (226, 182, 92)
BAD = (226, 106, 106)
ACCENT = (110, 168, 232)


class Wizard:
    def __init__(self) -> None:
        self.steps: list[tuple[str, str, str, str]] = []
        for key, what, hint in BUTTON_STEPS:
            self.steps.append(("button", key, what, hint))
        self.steps.append(("hat", "hat", "把【十字键】推向上，保持住",
                           "前后左右都靠它"))
        for key, what in AXIS_STEPS:
            self.steps.append(("axis", key, what, "推到底再松手"))

        self.index = 0
        self.buttons: dict[str, int] = {}
        self.axes: dict[str, int] = {}
        self.invert: dict[str, bool] = {}
        self.hat: int | None = None
        self.rest: dict[int, float] = {}
        self.cooling = False
        self.note = ""

    # ---- 状态 ---------------------------------------------------------------
    @property
    def done(self) -> bool:
        return self.index >= len(self.steps)

    def current(self):
        return None if self.done else self.steps[self.index]

    def advance(self, note: str) -> None:
        self.note = note
        self.index += 1
        self.cooling = True

    def skip(self) -> None:
        kind, key, _, _ = self.steps[self.index]
        if kind == "button":
            self.buttons.pop(key, None)
        elif kind == "axis":
            self.axes.pop(key, None)
            self.invert.pop(key, None)
        self.advance(f"已跳过 {key}")

    def redo(self) -> None:
        if self.index == 0:
            return
        self.index -= 1
        kind, key, _, _ = self.steps[self.index]
        if kind == "button":
            self.buttons.pop(key, None)
        elif kind == "axis":
            self.axes.pop(key, None)
            self.invert.pop(key, None)
        else:
            self.hat = None
        self.note = f"重做 {key}"
        self.cooling = True

    # ---- 采集 ---------------------------------------------------------------
    def sample_rest(self, pad) -> None:
        self.rest = {i: pad.get_axis(i) for i in range(pad.get_numaxes())}

    def released(self, pad) -> bool:
        """判断上一项的输入是否已经松开，避免一次动作连吃两项。"""
        if any(pad.get_button(i) for i in range(pad.get_numbuttons())):
            return False
        for h in range(pad.get_numhats()):
            if pad.get_hat(h) != (0, 0):
                return False
        for i in range(pad.get_numaxes()):
            if abs(pad.get_axis(i) - self.rest.get(i, 0.0)) > AXIS_RELEASE:
                return False
        return True

    def handle_event(self, event: pygame.event.Event) -> None:
        step = self.current()
        if step is None or self.cooling:
            return
        kind, key, _, _ = step
        if kind == "button" and event.type == pygame.JOYBUTTONDOWN:
            stolen = self.steal_button(event.button, key)
            self.buttons[key] = event.button
            self.advance(f"{key} = 按键 {event.button}{stolen}")
        elif kind == "hat" and event.type == pygame.JOYHATMOTION:
            if event.value == (0, 0):
                return
            self.hat = event.hat
            if event.value != (0, 1):
                self.note = (f"hat{event.hat} 记下了，但你推的方向是 "
                             f"{event.value}，不是正上方；前后左右可能会对调")
            else:
                self.note = f"hat = {event.hat}"
            self.index += 1
            self.cooling = True

    def requeue(self, kind: str, key: str) -> None:
        """把某一项重新排到队尾，等会儿再问一遍。"""
        what = (BUTTON_LABELS if kind == "button" else AXIS_LABELS)[key]
        self.steps.append((kind, key, what, "这一项要重采一次"))

    def steal_axis(self, index: int, new_key: str) -> str:
        """轴已经记给别人时，抢过来并把原来那项排到队尾重采。

        以前这里是直接拒绝，结果只要采错一次就彻底卡死，
        唯一的出路是按 R —— 而窗口一旦没有键盘焦点，按 R 是没反应的。
        向导不该有任何只能靠快捷键脱身的死角。
        """
        for name, assigned in list(self.axes.items()):
            if assigned == index and name != new_key:
                self.axes.pop(name, None)
                self.invert.pop(name, None)
                self.requeue("axis", name)
                return f"（轴 {index} 原先记给 {name}，已改记给 {new_key}，"\
                       f"{name} 排到最后重采）"
        return ""

    def steal_button(self, number: int, new_key: str) -> str:
        for name, assigned in list(self.buttons.items()):
            if assigned == number and name != new_key:
                self.buttons.pop(name, None)
                self.requeue("button", name)
                return f"（按键 {number} 原先记给 {name}，已改记给 {new_key}，"\
                       f"{name} 排到最后重采）"
        return ""

    def axis_candidate(self, pad):
        """返回 (轴号, 偏移, 说明)。轴号为 None 表示这一帧还不能采。

        只取"偏移最大的轴"是不够的：摇杆推斜一点，另一根轴的偏移可能更大，
        就会把 X 记成 Y。所以要求冠军明显压过亚军（AXIS_DOMINANCE 倍），
        否则一直等，并把当前情况说清楚，不让人对着不动的界面干猜。
        """
        ranked = []
        for i in range(pad.get_numaxes()):
            if abs(self.rest.get(i, 0.0)) > REST_SUSPECT:
                continue   # 扳机，永远不作为摇杆候选
            delta = pad.get_axis(i) - self.rest.get(i, 0.0)
            ranked.append((abs(delta), i, delta))
        ranked.sort(reverse=True)
        if not ranked:
            return None, 0.0, "没有可用的摇杆轴（全部被判为扳机）"

        magnitude, index, delta = ranked[0]
        if magnitude < 0.2:
            return None, 0.0, "还没检测到明显的摇杆动作"
        if magnitude < AXIS_TRIGGER:
            return None, 0.0, (f"轴 {index} 偏移 {delta:+.2f}，还不够 "
                               f"{AXIS_TRIGGER:.1f}，再推到底一点")
        if len(ranked) > 1 and ranked[1][0] * AXIS_DOMINANCE > magnitude:
            return None, 0.0, (f"轴 {index} 和轴 {ranked[1][1]} 在同时动"
                               f"（{delta:+.2f} / {ranked[1][2]:+.2f}），"
                               "请只沿一个方向推到底，别推斜")
        return index, delta, ""

    def poll_axis(self, pad) -> None:
        step = self.current()
        if step is None or self.cooling or step[0] != "axis":
            return
        index, delta, _ = self.axis_candidate(pad)
        if index is None:
            return
        key = step[1]
        stolen = self.steal_axis(index, key)
        self.axes[key] = index
        # 约定：推到右 / 推到上 为正。原始读数为负就打开 invert。
        self.invert[key] = delta < 0
        self.advance(f"{key} = 轴 {index}"
                     f"（推到底读 {delta:+.2f}，"
                     f"invert={'开' if self.invert[key] else '关'}）{stolen}")

    # ---- 输出 ---------------------------------------------------------------
    def mapping(self) -> dict:
        return {
            "buttons": dict(self.buttons),
            "axes": dict(self.axes),
            "hat": self.hat if self.hat is not None else 0,
            "deadzone": 0.25,
            "invert": {k: bool(v) for k, v in self.invert.items()},
        }


def main() -> int:
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((900, 620))
    pygame.display.set_caption("手柄标定向导")
    font = fonts.load_font(16)
    big = fonts.load_font(20, bold=True)
    clock = pygame.time.Clock()

    wizard = Wizard()
    pad = None
    saved = False
    baseline_until = 0.0

    running = True
    while running:
        now = time.monotonic()
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_s and not wizard.done:
                    wizard.skip()
                elif event.key == pygame.K_r:
                    wizard.redo()
                elif event.key == pygame.K_b and pad is not None:
                    baseline_until = now + BASELINE_SETTLE_S
                    wizard.cooling = False
                    wizard.note = "重新采静止基准"
            elif event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                pad = None

        if pad is None and pygame.joystick.get_count() > 0:
            pad = pygame.joystick.Joystick(0)
            pad.init()
            wizard.sample_rest(pad)
            # 有些手柄的扳机轴在刚枚举出来时报 0，过一会儿才跳到 -1.00。
            # 立刻采基准会把 0 记成静止值，之后 released() 永远不成立，
            # 整个向导就卡在"请先松手"。所以先持续刷新一段时间再定下来。
            baseline_until = now + BASELINE_SETTLE_S
            wizard.note = f"已识别 {pad.get_name()}"

        settling = pad is not None and now < baseline_until
        if pad is not None:
            if settling:
                wizard.sample_rest(pad)   # 这段时间里持续跟随，不采集任何输入
                wizard.cooling = False
            elif wizard.cooling:
                if wizard.released(pad):
                    wizard.cooling = False
            else:
                for event in events:
                    wizard.handle_event(event)
                wizard.poll_axis(pad)

        if wizard.done and not saved:
            target = Path(__file__).with_name("my_pad.json")
            payload = json.dumps(wizard.mapping(), indent=2, ensure_ascii=False)
            target.write_text(payload, encoding="utf-8")
            saved = True
            print(f"\n已写入 {target}\n")
            print(payload)

        # ---- 绘制 ----------------------------------------------------------
        screen.fill(BACKGROUND)
        y = 14

        def line(text: str, colour=TEXT, f=None, x: int = 18) -> None:
            nonlocal y
            surface = (f or font).render(text, True, colour)
            screen.blit(surface, (x, y))
            y += surface.get_height() + 4

        if pad is None:
            line("没有检测到手柄。插上 USB 或配对蓝牙后本窗口会自动识别。",
                 BAD, big)
        elif settling:
            line("正在读取静止基准，请勿触碰手柄…", WARN, big)
            line("")
            line("（部分手柄的扳机轴要过一会儿才报出真实值，"
                 "所以要等一下再定基准）", MUTED)
        elif wizard.done:
            line("标定完成，my_pad.json 已写好。", OK, big)
            line("")
            line("接下来（推进器动力线务必先断开）：", ACCENT)
            line("  python joystick_control.py --port COM11 "
                 "--mapping my_pad.json --no-camera")
            line("")
            line("窗口出来后先什么都别按，确认四路舵机稳稳停在 1500 us。", MUTED)
            line("哪一路自己在爬，就是映射还有错，按 R 回来重做那一项。", MUTED)
        else:
            kind, key, what, hint = wizard.current()
            line(f"第 {wizard.index + 1} / {len(wizard.steps)} 项", MUTED)
            line(what, ACCENT, big)
            line(f"（{hint}）", MUTED)
            line("")
            if wizard.cooling:
                line("请先松手…", WARN)
                line("一直卡在这里？说明静止基准不对，按 B 重新采一次。", MUTED)
            elif kind == "button":
                line("按一下对应的键", TEXT)
            elif kind == "hat":
                line("推十字键", TEXT)
            else:
                line("推到底并保持住", TEXT)
                line("")
                # 卡住时必须说清楚卡在哪，否则只能对着不动的界面干猜
                _, _, why = wizard.axis_candidate(pad)
                if why:
                    line(why, WARN)

        # 右栏：已记录的项，避免和左边的提示、底部的读数互相压字
        y = 150
        line("已记录：", ACCENT, x=430)
        for name, index in wizard.buttons.items():
            line(f"{name:<19}按键 {index}", OK, font, x=430)
        if wizard.hat is not None:
            line(f"{'hat':<19}{wizard.hat}", OK, font, x=430)
        for name, index in wizard.axes.items():
            line(f"{name:<19}轴 {index}  "
                 f"invert={'开' if wizard.invert.get(name) else '关'}",
                 OK, font, x=430)

        if pad is not None:
            y = 500
            suspect = [i for i, v in wizard.rest.items()
                       if abs(v) > REST_SUSPECT]
            if suspect:
                line(f"静止时不为 0 的轴 {suspect}（多半是扳机），"
                     "已排除在摇杆之外", WARN)
            values = "  ".join(f"{i}:{pad.get_axis(i):+.2f}"
                               for i in range(pad.get_numaxes()))
            line(f"当前各轴  {values}", MUTED)

        y = 572
        if wizard.note:
            line(wizard.note, MUTED)
        line("S 跳过当前项    R 重做上一项    B 重采静止基准    ESC 退出"
             "    （键盘按键需先点一下本窗口）", MUTED)

        pygame.display.flip()
        clock.tick(60)

    if not saved:
        print("\n没有完成标定，my_pad.json 未写入。")
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
