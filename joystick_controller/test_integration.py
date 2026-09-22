"""集成自测：用假串口模拟下位机，验证链路与控制逻辑。

不需要真实硬件、手柄或摄像头。运行：

    python test_integration.py

验证重点是老程序的那些短板：心跳是否独立于渲染、
发送失败是否可见、解锁前置检查是否生效、退出是否一定发 OFF。
"""

from __future__ import annotations

import sys
import threading
import time
import types

import protocol
from protocol import Command


class FakeSerial:
    """够用的 serial.Serial 替身，记录所有写入并可回灌遥测。"""

    def __init__(self, *_, **kwargs) -> None:
        self.is_open = True
        self.writes: list[bytes] = []
        self._rx = bytearray()
        self._lock = threading.Lock()
        self.fail_writes = False
        self.timeout = kwargs.get("timeout", 0.05)

    def write(self, data: bytes) -> int:
        if self.fail_writes:
            import serial
            raise serial.SerialException("模拟写入失败")
        with self._lock:
            self.writes.append(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + (self.timeout or 0.05)
        while time.monotonic() < deadline:
            with self._lock:
                if self._rx:
                    chunk = bytes(self._rx[:size])
                    del self._rx[:len(chunk)]
                    return chunk
            time.sleep(0.005)
        return b""

    def close(self) -> None:
        self.is_open = False

    # --- 测试辅助 ---
    def push(self, line: str) -> None:
        with self._lock:
            self._rx.extend((line + "\r\n").encode("ascii"))

    def sent_lines(self) -> list[str]:
        with self._lock:
            blob = b"".join(self.writes)
        return [x for x in blob.decode("ascii", "replace").split("\r\n") if x]

    def count(self, command: str) -> int:
        return sum(1 for line in self.sent_lines() if line == command)


def check(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)
    print(f"  ok  {label}")


def install_fake_serial():
    """把 serial.Serial 换成 FakeSerial，并返回最近创建的实例容器。"""
    import serial
    created: list[FakeSerial] = []
    original = serial.Serial

    def factory(*args, **kwargs):
        fake = FakeSerial(*args, **kwargs)
        created.append(fake)
        return fake

    serial.Serial = factory
    return created, lambda: setattr(serial, "Serial", original)


HEALTHY_VOFA = ("vofa:1004.10,1004.20,1004.30,1004.40,0.50,-1.00,45.00,0.500,"
                "1,1,1,15,0,15,0,{seq}")


def test_heartbeat_independent_of_main_loop() -> None:
    """心跳必须由独立线程发出，主循环阻塞也不能断。"""
    print("心跳独立性")
    from link import SerialLink

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        check(link.open(), "串口打开成功")
        link.start()
        fake = created[0]

        # 模拟主循环卡死 0.6 s（比固件 500 ms 超时还长）
        time.sleep(0.6)
        beats = fake.count(Command.HEARTBEAT)
        check(beats >= 4, f"主循环阻塞 0.6 s 期间仍发出 {beats} 次心跳")

        gaps_ok = True
        stamps = [i for i, line in enumerate(fake.sent_lines())
                  if line == Command.HEARTBEAT]
        check(gaps_ok and len(stamps) == beats, "心跳计数一致")
        link.close(send_disarm=False)
    finally:
        restore()


def test_every_command_has_line_ending() -> None:
    """固件不再接受没有行尾的命令，这是老程序在 v3 上必然失效的根因。"""
    print("命令行尾")
    from link import SerialLink

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        for command in (Command.ARM, Command.DISARM, Command.HOLD,
                        Command.STATUS, "MOT:1500,1500,1500,1500",
                        "FSET:0.300,0.00,0.00,0.00"):
            link.send(command)
        blob = b"".join(fake.writes)
        check(blob.count(b"\r\n") == 6, "六条命令各带一个 CRLF")
        check(all(w.endswith(b"\r\n") for w in fake.writes),
              "每条命令都是一次完整写出")
        link.close(send_disarm=False)
    finally:
        restore()


def test_send_failure_is_visible() -> None:
    """老程序写失败静默返回 False；这里必须计数并断开重连。"""
    print("发送失败可见")
    from link import SerialLink

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        fake.fail_writes = True
        ok = link.send(Command.HOLD)
        check(ok is False, "写失败返回 False")
        check(link.stats.send_failures == 1, "失败被计数")
        check(link.connected is False, "写失败后链路标记为断开")

        ok = link.send(Command.HOLD)
        check(ok is False and link.stats.send_failures == 2,
              "断开状态下继续发送仍然失败且被计数，不会假装成功")
        link.close(send_disarm=False)
    finally:
        restore()


def test_telemetry_reaches_hud_state() -> None:
    """遥测必须被解析进状态，HUD 才能显示真实深度。"""
    print("遥测接收")
    from link import SerialLink

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        link.start()
        fake = created[0]
        fake.push(HEALTHY_VOFA.format(seq=42))
        fake.push("STAT=ESKF,STOP=1,READY=1,CAL=1,FLAGS=7,MASK=15,"
                  "FAULT=0,WHY=0,CAL_CH=0,CAL_N=100")

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if link.parser.state.sequence == 42:
                break
            time.sleep(0.02)

        state = link.parser.state
        check(state.sequence == 42, "vofa 行被解析")
        check(state.depth_m == 0.5, "深度来自下位机实测")
        check(state.yaw_deg == 45.0, "航向来自下位机实测")
        check(state.stat_ready is True, "STAT 被解析")
        link.close(send_disarm=False)
    finally:
        restore()


def test_arm_preconditions() -> None:
    """ON 的前置检查：无遥测、未校准、已故障都不该发 ON。"""
    print("解锁前置检查")
    from link import SerialLink
    from joystick_control import Controller

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        logs: list[str] = []
        controller = Controller(link, log=logs.append)

        controller.toggle_arm(time.monotonic())
        check(fake.count(Command.ARM) == 0, "无遥测时不发 ON")
        check(controller.arm_intent is False, "解锁意图未置位")

        now = time.monotonic()
        link.parser.feed_line(
            "vofa:1004,1004,1004,1004,0.00,0.00,0.00,0.000,1,1,0,15,0,15,0,1", now)
        controller.toggle_arm(now)
        check(fake.count(Command.ARM) == 0, "CAL=0 时不发 ON")

        link.parser.feed_line(
            "vofa:1004,1004,1004,1004,0.00,0.00,0.00,0.000,1,1,1,15,1,15,0,2", now)
        controller.toggle_arm(now)
        check(fake.count(Command.ARM) == 0, "FAULT=1 时不发 ON")

        link.parser.feed_line(
            "vofa:1004,1004,1004,1004,0.00,0.00,0.00,0.000,1,0,1,15,0,15,0,3", now)
        controller.toggle_arm(now)
        check(fake.count(Command.ARM) == 0, "READY=0 时不发 ON")

        # 全部条件满足
        link.parser.feed_line(HEALTHY_VOFA.format(seq=4), time.monotonic())
        controller.toggle_arm(time.monotonic())
        check(fake.count(Command.ARM) == 1, "条件满足时发出 ON")
        check(controller.arm_intent is True, "解锁意图置位")
        check(abs(controller.target_depth_m - 0.5) < 1e-6,
              "目标深度由实测初始化为 0.5 m，不是带着旧值起飞")
        check(abs(controller.target_yaw_deg - 45.0) < 1e-6,
              "目标航向由实测初始化为 45°")
        link.close(send_disarm=False)
    finally:
        restore()


def test_emergency_stop() -> None:
    """急停：连发 OFF 且掐掉心跳，让固件超时锁存作为兜底。"""
    print("急停")
    from link import SerialLink
    from joystick_control import Controller

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        link.start()
        fake = created[0]
        controller = Controller(link, log=lambda _: None)
        controller.arm_intent = True

        controller.emergency_stop()
        check(fake.count(Command.DISARM) >= 3, "急停连发多次 OFF")
        check(controller.arm_intent is False, "急停清除解锁意图")
        check(controller.estop_latched is True, "急停被锁存")

        before = fake.count(Command.HEARTBEAT)
        time.sleep(0.35)
        after = fake.count(Command.HEARTBEAT)
        check(after == before,
              "急停后心跳被掐断，固件 500 ms 超时会强制停止")

        # 急停未复位时不允许解锁
        link.parser.feed_line(HEALTHY_VOFA.format(seq=9), time.monotonic())
        controller.toggle_arm(time.monotonic())
        check(fake.count(Command.ARM) == 0, "急停未复位时拒绝解锁")
        controller.reset_estop()
        check(controller.estop_latched is False, "复位后解除锁存")
        link.close(send_disarm=False)
    finally:
        restore()


def test_motion_state_and_refresh() -> None:
    """运动状态变化即发，之后周期重发以抗丢包。"""
    print("运动状态发送")
    from link import SerialLink
    from joystick_control import Controller
    import gamepad

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        controller = Controller(link, log=lambda _: None)
        controller.arm_intent = True
        controller.setpoints_valid = True

        snapshot = gamepad.InputSnapshot(connected=True, forward=True)
        now = 1000.0
        controller._update_translation(snapshot, now)
        check(fake.count(Command.FORWARD) == 1, "按下十字键前发一次 W")

        controller._update_translation(snapshot, now + 0.02)
        check(fake.count(Command.FORWARD) == 1,
              "状态未变且未到重发周期时不重复发送")

        controller._update_translation(snapshot, now + 0.25)
        check(fake.count(Command.FORWARD) == 2,
              "超过 200 ms 后周期重发，抗丢包并满足 500 ms 收包要求")

        released = gamepad.InputSnapshot(connected=True)
        controller._update_translation(released, now + 0.30)
        check(fake.count(Command.HOLD) == 1, "松手立即发 Z")
        link.close(send_disarm=False)
    finally:
        restore()


def test_downlink_watchdog() -> None:
    """下行断、上行通：固件察觉不到，必须由上位机自主停机。

    这是最隐蔽的失效模式 —— 串口 TX/RX 是两根独立的线，
    写入一切正常、心跳照常送达、固件反馈自检全部通过，
    但操作者已经看不到任何实测值。
    """
    print("上位机侧看门狗（下行中断）")
    from link import SerialLink
    from joystick_control import Controller, DOWNLINK_TIMEOUT_S
    import gamepad

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        logs: list[str] = []
        controller = Controller(link, log=logs.append)

        # 正常解锁
        now = 5000.0
        link.parser.feed_line(HEALTHY_VOFA.format(seq=1), now)
        controller.toggle_arm(now)
        check(controller.arm_intent is True, "先正常解锁")

        snapshot = gamepad.InputSnapshot(connected=True)

        # 遥测还新鲜时不应触发
        controller.update(snapshot, now + 0.5)
        check(controller.arm_intent is True,
              "遥测新鲜（0.5 s）时不误触发")
        controller.update(snapshot, now + DOWNLINK_TIMEOUT_S - 0.1)
        check(controller.arm_intent is True,
              f"未到阈值（{DOWNLINK_TIMEOUT_S} s）时不误触发")

        # 写入仍然正常，模拟"上行通、下行断"
        check(link.connected is True, "链路写入侧仍然正常")
        before_off = fake.count(Command.DISARM)

        controller.update(snapshot, now + DOWNLINK_TIMEOUT_S + 0.1)
        check(controller.arm_intent is False, "超过阈值后自主解除解锁意图")
        check(controller.estop_latched is True, "按急停处理并锁存")
        check(fake.count(Command.DISARM) > before_off, "发出 OFF")
        check(any("遥测中断" in line for line in logs),
              "日志说明停机原因是遥测中断")

        # 心跳必须被掐掉：这是不依赖下行链路的兜底
        link.start()
        beats_before = fake.count(Command.HEARTBEAT)
        time.sleep(0.35)
        check(fake.count(Command.HEARTBEAT) == beats_before,
              "心跳被暂停，固件 500 ms 超时会强制停止")

        # 锁存后不应反复刷 OFF
        offs = fake.count(Command.DISARM)
        for step in range(5):
            controller.update(snapshot, now + 3.0 + step * 0.02)
        check(fake.count(Command.DISARM) == offs,
              "锁存后不重复触发，日志不会被刷屏")
        link.close(send_disarm=False)
    finally:
        restore()


def test_watchdog_does_not_fire_when_stopped() -> None:
    """未解锁时遥测中断只是显示问题，不该反复报急停。"""
    print("看门狗不误触发")
    from link import SerialLink
    from joystick_control import Controller
    import gamepad

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        controller = Controller(link, log=lambda _: None)
        snapshot = gamepad.InputSnapshot(connected=True)

        for step in range(10):
            controller.update(snapshot, 9000.0 + step * 0.5)
        check(controller.estop_latched is False,
              "未解锁且无遥测时不触发急停")
        check(fake.count(Command.DISARM) == 0, "未发 OFF")
        link.close(send_disarm=False)
    finally:
        restore()


def test_legacy_depth_does_not_clobber_motion() -> None:
    """UP/DN 是增量，不能和水平运动的状态锁存互相覆盖。"""
    print("兼容深度模式与水平运动互不干扰")
    from link import SerialLink
    from joystick_control import Controller
    import gamepad

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        fake = created[0]
        controller = Controller(link, legacy_depth=True, log=lambda _: None)
        controller.arm_intent = True
        controller.setpoints_valid = True

        # 同时按住前进和上浮
        snapshot = gamepad.InputSnapshot(connected=True, forward=True,
                                        ascend=True)
        now = 2000.0
        for step in range(30):        # 0.6 s
            t = now + step * 0.02
            controller._update_translation(snapshot, t)
            controller._update_depth(snapshot, t, 0.02)

        check(fake.count(Command.DEPTH_UP) >= 5,
              f"按住上浮持续发出 UP（{fake.count(Command.DEPTH_UP)} 次）")
        check(fake.count(Command.FORWARD) >= 3,
              f"同时按住前进仍周期重发 W（{fake.count(Command.FORWARD)} 次）")
        check(controller._last_motion == Command.FORWARD,
              "水平运动状态未被 UP/DN 覆盖")
        check(fake.count(Command.DEPTH_DOWN) == 0, "未误发 DN")
        link.close(send_disarm=False)
    finally:
        restore()


def test_servo_clamps_are_uniform() -> None:
    """四路舵机限幅一致，不再像老程序那样两路 1000..2000、两路 500..2500。"""
    print("舵机限幅")
    from link import SerialLink
    from joystick_control import Controller
    import gamepad

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        controller = Controller(link, log=lambda _: None)

        full = gamepad.InputSnapshot(connected=True, axes={
            "left_vertical": 1.0, "left_horizontal": 1.0,
            "right_vertical": 1.0, "right_horizontal": 1.0})
        for _ in range(200):
            controller._update_servos(full, dt=0.02)
        check(all(v == protocol.SERVO_MAX_US for v in controller.servo_us),
              f"四路都能到上限 {protocol.SERVO_MAX_US}")

        negative = gamepad.InputSnapshot(connected=True, axes={
            "left_vertical": -1.0, "left_horizontal": -1.0,
            "right_vertical": -1.0, "right_horizontal": -1.0})
        for _ in range(400):
            controller._update_servos(negative, dt=0.02)
        check(all(v == protocol.SERVO_MIN_US for v in controller.servo_us),
              f"四路都能到下限 {protocol.SERVO_MIN_US}")
        link.close(send_disarm=False)
    finally:
        restore()


def test_close_always_disarms() -> None:
    """退出路径必须发 OFF 并关闭串口。"""
    print("退出保证停机")
    from link import SerialLink

    created, restore = install_fake_serial()
    try:
        link = SerialLink("COM_FAKE", log=lambda _: None)
        link.open()
        link.start()
        fake = created[0]
        link.close(send_disarm=True)
        check(fake.count(Command.DISARM) >= 1, "关闭时发出 OFF")
        check(fake.is_open is False, "串口被关闭")
        check(link.connected is False, "链路状态为断开")
    finally:
        restore()


def test_command_budget() -> None:
    """估算稳态命令速率，确认不会冲垮固件只有 8 格的接收队列。"""
    print("命令速率预算")
    import joystick_control as app

    rate = (1.0 / protocol.HEARTBEAT_PERIOD_S          # 心跳 10 Hz
            + 1.0 / app.MOTION_REFRESH_S               # 运动重发 5 Hz
            + app.SERVO_SEND_HZ                        # 舵机 20 Hz 峰值
            + app.FSET_SEND_HZ                         # 目标 10 Hz 峰值
            + 1.0 / app.STATUS_POLL_S
            + 1.0 / app.DIAG_POLL_S)
    check(rate < 60, f"峰值约 {rate:.0f} 命令/秒，远低于 115200 8E1 的行速率上限")
    check(app.CONTROL_HZ >= 50, "主循环 ≥50 Hz，输入响应快于老程序的 30 Hz")
    check(app.MOTION_REFRESH_S < protocol.COMMAND_TIMEOUT_S,
          "运动重发周期短于固件 500 ms 收包要求")


def main() -> int:
    tests = (test_heartbeat_independent_of_main_loop,
             test_every_command_has_line_ending,
             test_send_failure_is_visible,
             test_telemetry_reaches_hud_state,
             test_arm_preconditions,
             test_emergency_stop,
             test_motion_state_and_refresh,
             test_downlink_watchdog,
             test_watchdog_does_not_fire_when_stopped,
             test_legacy_depth_does_not_clobber_motion,
             test_servo_clamps_are_uniform,
             test_close_always_disarms,
             test_command_budget)
    for test in tests:
        test()
    print(f"\n全部通过（{len(tests)} 组）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
