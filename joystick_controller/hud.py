"""状态显示。

显示原则：区分"实测值"、"设定值"和"缺测"。
老程序把写死的 currentDepth=300 当深度显示，操作者无法发现自己在盲飞，
这里所有实测量都来自下位机 vofa/STAT，取不到就显示 "--" 并变红。
"""

from __future__ import annotations

import collections
import threading
import time

import pygame

import fonts
import protocol

WINDOW_SIZE = (1080, 660)
PANEL_WIDTH = 420

BACKGROUND = (22, 24, 28)
PANEL = (32, 35, 41)
TEXT = (226, 229, 234)
MUTED = (140, 146, 156)
OK = (104, 200, 132)
WARN = (226, 182, 92)
BAD = (226, 106, 106)
ACCENT = (110, 168, 232)


class Console:
    """滚动日志，同时写到标准输出便于事后复查。

    write() 会被接收线程和心跳线程调用（SerialLink 的日志回调），
    而主线程同时在遍历 lines 画 HUD。deque 在遍历过程中被 append
    会抛 RuntimeError: deque mutated during iteration —— 串口连不上、
    日志刷得快的时候必然撞上。所以加锁，并且只把快照交给绘制方。
    """

    def __init__(self, capacity: int = 9) -> None:
        self._lock = threading.Lock()
        self._lines: collections.deque[str] = collections.deque(maxlen=capacity)

    def write(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        line = f"{stamp} {text}"
        with self._lock:
            self._lines.append(line)
        print(line, flush=True)

    def snapshot(self) -> list[str]:
        """返回当前日志的副本，绘制方只能拿到这个，不碰原始容器。"""
        with self._lock:
            return list(self._lines)

    @property
    def lines(self) -> list[str]:
        return self.snapshot()


class Hud:
    def __init__(self) -> None:
        # 不用 SysFont：它在 Windows 上会扫字体注册表并可能抛 TypeError，
        # 而且 Consolas 没有中文字形。详见 fonts.py。
        self.font = fonts.load_font(15)
        self.small = fonts.load_font(13)
        self.bold = fonts.load_font(17, bold=True)
        self._y = 0

    # ---- 绘制辅助 ----------------------------------------------------------
    def _line(self, screen, text: str, colour=TEXT, font=None) -> None:
        surface = (font or self.font).render(text, True, colour)
        screen.blit(surface, (14, self._y))
        self._y += surface.get_height() + 3

    def _gap(self, height: int = 7) -> None:
        self._y += height

    @staticmethod
    def _fmt(value: float | None, spec: str = "7.2f", unit: str = "") -> str:
        if value is None:
            return "     --" + unit
        return f"{value:{spec}}{unit}"

    def draw(self, screen, controller, link, snapshot, console,
             frame, frame_age, now: float) -> None:
        screen.fill(BACKGROUND)
        pygame.draw.rect(screen, PANEL, (0, 0, PANEL_WIDTH, WINDOW_SIZE[1]))
        self._y = 12

        state = controller.telemetry
        fresh = controller.telemetry_fresh(now)

        self._draw_link(screen, link, state, fresh, now)
        self._gap()
        self._draw_safety(screen, controller, state, fresh)
        self._gap()
        self._draw_measured(screen, state, fresh)
        self._gap()
        self._draw_targets(screen, controller)
        self._gap()
        self._draw_servos(screen, controller)
        self._gap()
        self._draw_diag(screen, link, state, snapshot)
        self._draw_console(screen, console)
        self._draw_video(screen, frame, frame_age)

    # ---- 分区 --------------------------------------------------------------
    def _draw_link(self, screen, link, state, fresh: bool, now: float) -> None:
        if link.connected:
            self._line(screen, f"链路  {link.port_name}  已连接", OK, self.bold)
        else:
            self._line(screen, f"链路  {link.port_name}  断开，重连中",
                       BAD, self.bold)

        age = state.vofa_age_s(now)
        if age is None:
            self._line(screen, "遥测  从未收到 vofa 行", BAD)
        elif fresh:
            self._line(screen, f"遥测  {age * 1000:.0f} ms 前  "
                               f"SEQ={state.sequence}", MUTED)
        else:
            self._line(screen, f"遥测  已过期 {age:.1f} s，显示值不可信", BAD)

        hb = link.heartbeat_age_s()
        hb_text = "--" if hb == float("inf") else f"{hb * 1000:.0f} ms"
        hb_colour = OK if hb < protocol.COMMAND_TIMEOUT_S / 2 else BAD
        self._line(screen, f"心跳  {hb_text} 前（阈值 500 ms）", hb_colour)

    def _draw_safety(self, screen, controller, state, fresh: bool) -> None:
        if controller.estop_latched:
            self._line(screen, "!! 急停已触发，按 R 复位 !!", BAD, self.bold)

        stopped = state.stopped if fresh else state.stat_stopped
        if stopped is None:
            arm_text, colour = "未知", WARN
        elif stopped:
            arm_text, colour = "已停止", MUTED
        else:
            arm_text, colour = "运行中", OK
        if controller.arm_intent:
            intent = "解锁"
        elif getattr(controller, "arm_pending", False):
            intent = "待入水"
        else:
            intent = "停止"
        self._line(screen, f"状态  下位机 {arm_text}   上位机意图 {intent}",
                   colour, self.bold)
        if getattr(controller, "arm_pending", False):
            measured = controller.telemetry.value_or_none("depth_m")
            shown = "--" if measured is None else f"{measured:.2f}"
            self._line(screen, f"  等待入水：当前 {shown} m，需 ≥ "
                               f"{controller.arm_depth_m:.2f} m 才发 ON", WARN)

        def flag(label: str, value: bool | None, good: bool = True) -> None:
            if value is None:
                self._line(screen, f"  {label}  未知", WARN)
                return
            hit = value if good else not value
            self._line(screen, f"  {label}  {'是' if value else '否'}",
                       OK if hit else BAD)

        flag("反馈就绪 READY", state.ready if fresh else state.stat_ready)
        # READY 灭的时候直接说是哪一项没成立，不用再去翻 EDIAG
        ready_now = state.ready if fresh else state.stat_ready
        if ready_now is False and state.fusion_flags_stat is not None:
            missing = protocol.describe_missing_flags(state.fusion_flags_stat)
            if missing:
                self._line(screen, "    缺：" + "、".join(missing), BAD,
                           self.small)
        flag("压力校准 CAL", state.calibrated if fresh else state.stat_calibrated)
        flag("锁存故障 FAULT", state.fault if fresh else state.stat_fault,
             good=False)

        # 标定零点：固件只能拦住"四路差太多"的情况（倾斜入水标定）；
        # 整机水平泡在水里标的零点四路一致，只能靠绝对值看出来。
        if state.cal_surface_pa is not None:
            offset = state.cal_surface_pa - protocol.NOMINAL_SURFACE_PA
            if abs(offset) > protocol.SURFACE_PA_TOLERANCE:
                self._line(screen, f"  !! 标定零点 {state.cal_surface_pa} Pa，"
                                   f"偏离大气压 {offset / 9810:+.2f} m 水头", BAD)
                self._line(screen, "     零点多半是在水里标的，深度全程不可信，"
                                   "请在空气中断电重启", BAD, self.small)
            else:
                self._line(screen, f"  标定零点  {state.cal_surface_pa} Pa"
                                   f"（四路差 {state.cal_channel_spread_pa} Pa）",
                           MUTED, self.small)

        reason = state.stop_reason
        if reason:
            self._line(screen, f"  最后停止原因  {reason} "
                               f"{protocol.describe_stop_reason(reason)}", WARN)
            # 停机那一刻的 NR 快照。控制环的就绪判据比 STAT 的 READY 多了
            # "本周期压力总线成功"和"8 ms 内等到 IMU 帧"，所以完全可能
            # READY 显示是、却以"6 反馈无效"停机 —— 真正的原因只在这里。
            why = protocol.describe_not_ready(state.stop_not_ready_mask)
            if why:
                self._line(screen, "    停机那一刻未成立：" + "、".join(why),
                           BAD, self.small)
            if state.last_pressure_reject_bits:
                bits = protocol.describe_pressure_reject(
                    state.last_pressure_reject_bits)
                self._line(screen, "    最近一次通道被拒：" + "、".join(bits),
                           WARN, self.small)

    def _draw_measured(self, screen, state, fresh: bool) -> None:
        self._line(screen, "实测（下位机 ESKF）", ACCENT, self.bold)
        if not fresh:
            self._line(screen, "  遥测过期，以下为最后一次收到的值", BAD)

        colour = TEXT if fresh else MUTED
        self._line(screen, f"  深度    {self._fmt(state.value_or_none('depth_m'), '7.3f', ' m')}", colour)
        self._line(screen, f"  横滚    {self._fmt(state.value_or_none('roll_deg'), '7.2f', ' °')}", colour)
        self._line(screen, f"  俯仰    {self._fmt(state.value_or_none('pitch_deg'), '7.2f', ' °')}", colour)
        self._line(screen, f"  航向    {self._fmt(state.value_or_none('yaw_deg'), '7.2f', ' °')}", colour)

        if state.pressure_legacy:
            # legacy 裸值，不是米也不是 Pa：空气中约 1000~1015。
            # 标签必须写清楚，否则操作者会当成深度读。
            cells = []
            for value in state.pressure_legacy:
                cells.append("   --   " if value == protocol.MISSING
                             else f"{value:8.2f}")
            self._line(screen, "  四路压力(legacy 裸值，非米)", MUTED, self.small)
            self._line(screen, "   " + " ".join(cells), MUTED, self.small)
        # 四路压差隐含的倾角。入水后 ESKF 用压差做姿态观测，压差一旦超过
        # 安装基线，就没有任何姿态能解释它，固件会连整帧深度一起作废。
        tilt = protocol.pressure_tilt(state.pressure_legacy)
        if tilt is not None:
            if tilt["beyond_geometry"]:
                self._line(screen, "  !! 压差超出几何极限：前后高差 "
                                   f"{tilt['front_minus_back_m'] * 100:+.1f} cm / "
                                   f"基线 {protocol.PITCH_BASELINE_M * 100:.1f} cm",
                           BAD, self.small)
                self._line(screen, "     机器近乎立着，任何姿态都解释不了这个压差，"
                                   "ESKF 会拒绝整帧 → 深度失效 → READY 灭", BAD,
                           self.small)
                self._line(screen, "     把机器平着放进水里再试", BAD, self.small)
            else:
                colour = WARN if tilt["near_limit"] else MUTED
                self._line(screen, f"  压差隐含姿态  俯仰 {tilt['pitch_deg']:+.1f}°  "
                                   f"横滚 {tilt['roll_deg']:+.1f}°"
                                   + ("   接近几何极限" if tilt["near_limit"] else ""),
                           colour, self.small)
                # 压差是纯物理量：哪一路深哪一边就低。它和 IMU 同号才说明
                # 安装朝向假设是对的；持续反号意味着 imu_to_body 或压力通道
                # 映射差了绕法线 180 度（见固件 LC_IMU_BOARD_YAW_180）。
                for label, measured, implied in (
                        ("俯仰", state.value_or_none("pitch_deg"), tilt["pitch_deg"]),
                        ("横滚", state.value_or_none("roll_deg"), tilt["roll_deg"])):
                    if (measured is not None and abs(measured) > 4.0
                            and abs(implied) > 4.0 and measured * implied < 0):
                        self._line(screen, f"  !! {label}：IMU {measured:+.1f}° 与压差 "
                                           f"{implied:+.1f}° 反号，安装朝向可能差 180°",
                                   BAD, self.small)

        if state.i2c2_errors:
            self._line(screen, f"  I2C2 累计错误 {state.i2c2_errors}", WARN,
                       self.small)
        if state.raw_mask is not None:
            self._line(screen, f"  有效通道 RAW=0b{state.raw_mask:04b}  "
                               f"FUSED=0b{(state.fused_mask or 0):04b}",
                       MUTED, self.small)

    def _draw_targets(self, screen, controller) -> None:
        self._line(screen, "设定（上位机）", ACCENT, self.bold)
        if not controller.setpoints_valid:
            self._line(screen, "  未解锁，设定值将在 ON 时由实测初始化", MUTED)
            return
        mode = "UP/DN 步进" if controller.legacy_depth else "FSET 绝对值"
        if getattr(controller, "depth_hold", True):
            self._line(screen, f"  目标深度  {controller.target_depth_m:7.3f} m"
                               f"   [{mode}]")
        else:
            self._line(screen, "  深度闭环 关（D 键切换）  只稳姿态，"
                               "深浅由浮力配平决定", WARN)
        self._line(screen, f"  目标航向  {controller.target_yaw_deg:7.2f} °"
                           f"   偏航闭环 {'开' if controller.yaw_hold else '关'}")
        self._line(screen, f"  目标横滚/俯仰  {controller.target_roll_deg:.1f}° "
                           f"/ {controller.target_pitch_deg:.1f}°", MUTED,
                   self.small)

    def _draw_servos(self, screen, controller) -> None:
        self._line(screen, "舵机", ACCENT, self.bold)
        sent = tuple(int(round(v)) for v in controller.servo_us)
        echoed = protocol.firmware_servo_echo(sent)
        labels = ("左上", "左下", "右上", "右下")
        for index, label in enumerate(labels):
            angle = protocol.servo_pwm_to_angle_deg(sent[index])
            text = (f"  {label}  发出 {sent[index]:4d} us  "
                    f"{angle:+6.1f}°")
            if echoed[index] != sent[index]:
                # 第三路被固件反转，必须让操作者看见实际值
                text += f"  → 舵机实收 {echoed[index]:4d} us"
            self._line(screen, text, self.small and TEXT, self.small)

    def _draw_diag(self, screen, link, state, snapshot) -> None:
        self._line(screen, "诊断", ACCENT, self.bold)
        pad = snapshot.name if snapshot.connected else "未连接"
        self._line(screen, f"  手柄  {pad}",
                   TEXT if snapshot.connected else BAD, self.small)
        stats = link.stats
        self._line(screen, f"  发送 {stats.sent}  失败 {stats.send_failures}  "
                           f"重连 {stats.reconnects}  收行 {stats.lines_received}",
                   MUTED, self.small)
        if state.cycle_us is not None:
            self._line(screen, f"  控制周期 {state.cycle_us} us  "
                               f"峰值 {state.max_cycle_us} us  "
                               f"超期 {state.deadline_misses}", MUTED, self.small)
            self._line(screen, f"  丢包 {state.rx_drops}  "
                               f"I2C 错误 {state.i2c_errors}  "
                               f"温度 {self._fmt(state.temperature_c, '.1f', ' °C')}",
                       MUTED, self.small)

    def _draw_console(self, screen, console) -> None:
        lines = console.snapshot()   # 先取快照，绝不在绘制中遍历活的容器
        y = WINDOW_SIZE[1] - 12 - len(lines) * 16
        for line in lines:
            surface = self.small.render(line[:58], True, MUTED)
            screen.blit(surface, (14, y))
            y += 16

    def _draw_video(self, screen, frame, frame_age) -> None:
        area = pygame.Rect(PANEL_WIDTH + 12, 12,
                           WINDOW_SIZE[0] - PANEL_WIDTH - 24,
                           WINDOW_SIZE[1] - 24)
        pygame.draw.rect(screen, PANEL, area)

        if frame is None:
            message = ("摄像头无画面" if frame_age is None
                       else f"画面已过期 {frame_age:.1f} s")
            surface = self.font.render(message, True, BAD)
            screen.blit(surface, surface.get_rect(center=area.center))
            return

        # 按比例缩放进显示区，不再像老程序那样被窗口裁掉一半
        surface = pygame.surfarray.make_surface(frame)
        width, height = surface.get_size()
        scale = min(area.width / width, area.height / height)
        surface = pygame.transform.smoothscale(
            surface, (int(width * scale), int(height * scale)))
        screen.blit(surface, surface.get_rect(center=area.center))
