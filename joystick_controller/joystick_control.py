"""fins_v3 手柄遥控上位机。

用法：
    python joystick_control.py --port COM11
    python joystick_control.py --port COM11 --camera 0 --no-camera
    python joystick_control.py --list-ports

与下位机的约定（Lower_Level_Controller/docs/CONFIG_AND_INTERFACES.md）：
  * 每条命令必须以 CRLF 结束，整行 250 ms 内送完，最长 99 字符。
  * 必须约每 100 ms 发一次 HB；500 ms 收不到即锁存停止。
  * ON 只在校准通过、反馈就绪、无故障时才被接受。
  * 深度/姿态目标用 FSET 一次提交；水平移动仍用 W/A/S/D/Q/E/Z。

安全提示：本程序的急停依赖串口可达或固件心跳超时。
按文档要求，必须另有独立的物理动力切断手段。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import pygame

import protocol
from protocol import Command
import gamepad
import hud as hud_module
from link import SerialLink

# ---- 客户端调参 ------------------------------------------------------------
CONTROL_HZ = 50                 # 主循环频率，独立于摄像头
MOTION_REFRESH_S = 0.2          # 运动状态重发周期，防丢包
SERVO_SEND_HZ = 20              # MOT 最高发送频率
FSET_SEND_HZ = 10               # FSET 最高发送频率
STATUS_POLL_S = 1.0             # STAT 轮询周期
DIAG_POLL_S = 5.0               # DIAG 轮询周期

DEPTH_RATE_M_S = 0.10           # 按住 A/B 时目标深度变化速率
DEPTH_STEP_HZ = 10              # --legacy-depth 下 UP/DN 的步进频率

#: 上位机侧看门狗阈值：解锁状态下遥测中断多久即自主停机。
#: 固件 vofa 名义 10 Hz，1.5 s 相当于连丢 15 行，不会被偶发丢包误触发。
DOWNLINK_TIMEOUT_S = 1.5
YAW_RATE_DEG_S = 25.0           # ACL 开启时按住 X/Y 的目标航向变化速率
SERVO_STEP_US_S = 600.0         # 摇杆满偏时舵机脉宽变化速率
TELEMETRY_STALE_S = 1.0         # 遥测超过此年龄视为过期

#: 解锁互锁：ON 之后固件立刻让四路垂直推进器进入深度闭环，桨会转。
#: 空气中长时间空转既无水冷也无负载，所以按 Start 只表达"意图"，
#: 真正的 ON 要等水压计确认已经入水。设为 0 关闭该互锁（台架断动力测试用）。
ARM_DEPTH_M = 0.15

#: 开启偏航闭环时，先连续取这么多帧实测航向求平均作为目标。
#: 单帧航向有噪声（磁力计在机体里受推进器电流干扰），
#: 直接拿一帧当基准会把那一帧的误差固化成永久的航向偏置。
YAW_CAPTURE_SAMPLES = 50


class Controller:
    """把手柄输入翻译成下位机命令，并维护本地设定值。"""

    def __init__(self, link: SerialLink, *, legacy_depth: bool = False,
                 arm_depth_m: float = ARM_DEPTH_M, log=print) -> None:
        self.link = link
        self.log = log
        self.legacy_depth = legacy_depth
        self.arm_depth_m = arm_depth_m

        self.depth_hold = True           # 深度闭环；D 键切换，见 toggle_depth_hold
        self._yaw_samples: list[float] = []   # 偏航基准采样中的临时缓冲
        self._yaw_capture_seq: int | None = None
        self._yaw_capturing = False
        self.arm_intent = False          # 操作者是否希望解锁
        self.arm_pending = False         # 已请求解锁，等水压计确认入水
        self.yaw_hold = False            # ACL 状态
        self.estop_latched = False       # 急停后必须显式复位

        # 舵机设定值，四路统一用 500..2500，不再像老程序那样两路 1000..2000
        self.servo_us = [float(protocol.SERVO_MID_US)] * protocol.SERVO_COUNT

        # 深度/姿态设定值。深度在解锁时由遥测初始化，避免带着旧值起飞
        self.target_depth_m = 0.0
        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_yaw_deg = 0.0
        self.setpoints_valid = False

        self._last_motion = Command.HOLD
        self._last_motion_sent = 0.0
        self._last_depth_step = 0.0
        self._last_servo_sent = 0.0
        self._last_fset_sent = 0.0
        self._last_stat = 0.0
        self._last_diag = 0.0
        self._servo_dirty = False
        self._fset_dirty = False
        self._was_running = False      # 上一帧下位机是否在运行，用来抓"运行→停止"这一刻

    # ---- 状态查询 ----------------------------------------------------------
    @property
    def telemetry(self):
        return self.link.parser.state

    def telemetry_fresh(self, now: float) -> bool:
        age = self.telemetry.vofa_age_s(now)
        return age is not None and age <= TELEMETRY_STALE_S

    # ---- 解锁 / 停止 -------------------------------------------------------
    def toggle_arm(self, now: float) -> None:
        if self.arm_intent:
            self.disarm("操作者请求")
            return
        if self.arm_pending:
            self.arm_pending = False
            self.log("[控制] 已取消待入水解锁")
            return

        if self.estop_latched:
            self.log("[控制] 急停未复位，按 R 复位后再解锁")
            return
        if not self.link.connected:
            self.log("[控制] 串口未连接，无法解锁")
            return

        state = self.telemetry
        if not self.telemetry_fresh(now):
            self.log("[控制] 无新鲜遥测，先确认下位机在发 vofa 行")
            self.link.send(Command.STATUS)
            return
        if state.calibrated is False:
            self.log("[控制] 压力校准未通过，需在空气中重新上电（CA）")
            return
        if state.fault:
            self.log(f"[控制] 已锁存故障（{protocol.describe_stop_reason(state.stop_reason)}），"
                     "须断电重启")
            return
        if state.ready is False:
            # EDIAG1 的 NR 掩码直接给出是哪个 READY 条件没过，
            # 比让操作者对着 STAT 猜要省事
            self.log("[控制] 反馈未就绪，ON 会被拒绝，正在查 EDIAG")
            self.link.send(Command.STATUS)
            self.link.send(Command.EXTENDED_DIAGNOSTICS)
            return

        # 互锁：水压计没有明确显示在水里之前，只记下意图，不发 ON。
        depth = state.value_or_none("depth_m")
        if self.arm_depth_m > 0.0:
            if depth is None:
                self.log("[控制] 深度无效，无法确认是否入水，暂不解锁")
                return
            if depth < self.arm_depth_m:
                self.arm_pending = True
                self.log(f"[控制] 已就绪，等待入水：当前 {depth:.2f} m，"
                         f"需 ≥ {self.arm_depth_m:.2f} m。入水即自动发 ON，"
                         "再按一次 Start 可取消")
                return
        self._send_arm(now)

    def _send_arm(self, now: float) -> None:
        """真正发 ON。互锁通过、或互锁关闭时才走到这里。

        这里要把 toggle_arm 的检查重做一遍：从按下 Start 到实际入水可能
        过了很久，期间完全可能已经掉遥测或锁存故障。
        """
        state = self.telemetry
        reasons = []
        if not self.telemetry_fresh(now):
            reasons.append("遥测不新鲜")
        if state.fault:
            reasons.append("已锁存故障")
        if state.ready is False:
            # 说清楚是哪一项融合条件没成立，而不是笼统一句"复查未通过"
            missing = protocol.describe_missing_flags(state.fusion_flags_stat)
            reasons.append("READY 未成立" +
                           ("（缺：" + "、".join(missing) + "）" if missing else ""))
        if reasons:
            self.arm_pending = False
            self.log("[控制] 入水复查未通过（" + "；".join(reasons) +
                     "），已放弃解锁，请重新按 Start")
            tilt = protocol.pressure_tilt(state.pressure_legacy)
            if tilt is not None and tilt["beyond_geometry"]:
                self.log("[控制] 四路压差超出安装基线，机器多半是立着入水的；"
                         "请平着放进水里")
            self.link.send(Command.STATUS)
            return

        # 用当前实测值初始化设定值：深度保持原地，航向保持当前朝向
        depth = state.value_or_none("depth_m")
        yaw = state.value_or_none("yaw_deg")
        self.target_depth_m = max(0.0, depth if depth is not None else 0.0)
        self.target_yaw_deg = yaw if yaw is not None else 0.0
        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.setpoints_valid = True

        self.link.parser.state.armed_reported = False
        if self.link.send(Command.ARM):
            self.arm_pending = False
            self.arm_intent = True
            # 深度闭环的开关状态在固件侧是持久的，但 ON 之后重申一次，
            # 免得中途重连或下位机重启后两边不一致。
            self.toggle_depth_hold(self.depth_hold)
            self.log(f"[控制] 已发 ON，保持深度 {self.target_depth_m:.2f} m，"
                     f"航向 {self.target_yaw_deg:.1f}°")

    def disarm(self, reason: str) -> None:
        self.arm_intent = False
        self.arm_pending = False
        self.setpoints_valid = False
        self.link.send(Command.DISARM)
        self._last_motion = Command.HOLD
        self.log(f"[控制] 已发 OFF（{reason}）")

    def emergency_stop(self, reason: str = "操作者") -> None:
        """急停：连发 OFF，并掐掉心跳让固件超时锁存作为兜底。"""
        self.estop_latched = True
        self.arm_intent = False
        self.arm_pending = False
        self.setpoints_valid = False
        for _ in range(3):
            self.link.send(Command.DISARM)
        self.link.suppress_heartbeat(0.8)   # > 500 ms，确保超时停止生效
        self.log(f"[急停] {reason}：已发 OFF 并暂停心跳。"
                 "仍须用物理开关切断动力确认。")

    def check_downlink(self, now: float) -> None:
        """上位机侧看门狗：解锁后失去遥测即自主停机。

        固件察觉不到这种失效。串口 TX/RX 是两根独立的线，
        下行断掉（或本机接收线程卡死）时，心跳仍在正常发出、
        固件的反馈自检也全部通过，于是潜器会带着最后的目标继续运行，
        而操作者已经看不到任何实测值。这种"盲飞"必须由上位机自己终止。

        停心跳是这里的关键动作：它让固件在 500 ms 内锁存停止，
        不依赖 OFF 能否通过那条可能已经半死的链路送达。
        """
        if not self.arm_intent or self.estop_latched:
            return
        age = self.telemetry.vofa_age_s(now)
        if age is None or age >= DOWNLINK_TIMEOUT_S:
            shown = "从未收到" if age is None else f"{age:.1f} s 无更新"
            self.emergency_stop(f"遥测中断（{shown}），已失去状态可见性")

    def toggle_depth_hold(self, enable: bool | None = None) -> None:
        """开关下位机的深度闭环。

        关掉之后垂直推进器只做姿态稳定，潜器停在浮力配平决定的深度上，
        不再往解锁那一刻记下的目标深度跑。
        """
        want = (not self.depth_hold) if enable is None else enable
        command = (Command.DEPTH_HOLD_ON if want else Command.DEPTH_HOLD_OFF)
        if not self.link.send(command):
            self.log("[控制] 深度闭环切换失败：命令没发出去")
            return
        self.depth_hold = want
        if want and self.arm_intent and self.setpoints_valid:
            # 固件里的目标深度还停在解锁那一刻的值。中途打开深度保持时若不
            # 重设，潜器会一头冲回那个旧目标（通常是入水互锁的 0.15 m）。
            # 就地保持才是操作者按下这个键时期待的行为。
            here = self.telemetry.value_or_none("depth_m")
            if here is not None:
                self.target_depth_m = min(
                    protocol.FSET_DEPTH_MAX_M,
                    max(protocol.FSET_DEPTH_MIN_M, here))
                self._fset_dirty = True
                self.log(f"[控制] 深度闭环 开，就地保持 {self.target_depth_m:.2f} m")
                return
            self.log("[控制] 深度闭环 开，但深度无效，目标深度未更新")
            return
        self.log(f"[控制] 深度闭环 {'开' if want else '关'}"
                 + ("" if want else "（垂直推进器只稳姿态，深浅由浮力决定）"))

    def reset_estop(self) -> None:
        if not self.estop_latched:
            return
        self.estop_latched = False
        self.log("[控制] 急停已复位。若下位机已锁存故障，仍须断电重启。")

    # ---- 逐帧处理 ----------------------------------------------------------
    def update(self, snapshot: gamepad.InputSnapshot, now: float) -> None:
        # 边沿动作
        if "estop" in snapshot.pressed:
            self.emergency_stop()
        if "arm_toggle" in snapshot.pressed:
            self.toggle_arm(now)
        if "yaw_hold_toggle" in snapshot.pressed:
            self._toggle_yaw_hold()
        if "servo_left_center" in snapshot.pressed:
            self.servo_us[0] = self.servo_us[1] = float(protocol.SERVO_MID_US)
            self._servo_dirty = True
        if "servo_right_center" in snapshot.pressed:
            self.servo_us[2] = self.servo_us[3] = float(protocol.SERVO_MID_US)
            self._servo_dirty = True

        # 上位机侧看门狗：先于一切控制输出判断，失去遥测就不该继续驾驶
        self.check_downlink(now)
        self._check_arm_interlock(now)
        self._watch_unexpected_stop(now)
        self._update_yaw_capture(now)

        # 手柄掉线即停止运动，不保持最后状态
        if not snapshot.connected and self.arm_intent:
            self._send_motion(Command.HOLD, now, force=True)

        dt = 1.0 / CONTROL_HZ
        self._update_translation(snapshot, now)
        self._update_depth(snapshot, now, dt)
        self._update_yaw_setpoint(snapshot, dt)
        self._update_servos(snapshot, dt)

        self._flush(now)
        self._poll_status(now)

    def _watch_unexpected_stop(self, now: float) -> None:
        """下位机自己停下来的那一刻，立刻抓一次诊断。

        NR 掩码在固件里每个控制周期都会被覆盖，等操作者反应过来再手敲
        EDIAG 早就冲掉了；固件侧锁存了停机那一刻的快照，这里负责把它取回来。
        """
        stopped = self.telemetry.stopped
        if stopped is None or not self.telemetry_fresh(now):
            return
        running = not stopped
        if self._was_running and not running:
            # 只有"我们以为它该在跑"时才值得报警；主动 OFF 不算
            if self.arm_intent:
                self.log("[控制] 下位机自行停机，正在抓取停机诊断")
                self.link.send(Command.STATUS)
                self.link.send(Command.EXTENDED_DIAGNOSTICS)
        self._was_running = running

    def _check_arm_interlock(self, now: float) -> None:
        """待入水状态下，水压计一旦确认入水就自动发 ON。"""
        if not self.arm_pending:
            return
        if self.estop_latched or not self.link.connected:
            self.arm_pending = False
            self.log("[控制] 待入水解锁已取消（急停或链路断开）")
            return
        if not self.telemetry_fresh(now):
            return
        depth = self.telemetry.value_or_none("depth_m")
        if depth is not None and depth >= self.arm_depth_m:
            # 只说"确认入水"；ON 到底发没发由 _send_arm 自己报，
            # 否则复查不通过时日志会先说"发送 ON"再说"已放弃"，自相矛盾。
            self.log(f"[控制] 已确认入水（{depth:.2f} m），开始复查")
            self._send_arm(now)

    def _toggle_yaw_hold(self) -> None:
        if self.yaw_hold or self._yaw_capturing:
            self._yaw_capturing = False
            self._yaw_samples.clear()
            self.yaw_hold = False
            self.link.send(Command.YAW_HOLD_OFF)
            self.log("[控制] 偏航闭环 关")
            return

        # ACL 打开后，FusedStateUsable 会把"航向已观测"列为必需项；
        # 磁力计若正被大量拒绝，一开就会让 READY 掉、整机停机。宁可先不开。
        flags = self.telemetry.fusion_flags
        if flags is not None and not flags & (1 << 2):
            self.log("[控制] 航向尚未被观测到，现在开偏航闭环会让 READY 掉；"
                     "先让磁力计稳定（远离金属、等几秒）再试")
            return
        if not self.telemetry_fresh(time.monotonic()):
            self.log("[控制] 无新鲜遥测，暂不开偏航闭环")
            return

        self._yaw_samples.clear()
        self._yaw_capture_seq = None
        self._yaw_capturing = True
        self.log(f"[控制] 正在取 {YAW_CAPTURE_SAMPLES} 帧航向求基准，"
                 "这一秒内请让潜器保持不动")

    def _update_yaw_capture(self, now: float) -> None:
        """连续取 N 帧实测航向，圆周平均后作为偏航闭环的目标。

        必须用圆周平均：航向在 ±180 度处回绕，直接算术平均会把
        -179 和 +179 平均成 0，得到一个正好相反的基准。
        """
        if not self._yaw_capturing:
            return
        if not self.telemetry_fresh(now):
            return
        state = self.telemetry
        if state.sequence is None or state.sequence == self._yaw_capture_seq:
            return                       # 同一帧不重复采样
        self._yaw_capture_seq = state.sequence
        yaw = state.value_or_none("yaw_deg")
        if yaw is None:
            return
        self._yaw_samples.append(yaw)
        if len(self._yaw_samples) < YAW_CAPTURE_SAMPLES:
            return

        radians = [math.radians(v) for v in self._yaw_samples]
        mean = math.degrees(math.atan2(
            sum(math.sin(v) for v in radians) / len(radians),
            sum(math.cos(v) for v in radians) / len(radians)))
        spread = max(self._yaw_samples) - min(self._yaw_samples)
        self._yaw_capturing = False
        self._yaw_samples.clear()

        if not self.link.send(Command.YAW_HOLD_ON):
            self.log("[控制] 偏航闭环开启失败：命令没发出去")
            return
        self.yaw_hold = True
        self.target_yaw_deg = max(-protocol.FSET_YAW_LIMIT_DEG,
                                  min(protocol.FSET_YAW_LIMIT_DEG, mean))
        self._fset_dirty = True          # 随即用 FSET 把这个基准下发给固件
        self.log(f"[控制] 偏航闭环 开，基准航向 {self.target_yaw_deg:.1f}°"
                 f"（{YAW_CAPTURE_SAMPLES} 帧平均，跨度 {spread:.1f}°）")

    def _update_translation(self, snapshot: gamepad.InputSnapshot,
                            now: float) -> None:
        """水平移动仍是离散状态命令，固件只有这一种水平控制方式。"""
        command = Command.HOLD
        if snapshot.forward:
            command = Command.FORWARD
        elif snapshot.backward:
            command = Command.BACKWARD
        elif snapshot.left:
            command = Command.LEFT
        elif snapshot.right:
            command = Command.RIGHT
        elif not self.yaw_hold and snapshot.yaw_cw:
            command = Command.CLOCKWISE
        elif not self.yaw_hold and snapshot.yaw_ccw:
            command = Command.ANTICLOCKWISE
        self._send_motion(command, now)

    def _send_motion(self, command: str, now: float,
                     force: bool = False) -> None:
        """状态变化时立即发，之后按 MOTION_REFRESH_S 周期重发。

        固件会锁存运动状态，所以不需要 30 Hz 刷屏；
        但周期重发能在丢包后自愈，也顺带满足"收包不超过 500 ms"。
        """
        changed = command != self._last_motion
        stale = now - self._last_motion_sent >= MOTION_REFRESH_S
        if not (force or changed or stale):
            return
        if not self.arm_intent and command != Command.HOLD:
            return           # 未解锁时不发运动命令，固件也会拒绝
        if self.link.send(command):
            self._last_motion = command
            self._last_motion_sent = now

    def _update_depth(self, snapshot: gamepad.InputSnapshot,
                      now: float, dt: float) -> None:
        if self.legacy_depth:
            # 兼容模式：沿用 UP/DN 每次 1 cm，按住即连续步进，
            # 不再像老程序那样只在按下瞬间发一次。
            # UP/DN 是增量而非锁存状态，因此不走 _send_motion 的
            # 状态机，否则会和水平运动状态互相覆盖。
            if not self.arm_intent:
                return
            if now - self._last_depth_step < 1.0 / DEPTH_STEP_HZ:
                return
            command = None
            if snapshot.ascend:
                command = Command.DEPTH_UP
            elif snapshot.descend:
                command = Command.DEPTH_DOWN
            if command and self.link.send(command):
                self._last_depth_step = now
            return

        if not self.setpoints_valid:
            return
        delta = 0.0
        if snapshot.ascend:
            delta -= DEPTH_RATE_M_S * dt
        if snapshot.descend:
            delta += DEPTH_RATE_M_S * dt
        if delta:
            self.target_depth_m = min(
                protocol.FSET_DEPTH_MAX_M,
                max(protocol.FSET_DEPTH_MIN_M, self.target_depth_m + delta))
            self._fset_dirty = True

    def _update_yaw_setpoint(self, snapshot: gamepad.InputSnapshot,
                             dt: float) -> None:
        """ACL 开启时，X/Y 改的是目标航向而不是发离散旋转命令。"""
        if not (self.yaw_hold and self.setpoints_valid):
            return
        delta = 0.0
        if snapshot.yaw_cw:
            delta += YAW_RATE_DEG_S * dt
        if snapshot.yaw_ccw:
            delta -= YAW_RATE_DEG_S * dt
        if delta:
            value = self.target_yaw_deg + delta
            while value > 180.0:
                value -= 360.0
            while value < -180.0:
                value += 360.0
            self.target_yaw_deg = value
            self._fset_dirty = True

    def _update_servos(self, snapshot: gamepad.InputSnapshot,
                       dt: float) -> None:
        """四路舵机积分式控制，范围与限幅四路一致。"""
        scale = 0.4 if snapshot.fine else 1.0
        step = SERVO_STEP_US_S * dt * scale
        axis_order = ("left_vertical", "left_horizontal",
                      "right_vertical", "right_horizontal")
        for index, name in enumerate(axis_order):
            value = snapshot.axes.get(name, 0.0)
            if not value:
                continue
            updated = self.servo_us[index] + value * step
            self.servo_us[index] = min(
                float(protocol.SERVO_MAX_US),
                max(float(protocol.SERVO_MIN_US), updated))
            self._servo_dirty = True

    # ---- 发送节流 ----------------------------------------------------------
    def _flush(self, now: float) -> None:
        if self._servo_dirty and now - self._last_servo_sent >= 1.0 / SERVO_SEND_HZ:
            pulses = tuple(int(round(v)) for v in self.servo_us)
            try:
                command = protocol.servo_command(pulses)
            except ValueError as exc:
                self.log(f"[控制] 舵机命令无效: {exc}")
                self._servo_dirty = False
                return
            if self.link.send(command):
                self._last_servo_sent = now
                self._servo_dirty = False

        if (self._fset_dirty and self.arm_intent and self.setpoints_valid
                and now - self._last_fset_sent >= 1.0 / FSET_SEND_HZ):
            try:
                command = protocol.fused_target_command(
                    self.target_depth_m, self.target_roll_deg,
                    self.target_pitch_deg, self.target_yaw_deg)
            except ValueError as exc:
                self.log(f"[控制] 目标值无效: {exc}")
                self._fset_dirty = False
                return
            if self.link.send(command):
                self._last_fset_sent = now
                self._fset_dirty = False

    def _poll_status(self, now: float) -> None:
        if now - self._last_stat >= STATUS_POLL_S:
            self._last_stat = now
            self.link.send(Command.STATUS)
        if now - self._last_diag >= DIAG_POLL_S:
            self._last_diag = now
            self.link.send(Command.DIAGNOSTICS)


def list_ports() -> None:
    from serial.tools import list_ports as tools
    found = list(tools.comports())
    if not found:
        print("未发现串口设备")
        return
    print("可用串口：")
    for item in found:
        print(f"  {item.device}  {item.description}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="fins_v3 手柄遥控上位机")
    parser.add_argument("--port", help="下位机串口，例如 COM11")
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引")
    parser.add_argument("--no-camera", action="store_true", help="不开摄像头")
    parser.add_argument("--mapping", help="手柄映射 JSON 覆盖文件")
    parser.add_argument("--no-depth-hold", action="store_true",
                        help="解锁后立刻关闭深度闭环，垂直推进器只稳姿态，"
                             "深浅交给浮力配平；运行中按 D 键可随时切换")
    parser.add_argument("--arm-depth", type=float, default=ARM_DEPTH_M,
                        help="按 Start 后要等水压计读到多深才真正发 ON，"
                             "单位米；0 表示关闭该互锁（台架断动力测试用）")
    parser.add_argument("--legacy-depth", action="store_true",
                        help="深度用 UP/DN 步进，而不是 FSET 绝对目标")
    parser.add_argument("--list-ports", action="store_true",
                        help="列出可用串口后退出")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.list_ports:
        list_ports()
        return 0
    if not args.port:
        print("必须用 --port 指定串口。用 --list-ports 查看可用端口。",
              file=sys.stderr)
        return 2

    pygame.init()
    pygame.display.set_caption("FinsROV V3.3 手柄遥控")
    screen = pygame.display.set_mode(hud_module.WINDOW_SIZE)
    clock = pygame.time.Clock()

    console = hud_module.Console()
    link = SerialLink(args.port, log=console.write)
    controller = Controller(link, legacy_depth=args.legacy_depth,
                           arm_depth_m=args.arm_depth, log=console.write)
    controller.depth_hold = not args.no_depth_hold
    reader = gamepad.GamepadReader(gamepad.load_mapping(args.mapping),
                                  log=console.write)
    view = hud_module.Hud()

    stream = None
    if not args.no_camera:
        from camera import CameraStream
        stream = CameraStream(args.camera, log=console.write)

    if not link.open():
        console.write("[链路] 首次连接失败，后台会持续重试")
    link.start()
    if stream is not None:
        stream.start()

    if args.arm_depth > 0:
        console.write(f"已启动。Start 请求解锁（入水 ≥ {args.arm_depth:.2f} m "
                      "后才真正发 ON）/ Back 急停 / SPACE 键盘急停 / ESC 退出")
    else:
        console.write("已启动。入水互锁已关闭。"
                      "Start 解锁 / Back 急停 / SPACE 键盘急停 / ESC 退出")

    try:
        running = True
        while running:
            now = time.monotonic()
            pressed: set[str] = set()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        controller.emergency_stop()
                    elif event.key == pygame.K_r:
                        controller.reset_estop()
                    elif event.key == pygame.K_d:
                        controller.toggle_depth_hold()
                elif event.type == pygame.JOYBUTTONDOWN:
                    name = reader.button_name(event.button)
                    if name:
                        pressed.add(name)
                    else:
                        console.write(f"[手柄] 未映射按键 {event.button}")
                else:
                    reader.handle_event(event)

            snapshot = reader.sample(pressed)
            controller.update(snapshot, now)

            frame = None
            frame_age = None
            if stream is not None:
                frame, frame_age = stream.latest()

            view.draw(screen, controller, link, snapshot, console,
                      frame, frame_age, now)
            pygame.display.flip()
            clock.tick(CONTROL_HZ)
    except KeyboardInterrupt:
        console.write("收到中断，正在停机")
    finally:
        # 无论如何都要发 OFF 并释放资源。老程序在异常路径下
        # 既不停机也不关串口，潜器会保持最后的运动状态。
        try:
            link.close(send_disarm=True)
        finally:
            if stream is not None:
                stream.close()
            pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
