"""fins_v3 下位机串口协议层。

对应固件：Lower_Level_Controller，UART6 / 115200 / 8E1 / CRLF。
命令定义见 Lower_Level_Controller/docs/CONFIG_AND_INTERFACES.md。

本模块只负责"协议是什么"，不负责"什么时候发"，后者在 link.py。
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

# ---- 链路参数，必须与固件一致 ----------------------------------------------
BAUDRATE = 115200
BYTESIZE = 8
PARITY = "E"
STOPBITS = 1

LINE_END = "\r\n"          # 固件要求 CR 或 LF 结束；推荐 CRLF
MAX_LINE_CHARS = 99        # CommandLineAssembler line_[100]，含 NUL
LINE_ASSEMBLY_TIMEOUT_S = 0.25   # 整行须在 250 ms 内送完

# 固件 LC_COMMAND_TIMEOUT_US = 500000：超过 500 ms 无 HB 即锁存停止
COMMAND_TIMEOUT_S = 0.5
HEARTBEAT_PERIOD_S = 0.1   # 文档要求约每 100 ms 一次

# 固件 LC_VOFA_PERIOD_US = 20000，即默认 50 Hz 上报（旧版文档写的 10 Hz 已过时）
VOFA_PERIOD_S = 0.02

# ---- 舵机与推进器范围，取自 CONFIG_AND_INTERFACES.md ------------------------
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_MID_US = 1500
SERVO_COUNT = 4

THRUSTER_MIN_US = 1000
THRUSTER_MAX_US = 2000
# V3.3 实物标定：中位 1550 us（LC_THRUSTER_NEUTRAL_US），死区 1510..1610。
# 不是 1500，也不是早期版本的 1610。
THRUSTER_NEUTRAL_US = 1550
THRUSTER_DEADZONE_LOW_US = 1510
THRUSTER_DEADZONE_HIGH_US = 1610
THRUSTER_COUNT = 8

# FSET 取值范围，见 Propeller_I2C::SetFusedTarget
FSET_DEPTH_MIN_M = 0.0
FSET_DEPTH_MAX_M = 100.0
FSET_ROLL_LIMIT_DEG = 30.0
FSET_PITCH_LIMIT_DEG = 30.0
FSET_YAW_LIMIT_DEG = 180.0


def servo_pwm_to_angle_deg(pulse_us: int) -> float:
    """500..2500 us 线性映射到 -90..+90 度。"""
    return 180.0 * (pulse_us - SERVO_MIN_US) / 2000.0 - 90.0


def firmware_servo_echo(pulse_us: tuple[int, int, int, int]) -> tuple[int, ...]:
    """返回固件实际写给舵机的脉宽。

    Servo_I2C::data_extract 对第三路做 ``3000 - value`` 反转，
    所以 HUD 上必须区分"我发的值"和"舵机收到的值"，否则操作者会被误导。
    """
    out = list(pulse_us)
    out[2] = 3000 - out[2]
    return tuple(out)


# ---- 命令构造 --------------------------------------------------------------
class Command:
    """无参命令常量。值本身不含行尾，行尾由 encode() 统一添加。"""

    HEARTBEAT = "HB"
    ARM = "ON"
    DISARM = "OFF"
    STATUS = "STAT"
    DIAGNOSTICS = "DIAG"
    #: 扩展诊断，回复 EDIAG1..EDIAG7 共七行。
    #: EDIAG1 的 NR 掩码直接回答"哪个 READY 条件没过"。
    EXTENDED_DIAGNOSTICS = "EDIAG"
    RECALIBRATE = "CA"

    FORWARD = "W"
    BACKWARD = "S"
    LEFT = "A"
    RIGHT = "D"
    CLOCKWISE = "E"
    ANTICLOCKWISE = "Q"
    HOLD = "Z"            # 停止水平运动

    DEPTH_UP = "UP"       # 目标深度 -1 cm，最低 0
    DEPTH_DOWN = "DN"     # 目标深度 +1 cm

    YAW_HOLD_ON = "ACL:ON"
    YAW_HOLD_OFF = "ACL:OF"

    #: 深度闭环开关。关掉后垂直推进器只稳姿态，深浅交给浮力配平。
    DEPTH_HOLD_ON = "DEP:ON"
    DEPTH_HOLD_OFF = "DEP:OFF"

    TELEMETRY_ON = "VOFA:ON"
    TELEMETRY_OFF = "VOFA:OFF"


#: 会改变运动状态的命令。固件要求它们属于当前停止代次且收包不超过 500 ms。
MOTION_COMMANDS = frozenset({
    Command.FORWARD, Command.BACKWARD, Command.LEFT, Command.RIGHT,
    Command.CLOCKWISE, Command.ANTICLOCKWISE, Command.HOLD,
    Command.DEPTH_UP, Command.DEPTH_DOWN,
})


def encode(command: str) -> bytes:
    """把一条命令编码成一次完整写入的字节串。

    固件按行解析，且整行必须在 250 ms 内送完，所以调用方必须
    把返回值一次性 write 出去，不能分段。
    """
    if not command or "\r" in command or "\n" in command:
        raise ValueError(f"命令不得为空或自带行尾: {command!r}")
    if len(command) > MAX_LINE_CHARS:
        raise ValueError(f"命令超过 {MAX_LINE_CHARS} 字符: {command!r}")
    if any(ord(ch) < 32 or ord(ch) > 126 for ch in command):
        raise ValueError(f"命令含非法字节，固件会丢弃到行尾: {command!r}")
    return (command + LINE_END).encode("ascii")


def servo_command(pulse_us: tuple[int, int, int, int]) -> str:
    """MOT:a,b,c,d —— 四路舵机脉宽，500..2500。"""
    if len(pulse_us) != SERVO_COUNT:
        raise ValueError(f"需要 {SERVO_COUNT} 路舵机值，收到 {len(pulse_us)}")
    for value in pulse_us:
        if not SERVO_MIN_US <= value <= SERVO_MAX_US:
            raise ValueError(f"舵机脉宽越界 {value}，须在 "
                             f"{SERVO_MIN_US}..{SERVO_MAX_US}")
    return "MOT:" + ",".join(str(int(v)) for v in pulse_us)


def fused_target_command(depth_m: float, roll_deg: float,
                         pitch_deg: float, yaw_deg: float) -> str:
    """FSET:深度m,横滚deg,俯仰deg,偏航deg —— 四项一次提交。

    固件用 strtof 解析并做范围检查，任一项不合格则整条拒绝，
    所以这里先自查，避免把必然被拒的包塞进只有 8 格的接收队列。
    """
    if not FSET_DEPTH_MIN_M <= depth_m <= FSET_DEPTH_MAX_M:
        raise ValueError(f"目标深度越界 {depth_m}")
    if abs(roll_deg) > FSET_ROLL_LIMIT_DEG:
        raise ValueError(f"目标横滚越界 {roll_deg}")
    if abs(pitch_deg) > FSET_PITCH_LIMIT_DEG:
        raise ValueError(f"目标俯仰越界 {pitch_deg}")
    if abs(yaw_deg) > FSET_YAW_LIMIT_DEG:
        raise ValueError(f"目标偏航越界 {yaw_deg}")
    return f"FSET:{depth_m:.3f},{roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f}"


def manual_thruster_command(pulse_us: tuple[int, ...]) -> str:
    """TES:v0..v7 —— 完整 8 路手动 PWM，仅在闭环关闭时使用。"""
    if len(pulse_us) != THRUSTER_COUNT:
        raise ValueError(f"需要 {THRUSTER_COUNT} 路推进器值")
    for value in pulse_us:
        if not THRUSTER_MIN_US <= value <= THRUSTER_MAX_US:
            raise ValueError(f"推进器脉宽越界 {value}")
    return "TES:" + ",".join(str(int(v)) for v in pulse_us)


# ---- 遥测解析 --------------------------------------------------------------
#: 固件用 -9999 表示缺测/过期，绝不是真实测量值。
MISSING = -9999.0

#: VOFA FireWater 16 通道顺序，见 VofaTelemetry.h。
#:
#: 注意：通道顺序在 a69fbae 被改过，压力四路移到了最前面，
#: 目的是让旧的四数字 VOFA 视图保持不变。若继续按旧顺序解析，
#: 会把压力值当成姿态角显示，且不会报错 —— 必须按本顺序。
VOFA_FIELDS = (
    "p0_legacy", "p1_legacy", "p2_legacy", "p3_legacy",
    "roll_deg", "pitch_deg", "yaw_deg", "depth_m",
    "stopped", "ready", "calibrated", "raw_mask",
    "fault", "fused_mask", "error_code", "sequence",
)


def decode_error_code(code: int | None) -> dict[str, int]:
    """拆解 VOFA 的 ERROR_CODE（原 FLAGS）。

    位分配见 ControlLoopTask.cpp 的 diagnostic_code：
    低 8 位为最后停止原因，8..15 位为累计 I2C2 错误，16..31 位为 ESKF flags。
    """
    if code is None:
        return {}
    return {
        "stop_reason": code & 0xFF,
        "i2c2_errors": (code >> 8) & 0xFF,
        "fusion_flags": (code >> 16) & 0xFFFF,
    }


#: ESKF flags（STAT 的 FLAGS 字段 / ERROR_CODE 的 16..31 位）。
#: 见固件 SensorFusionData.h 的 FusionFlags。全部满足时为 31。
FUSION_FLAG_BITS = {
    0: "姿态有效",
    1: "深度有效",
    2: "航向已观测",
    3: "配置就绪",
    4: "陀螺连续",
}


def describe_missing_flags(flags: int | None) -> list[str]:
    """列出没有置位的 ESKF flags —— 直接回答"READY 为什么灭"。"""
    if flags is None:
        return []
    return [text for bit, text in FUSION_FLAG_BITS.items()
            if not flags & (1 << bit)]


# ---- 压力阵列的几何 --------------------------------------------------------
#: 四个水压计的安装位置，取自固件 FusionConfiguration.cpp（单位 m，按 TCA 0..3）。
#: TCA0=前左, TCA1=后左, TCA2=后右, TCA3=前右。
PRESSURE_POSITION_M = ((0.1247, -0.0691), (-0.1247, -0.0691),
                       (-0.1247, 0.0691), (0.1247, 0.0691))
#: 前后基线与左右基线（m）。压差能表达的倾角以此为上限：
#: 前后高差最大只能等于前后基线，超出就是任何姿态都解释不了的观测。
PITCH_BASELINE_M = 0.2494
ROLL_BASELINE_M = 0.1382
#: legacy 压力裸值（约 hPa）换算成米水头：1 hPa = 100 Pa，rho*g = 9810。
LEGACY_HPA_TO_M = 100.0 / 9810.0


def pressure_tilt(legacy: tuple[float, ...]) -> dict | None:
    """由四路压力算出它们隐含的倾角，以及是否已超出几何极限。

    ESKF 一旦入水就用四路压差做姿态观测；压差超过基线长度时
    没有任何姿态能解释它，NIS 必然拒绝，而固件在拒绝姿态的同时
    会把整帧的深度一起作废（AttitudeEskf.cpp 的 pressure_rank_ 分支），
    于是 FusionDepthValid 灭、READY 灭。这里把这件事直接显示出来。
    """
    if len(legacy) != 4 or any(v == MISSING for v in legacy):
        return None
    depth = [v * LEGACY_HPA_TO_M for v in legacy]
    front = (depth[0] + depth[3]) / 2.0   # TCA0/3 在前（x 为正侧）
    back = (depth[1] + depth[2]) / 2.0
    left = (depth[0] + depth[1]) / 2.0    # TCA0/1 在左（y 为负侧）
    right = (depth[2] + depth[3]) / 2.0
    # 符号按 FRD，与下位机 ESKF 的欧拉角一致，两行才能直接比对：
    #   俯仰正 = 抬头 = 机头更浅，所以取 (后 - 前)
    #   横滚正 = 右舷下 = 右侧更深，所以取 (右 - 左)
    pitch_ratio = (back - front) / PITCH_BASELINE_M
    roll_ratio = (right - left) / ROLL_BASELINE_M

    def angle(ratio: float) -> float | None:
        if abs(ratio) > 1.0:
            return None          # 超出几何极限，无解
        import math
        return math.degrees(math.asin(ratio))

    return {
        "pitch_ratio": pitch_ratio,
        "roll_ratio": roll_ratio,
        "pitch_deg": angle(pitch_ratio),
        "roll_deg": angle(roll_ratio),
        "front_minus_back_m": front - back,
        "left_minus_right_m": left - right,
        "back_minus_front_m": back - front,
        "beyond_geometry": abs(pitch_ratio) > 1.0 or abs(roll_ratio) > 1.0,
        # 接近极限时 NIS 也已经很容易拒绝了，不必等到真正 >1
        "near_limit": abs(pitch_ratio) > 0.95 or abs(roll_ratio) > 0.95,
    }


#: EDIAG1 的 NR 掩码：哪一个 READY 条件在最近一个周期未通过（置位=失败）。
#: 见 ControlLoopTask.cpp 的 not_ready_mask 注释。
NOT_READY_BITS = {
    0: "压力校准未通过",
    1: "已锁存故障",
    2: "IMU 温控故障",
    3: "压力总线失败",
    4: "IMU 等待超时",
    5: "IMU 帧无效",
    6: "融合状态不可用",
}


#: last_pressure_reject_bits 的位义，见固件 AttitudeEskf::ProcessPressure。
PRESSURE_REJECT_BITS = {
    0: "量程越界",
    1: "采样偏斜超限",   # I2C2 读一帧被拉长，多半是推进器一转就来的总线干扰
    2: "跳变超限",       # 潜器动得比 jump_margin + max_rate 快
}


def describe_pressure_reject(bits: int | None) -> list[str]:
    if not bits:
        return []
    return [t for b, t in PRESSURE_REJECT_BITS.items() if bits & (1 << b)]


def describe_not_ready(mask: int | None) -> list[str]:
    """把 EDIAG1 的 NR 掩码翻译成具体原因，回答"为什么 ON 被拒"。"""
    if not mask:
        return []
    return [text for bit, text in NOT_READY_BITS.items() if mask & (1 << bit)]

#: last_stop_reason / WHY 字段释义，见 README.md
STOP_REASONS = {
    0: "无记录",
    1: "心跳超时",
    2: "控制卡住",
    3: "总线回复超时",
    4: "断言失败",
    5: "输出写失败",
    6: "反馈无效",
    7: "周期超时",
    8: "控制计算无效",
    9: "收到 OFF",
    10: "请求重新校准",
    11: "温控故障",
}


def describe_stop_reason(code: int | None) -> str:
    if code is None:
        return "未知"
    return STOP_REASONS.get(code, f"未定义({code})")


@dataclass
class Telemetry:
    """下位机上报的状态快照。

    字段为 None 表示"从未收到过"，为 MISSING 表示"固件明确报告缺测"。
    这两者与"零"必须区分开，老程序把写死的 300 当深度显示就是反面例子。
    """

    # VOFA 数值通道
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    depth_m: float | None = None

    #: 四路 legacy 压力裸值。**不是米，也不是 Pa** —— 固件直接发
    #: 旧 data_pressure[]，空气中约 1000~1015，不经换算或 CAL 门控。
    pressure_legacy: tuple[float, ...] = ()

    # VOFA 状态通道
    stopped: bool | None = None
    ready: bool | None = None
    calibrated: bool | None = None
    fault: bool | None = None
    raw_mask: int | None = None
    fused_mask: int | None = None
    error_code: int | None = None
    sequence: int | None = None

    # 由 error_code 拆出，见 decode_error_code()
    i2c2_errors: int | None = None
    fusion_flags: int | None = None

    # EDIAG1：READY 为何不成立
    not_ready_mask: int | None = None

    # STAT 回复
    stat_stopped: bool | None = None
    stat_ready: bool | None = None
    stat_calibrated: bool | None = None
    stat_fault: bool | None = None
    stop_reason: int | None = None
    cal_failed_channel: int | None = None
    cal_samples: int | None = None
    #: 上电标定记下的"水面"绝对气压均值(Pa)。空气中应当接近 101325；
    #: 明显高出说明标定时传感器泡在水里，之后整条深度链路都带着这个偏差。
    cal_surface_pa: int | None = None
    cal_channel_spread_pa: int | None = None   # 四路零点的最大-最小(Pa)，空气中应接近 0
    cal_state: int | None = None               # 2=成功，3=失败，4=四路零点差太多
    fusion_flags_stat: int | None = None       # STAT 的 FLAGS 字段，见 FUSION_FLAG_BITS
    #: 停机那一刻的 not_ready_mask（STAT 的 NRL / EDIAG9）。位义同 NOT_READY_BITS。
    #: not_ready_mask 每个控制周期都会被覆盖，只有这个快照能回答"当时是哪一项没成立"。
    stop_not_ready_mask: int | None = None
    not_ready_counts: tuple[int, ...] = ()     # 七个 READY 条件各自导致停机判定的累计次数

    # EDIAG8：深度失效的原因分项
    pressure_reject_range: int | None = None
    pressure_reject_skew: int | None = None
    pressure_reject_jump: int | None = None
    pressure_reject_empty: int | None = None   # 四路全被拒
    pressure_reject_nis: int | None = None     # 压差姿态被拒，整帧深度一起作废
    last_pressure_reject_bits: int | None = None

    # DIAG 回复
    cycle_us: int | None = None
    max_cycle_us: int | None = None
    deadline_misses: int | None = None
    rx_drops: int | None = None
    i2c_errors: int | None = None
    temperature_c: float | None = None
    heater_fault: int | None = None

    # 链路与文本
    armed_reported: bool = False
    last_reply: str = ""
    rejections: list[str] = field(default_factory=list)
    last_vofa_monotonic: float = 0.0
    last_rx_monotonic: float = 0.0

    def vofa_age_s(self, now: float) -> float | None:
        if not self.last_vofa_monotonic:
            return None
        return now - self.last_vofa_monotonic

    def value_or_none(self, name: str) -> float | None:
        """取数值通道，把固件的 MISSING 归一成 None。"""
        value = getattr(self, name)
        if value is None or value == MISSING:
            return None
        return value


#: 标定零点的合理范围(Pa)。海平面标准大气压 101325；
#: ±3000 Pa 约 ±30 cm 水头，宽到容得下天气和海拔，窄到能看出"在水里标的"。
NOMINAL_SURFACE_PA = 101325
SURFACE_PA_TOLERANCE = 3000

_FIELD_RE = re.compile(r"(\w+)=(-?[\w.]+)")


def _parse_fields(line: str) -> dict[str, str]:
    """把 ``KEY=value,KEY=value`` 解析成字典。

    必须先去掉 ``STAT=`` / ``DIAG=`` 前缀：固件的 DIAG 行形如
    ``DIAG=US=3200,...``，直接扫描会把 ``US`` 当成 ``DIAG`` 的值而丢掉首个字段。
    """
    _, _, payload = line.partition("=")
    return dict(_FIELD_RE.findall(payload))


class TelemetryParser:
    """把下位机的文本行解析进一个 Telemetry 快照。

    容错优先：任何无法识别的行都不应让解析器抛异常，
    因为串口上随时可能出现上电横幅、半截行或噪声。
    """

    def __init__(self) -> None:
        self.state = Telemetry()

    def feed_line(self, line: str, now: float) -> str | None:
        """处理一行。返回值为需要提示给操作者的文本，或 None。"""
        line = line.strip()
        if not line:
            return None
        self.state.last_rx_monotonic = now

        if line.startswith("vofa:"):
            self._parse_vofa(line[5:], now)
            return None
        if line.startswith("STAT="):
            self._parse_stat(line)
            return None
        if line.startswith("DIAG="):
            self._parse_diag(line)
            return None
        if line.startswith("EDIAG"):
            return self._parse_ediag(line)

        # 其余都是文本回执：FB=OK / FB=REJECT xxx / CTRL=ARMED / CAL=...
        self.state.last_reply = line
        if line.startswith("CTRL=ARMED"):
            self.state.armed_reported = True
        elif line.startswith("CTRL=REJECT") or line.startswith("FB=REJECT"):
            self.state.rejections.append(line)
            del self.state.rejections[:-8]   # 只留最近 8 条
            return line
        elif line.startswith("CAL="):
            return line
        return None

    def _parse_vofa(self, payload: str, now: float) -> None:
        parts = payload.split(",")
        if len(parts) != len(VOFA_FIELDS):
            return   # 截断或交错行，整行丢弃
        try:
            values = [float(p) for p in parts]
        except ValueError:
            return

        s = self.state
        # 压力四路在前，姿态/深度在后 —— 顺序见 VOFA_FIELDS 的说明
        s.pressure_legacy = tuple(values[0:4])
        s.roll_deg, s.pitch_deg, s.yaw_deg, s.depth_m = values[4:8]
        s.stopped = bool(values[8])
        s.ready = bool(values[9])
        s.calibrated = bool(values[10])
        s.raw_mask = int(values[11])
        s.fault = bool(values[12])
        s.fused_mask = int(values[13])
        s.error_code = int(values[14])
        s.sequence = int(values[15])

        decoded = decode_error_code(s.error_code)
        s.i2c2_errors = decoded.get("i2c2_errors")
        s.fusion_flags = decoded.get("fusion_flags")
        # VOFA 的停止原因是连续上报的，比 1 s 一次的 STAT 更及时
        if decoded.get("stop_reason") is not None:
            s.stop_reason = decoded["stop_reason"]
        s.last_vofa_monotonic = now

    def _parse_ediag_pressure(self, line: str) -> str | None:
        """EDIAG8/EDIAG9：深度为什么失效、停机那一刻哪一项没成立。"""
        fields = _parse_fields(line)
        s = self.state

        def as_int(key: str) -> int | None:
            raw = fields.get(key)
            try:
                return int(raw) if raw is not None else None
            except ValueError:
                return None

        if line.startswith("EDIAG8="):
            s.pressure_reject_range = as_int("P_RANGE")
            s.pressure_reject_skew = as_int("P_SKEW")
            s.pressure_reject_jump = as_int("P_JUMP")
            s.pressure_reject_empty = as_int("P_EMPTY")
            s.pressure_reject_nis = as_int("P_NIS")
            s.last_pressure_reject_bits = as_int("P_LAST")
            parts = []
            for label, value in (("量程", s.pressure_reject_range),
                                 ("偏斜", s.pressure_reject_skew),
                                 ("跳变", s.pressure_reject_jump),
                                 ("四路全拒", s.pressure_reject_empty),
                                 ("压差NIS", s.pressure_reject_nis)):
                if value:
                    parts.append(f"{label}={value}")
            return "深度失效原因累计：" + ("、".join(parts) if parts else "无") 

        s.stop_not_ready_mask = as_int("NRL")
        # NR_CNT 用 / 分隔，_parse_fields 的 [\w.]+ 匹配不到斜杠，会只取到第一段。
        found = re.search(r"NR_CNT=([\d/]+)", line)
        if found:
            try:
                s.not_ready_counts = tuple(int(v) for v in found.group(1).split("/"))
            except ValueError:
                pass
        reasons = describe_not_ready(s.stop_not_ready_mask)
        if reasons:
            return "停机那一刻未成立：" + "、".join(reasons)
        return None

    def _parse_stat(self, line: str) -> None:
        fields = _parse_fields(line)
        s = self.state

        def as_int(key: str) -> int | None:
            raw = fields.get(key)
            try:
                return int(raw) if raw is not None else None
            except ValueError:
                return None

        s.stat_stopped = None if (v := as_int("STOP")) is None else bool(v)
        s.stat_ready = None if (v := as_int("READY")) is None else bool(v)
        s.stat_calibrated = None if (v := as_int("CAL")) is None else bool(v)
        s.stat_fault = None if (v := as_int("FAULT")) is None else bool(v)
        s.stop_reason = as_int("WHY")
        s.fusion_flags_stat = as_int("FLAGS")
        s.stop_not_ready_mask = as_int("NRL")
        s.cal_failed_channel = as_int("CAL_CH")
        s.cal_samples = as_int("CAL_N")
        s.cal_state = as_int("CAL_ST")
        s.cal_surface_pa = as_int("CAL_PA")
        s.cal_channel_spread_pa = as_int("CAL_SPD")

    def _parse_diag(self, line: str) -> None:
        fields = _parse_fields(line)
        s = self.state

        def as_int(key: str) -> int | None:
            raw = fields.get(key)
            try:
                return int(raw) if raw is not None else None
            except ValueError:
                return None

        s.cycle_us = as_int("US")
        s.max_cycle_us = as_int("MAX")
        s.deadline_misses = as_int("MISS")
        s.rx_drops = as_int("RX")
        s.i2c_errors = as_int("I2C")
        s.heater_fault = as_int("HEAT")
        s.stop_reason = as_int("WHY")
        try:
            s.temperature_c = float(fields["T"])
        except (KeyError, ValueError):
            pass

    def _parse_ediag(self, line: str) -> str | None:
        """EDIAG1..7。只取 EDIAG1 的 NR 掩码，它直接说明 READY 为何不成立。

        其余六行是 IMU/融合/总线的细粒度计数，交给操作者在日志里看原文，
        上位机不去复述固件已经排好的版。
        """
        if line.startswith("EDIAG8=") or line.startswith("EDIAG9="):
            return self._parse_ediag_pressure(line)
        if not line.startswith("EDIAG1="):
            self.state.last_reply = line
            return line
        fields = _parse_fields(line)
        try:
            self.state.not_ready_mask = int(fields["NR"])
        except (KeyError, ValueError):
            return line
        reasons = describe_not_ready(self.state.not_ready_mask)
        if reasons:
            return "READY 未成立：" + "、".join(reasons)
        return None
