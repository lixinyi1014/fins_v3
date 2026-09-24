"""协议层自测。不依赖串口、手柄和摄像头，可直接运行：

    python test_protocol.py

覆盖的是最容易出错、也最容易被静默忽略的部分：命令编码是否带行尾、
范围检查是否与固件一致、遥测解析能否容忍脏数据。
"""

from __future__ import annotations

import protocol
from protocol import Command, TelemetryParser


def check(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)
    print(f"  ok  {label}")


def test_encoding() -> None:
    print("编码")
    check(protocol.encode(Command.ARM) == b"ON\r\n", "ON 带 CRLF 行尾")
    check(protocol.encode(Command.HEARTBEAT) == b"HB\r\n", "HB 带 CRLF 行尾")

    for bad, label in (("", "空命令被拒"),
                       ("ON\r\n", "自带行尾被拒"),
                       ("X" * 100, "超长命令被拒"),
                       ("O\x01N", "非法字节被拒")):
        try:
            protocol.encode(bad)
        except ValueError:
            print(f"  ok  {label}")
        else:
            raise AssertionError(label)


def test_servo() -> None:
    print("舵机命令")
    check(protocol.servo_command((1500, 1500, 1500, 1500))
          == "MOT:1500,1500,1500,1500", "四路中位")
    check(protocol.servo_command((500, 2500, 500, 2500))
          == "MOT:500,2500,500,2500", "边界值可用")

    for bad in ((499, 1500, 1500, 1500), (1500, 2501, 1500, 1500)):
        try:
            protocol.servo_command(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"越界未被拒: {bad}")
    print("  ok  越界被拒")

    # 固件对第三路做 3000-x 反转，HUD 必须能算出实际值
    check(protocol.firmware_servo_echo((1500, 1600, 1700, 1800))
          == (1500, 1600, 1300, 1800), "第三路反转被正确还原")

    check(abs(protocol.servo_pwm_to_angle_deg(1500)) < 1e-9, "1500 us 对应 0°")
    check(abs(protocol.servo_pwm_to_angle_deg(500) + 90) < 1e-9, "500 us 对应 -90°")
    check(abs(protocol.servo_pwm_to_angle_deg(2500) - 90) < 1e-9, "2500 us 对应 +90°")


def test_fset() -> None:
    print("FSET 命令")
    check(protocol.fused_target_command(0.3, 0, 0, 0)
          == "FSET:0.300,0.00,0.00,0.00", "标准格式")

    for args in ((-0.1, 0, 0, 0), (101, 0, 0, 0), (1, 31, 0, 0),
                 (1, 0, 31, 0), (1, 0, 0, 181)):
        try:
            protocol.fused_target_command(*args)
        except ValueError:
            pass
        else:
            raise AssertionError(f"越界未被拒: {args}")
    print("  ok  四项范围检查与固件一致")


def test_thrusters() -> None:
    print("TES 命令")
    check(protocol.manual_thruster_command((1610,) * 8)
          == "TES:1610,1610,1610,1610,1610,1610,1610,1610", "八路中位 1610")
    for bad in ((1610,) * 7, (999,) + (1610,) * 7, (2001,) + (1610,) * 7):
        try:
            protocol.manual_thruster_command(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"非法输入未被拒: {bad}")
    print("  ok  路数与范围检查")


def test_vofa_parsing() -> None:
    print("VOFA 解析")
    parser = TelemetryParser()
    # 新通道顺序：压力四路在前，姿态/深度在后
    line = ("vofa:1004.10,1004.20,1004.30,1004.40,1.50,-2.25,30.00,0.412,"
            "0,1,1,15,0,15,265,1234")
    parser.feed_line(line, now=100.0)
    s = parser.state
    check(s.roll_deg == 1.50 and s.pitch_deg == -2.25, "姿态角解析")
    check(s.yaw_deg == 30.0 and s.depth_m == 0.412, "航向与深度解析")
    check(s.pressure_legacy == (1004.10, 1004.20, 1004.30, 1004.40),
          "四路 legacy 压力裸值解析（前 4 通道）")
    check(s.stopped is False and s.ready is True, "状态位解析")
    check(s.calibrated is True and s.fault is False, "校准与故障位解析")
    check(s.raw_mask == 15 and s.sequence == 1234, "掩码与序号解析")
    # ERROR_CODE=265=0x109：低8位=9(收到OFF)，8..15位=1(I2C2错误)
    check(s.error_code == 265, "ERROR_CODE 解析")
    check(s.stop_reason == 9, "ERROR_CODE 低 8 位为停止原因")
    check(s.i2c2_errors == 1, "ERROR_CODE 8..15 位为 I2C2 错误计数")
    check(s.vofa_age_s(100.5) == 0.5, "遥测年龄计算")

    # -9999 是缺测占位，绝不能当成真实深度
    parser.feed_line("vofa:-9999.00,-9999.00,-9999.00,-9999.00,"
                     "-9999.00,-9999.00,-9999.00,-9999.000,"
                     "1,0,1,0,0,0,0,1235", now=101.0)
    check(parser.state.depth_m == protocol.MISSING, "缺测值被保留为 MISSING")
    check(parser.state.value_or_none("depth_m") is None,
          "缺测经 value_or_none 归一为 None")

    # 脏数据不得抛异常，也不得污染已有状态
    before = parser.state.sequence
    for junk in ("vofa:1,2,3", "vofa:a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p",
                 "", "   ", "vofa:"):
        parser.feed_line(junk, now=102.0)
    check(parser.state.sequence == before, "脏 vofa 行被整行丢弃")


def test_vofa_field_order_regression() -> None:
    """通道顺序是回归重点：a69fbae 把压力四路移到了最前面。

    按旧顺序解析不会报错，只会把压力值当姿态角显示 ——
    正是最危险的那种静默错误，所以用一条可辨识的样例锁住顺序。
    """
    print("VOFA 通道顺序回归")
    parser = TelemetryParser()
    # 压力用 1000 量级、姿态用小值，一旦顺序错了数值会明显荒谬
    parser.feed_line("vofa:1001.00,1002.00,1003.00,1004.00,"
                     "11.00,22.00,33.00,0.440,1,1,1,15,0,15,0,1", now=1.0)
    s = parser.state
    check(s.pressure_legacy[0] == 1001.0, "通道 0 是压力，不是 roll")
    check(s.roll_deg == 11.0, "通道 4 是 roll，不是压力")
    check(s.pitch_deg == 22.0 and s.yaw_deg == 33.0, "通道 5/6 是 pitch/yaw")
    check(s.depth_m == 0.440, "通道 7 是深度")
    check(abs(s.roll_deg) <= 180 and abs(s.pitch_deg) <= 90,
          "姿态角落在物理量程内，说明没读错通道")
    check(protocol.VOFA_FIELDS[0] == "p0_legacy"
          and protocol.VOFA_FIELDS[4] == "roll_deg",
          "VOFA_FIELDS 顺序与固件一致")


def test_error_code_decoding() -> None:
    """ERROR_CODE 取代了原来的 FLAGS，按位打包三样信息。"""
    print("ERROR_CODE 位解析")
    decoded = protocol.decode_error_code(0x0007_02_01)
    check(decoded["stop_reason"] == 1, "低 8 位为停止原因")
    check(decoded["i2c2_errors"] == 2, "8..15 位为 I2C2 累计错误")
    check(decoded["fusion_flags"] == 7, "16..31 位为 ESKF flags")
    check(protocol.decode_error_code(None) == {}, "None 不抛异常")
    check(protocol.decode_error_code(0)["stop_reason"] == 0, "0 表示无记录")

    # VOFA 的停止原因应该覆盖进 stop_reason，它比 1 s 一次的 STAT 更及时
    parser = TelemetryParser()
    parser.feed_line("vofa:1004,1004,1004,1004,0,0,0,0.1,"
                     "1,0,1,15,0,15,257,5", now=1.0)
    check(parser.state.stop_reason == 1,
          "VOFA 持续上报的停止原因被采纳（257=0x101）")
    check(parser.state.i2c2_errors == 1, "同时取出 I2C2 错误计数")


def test_ediag_parsing() -> None:
    """EDIAG1 的 NR 掩码直接回答'为什么 ON 被拒'。"""
    print("EDIAG 解析")
    check(protocol.describe_not_ready(0) == [], "掩码 0 表示全部通过")
    check(protocol.describe_not_ready(None) == [], "None 不抛异常")
    check(protocol.describe_not_ready(1) == ["压力校准未通过"], "bit0 CAL")
    check(protocol.describe_not_ready(1 << 6) == ["融合状态不可用"],
          "bit6 NOT_USABLE")
    multi = protocol.describe_not_ready((1 << 3) | (1 << 4))
    check(len(multi) == 2 and "压力总线失败" in multi and "IMU 等待超时" in multi,
          "多位同时置位时全部列出")

    parser = TelemetryParser()
    notice = parser.feed_line(
        "EDIAG1=NR=9,WAIT_TO=0,INV=0,LF=0,F=0,T=26.5,HEAT=0,FE=0,EB=0", now=1.0)
    check(parser.state.not_ready_mask == 9, "NR 掩码被记录")
    check(notice is not None and "压力校准未通过" in notice
          and "压力总线失败" in notice, "NR=9 翻译出两个原因并提示操作者")

    # 其余六行原文透传，不复述固件已排好的版
    notice = parser.feed_line("EDIAG7=I2C2_ERR=3,I2C2_REC=1,I2C3_ERR=0,"
                              "I2C3_REC=0,P_FRAME_DROP=0,E2=0x00000000,"
                              "E3=0x00000000", now=2.0)
    check(notice is not None and notice.startswith("EDIAG7="), "EDIAG7 原文透传")


def test_status_parsing() -> None:
    print("STAT / DIAG 解析")
    parser = TelemetryParser()
    parser.feed_line("STAT=ESKF,STOP=1,READY=0,CAL=1,FLAGS=7,MASK=15,"
                     "FAULT=0,WHY=9,CAL_CH=0,CAL_N=100", now=1.0)
    s = parser.state
    check(s.stat_stopped is True and s.stat_ready is False, "STOP/READY")
    check(s.stat_calibrated is True and s.stat_fault is False, "CAL/FAULT")
    check(s.stop_reason == 9 and s.cal_samples == 100, "WHY/CAL_N")
    check(protocol.describe_stop_reason(9) == "收到 OFF", "停止原因释义")
    check(protocol.describe_stop_reason(1) == "心跳超时", "心跳超时释义")

    parser.feed_line("DIAG=US=3200,MAX=4100,MISS=0,RX=2,I2C=0,ACC_REJ=1,"
                     "P_REJ=0,T=26.5,HEAT=0,WHY=1", now=2.0)
    check(s.cycle_us == 3200 and s.max_cycle_us == 4100, "控制周期")
    check(s.rx_drops == 2 and s.temperature_c == 26.5, "丢包与温度")


def test_text_replies() -> None:
    print("文本回执")
    parser = TelemetryParser()
    check(parser.feed_line("CTRL=ARMED ESKF", now=1.0) is None, "ARMED 不提示")
    check(parser.state.armed_reported is True, "ARMED 被记录")

    notice = parser.feed_line("CTRL=REJECT feedback_not_ready USE_STAT", now=2.0)
    check(notice is not None, "REJECT 会提示操作者")
    check(len(parser.state.rejections) == 1, "REJECT 入列表")

    check(parser.feed_line("FB=OK", now=3.0) is None, "FB=OK 不提示")
    notice = parser.feed_line("FB=REJECT stale_command", now=4.0)
    check(notice is not None, "FB=REJECT 会提示")

    for index in range(20):
        parser.feed_line(f"FB=REJECT n{index}", now=5.0)
    check(len(parser.state.rejections) <= 8, "拒绝记录有上限，不会无限增长")


def test_constants_match_firmware() -> None:
    print("常量与固件一致性")
    check(protocol.COMMAND_TIMEOUT_S == 0.5, "命令超时 500 ms")
    check(protocol.HEARTBEAT_PERIOD_S * 1000 == 100, "心跳周期 100 ms")
    check(protocol.HEARTBEAT_PERIOD_S < protocol.COMMAND_TIMEOUT_S / 4,
          "心跳周期留出足够余量（允许连丢 4 包）")
    check(protocol.MAX_LINE_CHARS == 99, "行长上限 99 字符")
    check(protocol.THRUSTER_NEUTRAL_US == 1550,
          "推进器中位 1550 us（V3.3 实物标定，非 1500 也非早期的 1610）")
    check(protocol.THRUSTER_DEADZONE_LOW_US == 1510
          and protocol.THRUSTER_DEADZONE_HIGH_US == 1610, "死区 1510..1610")
    check(protocol.VOFA_PERIOD_S == 0.02, "VOFA 默认 50 Hz")
    check(protocol.PARITY == "E" and protocol.BAUDRATE == 115200, "115200/8E1")

    # 最长命令也必须在行长限制内
    longest = protocol.manual_thruster_command((2000,) * 8)
    check(len(longest) <= protocol.MAX_LINE_CHARS, "最长 TES 命令不超限")


def main() -> int:
    tests = (test_encoding, test_servo, test_fset, test_thrusters,
             test_vofa_parsing, test_vofa_field_order_regression,
             test_error_code_decoding, test_ediag_parsing,
             test_status_parsing, test_text_replies,
             test_constants_match_firmware)
    for test in tests:
        test()
    print(f"\n全部通过（{len(tests)} 组）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
