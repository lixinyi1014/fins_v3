"""串口链路管理：心跳、接收解析、断线重连、失效保护。

与老程序最大的区别是三点：
1. 心跳由独立线程按 100 ms 发送，不依赖主循环节奏。固件 500 ms 收不到 HB
   就锁存停止，把心跳压在渲染循环里意味着摄像头一卡就会被判掉线。
2. 接收线程持续解析下位机上报，HUD 显示真实深度/姿态而不是写死的数字。
3. 所有发送失败都会被记录并触发重连，不再静默返回 False。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import serial

import protocol
from protocol import Command, TelemetryParser


@dataclass
class LinkStats:
    sent: int = 0
    send_failures: int = 0
    reconnects: int = 0
    lines_received: int = 0
    parse_discards: int = 0


class SerialLink:
    """线程安全的下位机链路。

    生命周期：start() 起接收与心跳线程，close() 保证发出 OFF 并归位。
    未连接时所有 send 立即失败并被计数，不会假装成功。
    """

    def __init__(self, port: str, *, log=print) -> None:
        self.port_name = port
        self._log = log
        self._serial: serial.Serial | None = None

        self._write_lock = threading.Lock()   # 保证整行原子写出
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()

        self.stats = LinkStats()
        self.parser = TelemetryParser()

        # 心跳控制：arm 意图与心跳解耦，急停时主动掐掉心跳让固件超时锁存
        self._heartbeat_enabled = True
        self._heartbeat_suppressed_until = 0.0
        self._last_heartbeat = 0.0

        self._rx_thread: threading.Thread | None = None
        self._hb_thread: threading.Thread | None = None
        self._rx_buffer = bytearray()

    # ---- 连接管理 ----------------------------------------------------------
    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._serial is not None and self._serial.is_open

    def open(self) -> bool:
        """尝试打开串口。返回是否成功，调用方必须检查。"""
        try:
            handle = serial.Serial(
                port=self.port_name,
                baudrate=protocol.BAUDRATE,
                bytesize=protocol.BYTESIZE,
                parity=protocol.PARITY,
                stopbits=protocol.STOPBITS,
                timeout=0.05,        # 读超时，供接收线程退出检查
                write_timeout=0.2,   # 绝不无限阻塞在写上
            )
        except (serial.SerialException, OSError, ValueError) as exc:
            self._log(f"[链路] 打开 {self.port_name} 失败: {exc}")
            return False

        with self._state_lock:
            self._serial = handle
        self._rx_buffer.clear()
        self._log(f"[链路] 已连接 {self.port_name} "
                  f"{protocol.BAUDRATE}/8{protocol.PARITY}1")
        return True

    def _drop(self, reason: str) -> None:
        with self._state_lock:
            handle, self._serial = self._serial, None
        if handle is not None:
            self._log(f"[链路] 断开: {reason}")
            try:
                handle.close()
            except Exception:
                pass

    def start(self) -> None:
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop,
                                          name="fins-rx", daemon=True)
        self._hb_thread = threading.Thread(target=self._heartbeat_loop,
                                          name="fins-hb", daemon=True)
        self._rx_thread.start()
        self._hb_thread.start()

    # ---- 发送 --------------------------------------------------------------
    def send(self, command: str) -> bool:
        """发送一条命令。整行一次写出，失败即断开并计数。"""
        try:
            payload = protocol.encode(command)
        except ValueError as exc:
            self._log(f"[链路] 拒发非法命令: {exc}")
            return False

        with self._state_lock:
            handle = self._serial
        if handle is None:
            self.stats.send_failures += 1
            return False

        try:
            with self._write_lock:
                handle.write(payload)
            self.stats.sent += 1
            return True
        except (serial.SerialException, serial.SerialTimeoutException,
                OSError) as exc:
            self.stats.send_failures += 1
            self._drop(f"写入失败 {exc}")
            return False

    # ---- 接收 --------------------------------------------------------------
    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._state_lock:
                handle = self._serial
            if handle is None:
                if self._stop_event.wait(1.0):
                    break
                if self.open():
                    self.stats.reconnects += 1
                continue

            try:
                chunk = handle.read(256)
            except (serial.SerialException, OSError) as exc:
                self._drop(f"读取失败 {exc}")
                continue

            if not chunk:
                continue

            self._rx_buffer.extend(chunk)
            if len(self._rx_buffer) > 8192:      # 防御异常刷屏
                del self._rx_buffer[:-2048]
                self.stats.parse_discards += 1

            now = time.monotonic()
            while True:
                index = self._rx_buffer.find(b"\n")
                if index < 0:
                    break
                raw = bytes(self._rx_buffer[:index])
                del self._rx_buffer[:index + 1]
                try:
                    line = raw.decode("ascii", errors="replace")
                except Exception:
                    self.stats.parse_discards += 1
                    continue
                self.stats.lines_received += 1
                notice = self.parser.feed_line(line, now)
                if notice:
                    self._log(f"[下位机] {notice}")

    # ---- 心跳 --------------------------------------------------------------
    def _heartbeat_loop(self) -> None:
        """独立发送 HB。固件的 HB 只续期、绝不解除停止锁存，
        所以停止状态下继续发心跳是安全的，也能让遥测保持流动。"""
        while not self._stop_event.is_set():
            now = time.monotonic()
            if (self._heartbeat_enabled
                    and now >= self._heartbeat_suppressed_until
                    and self.connected):
                self.send(Command.HEARTBEAT)
                self._last_heartbeat = now
            self._stop_event.wait(protocol.HEARTBEAT_PERIOD_S)

    def suppress_heartbeat(self, duration_s: float) -> None:
        """暂停心跳一段时间，让固件的 500 ms 超时必然触发锁存停止。

        这是急停的兜底手段：即使 OFF 因链路故障没发出去，
        停发心跳也能让下位机在 500 ms 内停机。
        """
        self._heartbeat_suppressed_until = max(
            self._heartbeat_suppressed_until, time.monotonic() + duration_s)

    def heartbeat_age_s(self) -> float:
        if not self._last_heartbeat:
            return float("inf")
        return time.monotonic() - self._last_heartbeat

    # ---- 关闭 --------------------------------------------------------------
    def close(self, *, send_disarm: bool = True) -> None:
        """有序关闭：先停机，再停线程，最后关串口。

        老程序没有 try/finally，崩溃时既不发停止命令也不关串口，
        潜器会保持最后的运动状态。这里由调用方的 finally 保证执行。
        """
        if send_disarm and self.connected:
            for _ in range(3):        # OFF 在接收中断锁存，重发提高送达率
                self.send(Command.DISARM)
                time.sleep(0.02)
        self._stop_event.set()
        for thread in (self._hb_thread, self._rx_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        self._drop("正常关闭")
