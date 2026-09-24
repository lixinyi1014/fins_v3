"""摄像头采集线程。

老程序在主循环里同步调用 cap.read()，USB 取帧的阻塞时间直接压低控制频率，
clock.tick(30) 只是上限而不是保证。这里把取帧移到独立线程，
主循环只取"最新一帧"，取不到就跳过渲染，绝不等待。
"""

from __future__ import annotations

import threading
import time

import cv2


class CameraStream:
    """后台取帧，主线程随时取最新帧。

    摄像头缺失或掉线不影响控制：frame 返回 None，HUD 显示提示即可。
    """

    def __init__(self, index: int = 0, *, rotate: bool = True,
                 mirror: bool = True, log=print) -> None:
        self.index = index
        self.rotate = rotate
        self.mirror = mirror
        self._log = log

        self._capture: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._frame = None
        self._frame_monotonic = 0.0
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self.frames = 0
        self.failures = 0

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="fins-camera", daemon=True)
        self._thread.start()

    def _open(self) -> bool:
        try:
            capture = cv2.VideoCapture(self.index)
        except Exception as exc:
            self._log(f"[摄像头] 打开失败: {exc}")
            return False
        if not capture.isOpened():
            capture.release()
            return False
        # 尽量压低缓冲，避免看到几帧之前的画面
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._capture = capture
        self._log(f"[摄像头] 已打开 index={self.index}")
        return True

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            if self._capture is None:
                if not self._open():
                    if self._stop_event.wait(2.0):
                        break
                    continue

            try:
                ok, frame = self._capture.read()
            except Exception as exc:
                self._log(f"[摄像头] 读取异常: {exc}")
                ok, frame = False, None

            if not ok or frame is None:
                self.failures += 1
                self._capture.release()
                self._capture = None
                continue

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if self.rotate:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            if self.mirror:
                frame = cv2.flip(frame, 1)

            with self._lock:
                self._frame = frame
                self._frame_monotonic = time.monotonic()
            self.frames += 1

    def latest(self, max_age_s: float = 1.0):
        """返回 (帧, 年龄秒)。帧过期或从未取到则帧为 None。"""
        with self._lock:
            frame = self._frame
            stamp = self._frame_monotonic
        if frame is None:
            return None, None
        age = time.monotonic() - stamp
        if age > max_age_s:
            return None, age
        return frame, age

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._capture is not None:
            self._capture.release()
            self._capture = None
