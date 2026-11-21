"""CAMERA_RECONEXAO_20261121: continuous V4L2 capture, newest frame only.

A reader thread drains the driver at camera rate, so processing never gets a
frame that waited in the V4L2 queue. When no frame arrives for
``reconnect_after_s`` the device is released and reopened by its path
(use /dev/v4l/by-id/...), reapplying v4l2 controls and the format. Only the
capture is restarted: tape_keepout and its confirmed walls are not touched.
ROS-independent, so it can be tested with a fake capture.
"""
import subprocess
import sys
import threading
import time
import traceback

import cv2


class LatestFrameCamera:
    def __init__(self, device, width=640, height=480, fps=30.0, fourcc='MJPG',
                 controls='', reconnect_after_s=2.0, retry_s=2.0, read_timeout_ms=1000,
                 log=None, opener=None, set_controls=None, clock=time.monotonic):
        if reconnect_after_s <= 0 or retry_s <= 0:
            raise ValueError('reconnect_after_s and retry_s must be positive')
        self.device, self.width, self.height = device, width, height
        self.fps, self.fourcc, self.controls = fps, fourcc, controls
        self.reconnect_after_s, self.retry_s = reconnect_after_s, retry_s
        self.read_timeout_ms = read_timeout_ms
        # log: dict level -> callable. Each call site below uses one fixed level,
        # as rclpy loggers refuse severity changes from the same call site.
        quiet = lambda text: None
        self.log = dict(info=quiet, warn=quiet, error=quiet)
        self.log.update(log or {})
        self.opener = opener or (lambda device: cv2.VideoCapture(device, cv2.CAP_V4L2))
        self.set_controls = set_controls or self._v4l2_ctl
        self.clock = clock
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest_item = None
        self.sequence = 0
        self.reconnections = 0
        self.connected = False
        # Startup keeps the old contract: no camera, no node.
        self.cap = self._open()
        self.thread = threading.Thread(target=self._loop, name='xift_capture', daemon=True)
        self.thread.start()

    def _v4l2_ctl(self, device, controls):
        result = subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl=' + controls],
                                capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            raise RuntimeError('v4l2-ctl failed: ' + (result.stderr or result.stdout).strip())

    def _open(self):
        if self.controls:
            self.set_controls(self.device, self.controls)
        cap = self.opener(self.device)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError('Cannot open V4L2 camera: ' + self.device)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Without this, OpenCV may block ~10 s in select() on a dead device.
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.read_timeout_ms)
        self.log['info']('Negotiated camera: %.0fx%.0f %.1f FPS' % (
            cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            cap.get(cv2.CAP_PROP_FPS)))
        self.connected = True
        self.last_ok = self.clock()
        return cap

    def _release(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except cv2.error:
                pass
        self.cap = None
        self.connected = False

    def _loop(self):
        # The capture thread must never die: an exception here would leave the
        # node alive but blind (bug seen 21/11 on the first USB unplug test).
        while not self.stop_event.is_set():
            try:
                self._step()
            except Exception:
                traceback.print_exc(file=sys.stderr)
                self._release()
                self.stop_event.wait(self.retry_s)

    def _step(self):
        if self.cap is None:
            try:
                self.cap = self._open()
                self.reconnections += 1
                self.log['info']('CAMERA_RECONEXAO: camera reaberta (%d)' % self.reconnections)
            except (RuntimeError, OSError, subprocess.SubprocessError, cv2.error) as error:
                self.log['warn']('CAMERA_RECONEXAO: aguardando camera: %s' % error)
                self.stop_event.wait(self.retry_s)
            return
        try:
            ok, frame = self.cap.read()
        except cv2.error:
            ok, frame = False, None
        read_time = self.clock()
        if not ok or frame is None:
            if read_time - self.last_ok >= self.reconnect_after_s:
                self.log['error']('CAMERA_RECONEXAO: sem quadros ha %.1f s; reabrindo %s'
                         % (read_time - self.last_ok, self.device))
                self._release()
            else:
                self.stop_event.wait(0.02)
            return
        self.last_ok = read_time
        driver_ms = self.cap.get(cv2.CAP_PROP_POS_MSEC)
        # V4L2 buffer time; meaningful only if the driver uses CLOCK_MONOTONIC.
        driver_age = read_time - driver_ms / 1000. if driver_ms > 0 else None
        with self.lock:
            self.sequence += 1
            self.latest_item = (frame, read_time, driver_age, self.sequence)

    def latest(self, after_sequence=0):
        """Newest frame newer than after_sequence: (frame, read_time, driver_age, seq)."""
        with self.lock:
            item = self.latest_item
        if item is None or item[3] <= after_sequence:
            return None
        return item

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=3.)
        self._release()
