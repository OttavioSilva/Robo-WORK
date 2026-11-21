"""CAMERA_RECONEXAO_20261121: reconnection and newest-frame capture (no ROS)."""
import sys
import threading
import time

import cv2
import numpy as np
import pytest

from robo_virtual_wall.camera import LatestFrameCamera


class FakeDevice:
    """Shared fake: `alive` simulates the USB device being present."""
    def __init__(self):
        self.alive = True
        self.opens = 0
        self.controls = []
        self.frames = 0
        self.lock = threading.Lock()

    def open(self, device):
        self.opens += 1
        return FakeCapture(self)

    def set_controls(self, device, controls):
        if not self.alive:
            raise RuntimeError('device missing')
        self.controls.append((device, controls))


class FakeCapture:
    def __init__(self, device):
        self.device = device
        self.opened = device.alive
        self.settings = {}

    def isOpened(self):
        return self.opened

    def release(self):
        self.opened = False

    def set(self, key, value):
        self.settings[key] = value
        return True

    def get(self, key):
        if key == cv2.CAP_PROP_POS_MSEC:
            return time.monotonic() * 1000 - 5.  # driver on CLOCK_MONOTONIC, 5 ms old
        return self.settings.get(key, 0.)

    def read(self):
        time.sleep(0.005)
        if not (self.opened and self.device.alive):
            return False, None
        with self.device.lock:
            self.device.frames += 1
            value = self.device.frames
        return True, np.full((4, 4, 3), value % 256, np.uint8)


def wait_for(condition, timeout=3.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


class RclpyLikeLog(dict):
    """Mimic rclpy: one call site may not change severity between calls."""
    def __init__(self):
        super().__init__()
        self.sites, self.lines = {}, []
        for level in ('info', 'warn', 'error'):
            self[level] = self._make(level)

    def _make(self, level):
        def emit(text):
            frame = sys._getframe(1)
            site = (frame.f_code.co_filename, frame.f_lineno)
            if self.sites.setdefault(site, level) != level:
                raise ValueError('Logger severity cannot be changed between calls.')
            self.lines.append((level, text))
        return emit


def make(device, **kwargs):
    return LatestFrameCamera('/dev/v4l/by-id/fake', controls='focus_automatic_continuous=0',
                             reconnect_after_s=0.2, retry_s=0.05, opener=device.open,
                             set_controls=device.set_controls, **kwargs)


def test_startup_without_device_fails():
    device = FakeDevice()
    device.alive = False
    with pytest.raises(RuntimeError):
        make(device)


def test_format_and_single_buffer_applied():
    device = FakeDevice()
    camera = make(device)
    try:
        settings = camera.cap.settings
        assert settings[cv2.CAP_PROP_BUFFERSIZE] == 1
        assert settings[cv2.CAP_PROP_FRAME_WIDTH] == 640
        assert settings[cv2.CAP_PROP_FOURCC] == cv2.VideoWriter_fourcc(*'MJPG')
        assert device.controls == [('/dev/v4l/by-id/fake', 'focus_automatic_continuous=0')]
    finally:
        camera.close()


def test_only_newest_frame_and_driver_age():
    device = FakeDevice()
    camera = make(device)
    try:
        assert wait_for(lambda: camera.latest() is not None)
        first = camera.latest()
        assert wait_for(lambda: camera.latest(first[3]) is not None)
        newer = camera.latest(first[3])
        assert newer[3] > first[3]
        assert camera.latest(newer[3] + 10**6) is None
        assert 0. <= newer[2] < 0.1
    finally:
        camera.close()


def test_reopens_after_loss_and_reapplies_controls():
    log = RclpyLikeLog()
    logs = log.lines
    device = FakeDevice()
    camera = make(device, log=log)
    try:
        assert wait_for(lambda: camera.latest() is not None)
        device.alive = False
        assert wait_for(lambda: not camera.connected)
        lost = camera.latest()[3]
        time.sleep(0.2)
        assert camera.latest(lost) is None  # nothing new while unplugged
        device.alive = True
        assert wait_for(lambda: camera.reconnections == 1)
        assert wait_for(lambda: camera.latest(lost) is not None)
        assert device.opens >= 2 and len(device.controls) >= 2
        assert any('reabrindo' in text for _, text in logs)
        assert any('reaberta' in text for _, text in logs)
    finally:
        camera.close()
    assert not camera.thread.is_alive()


def test_thread_survives_unexpected_exception():
    device = FakeDevice()
    camera = make(device)
    try:
        assert wait_for(lambda: camera.latest() is not None)
        broken = camera.cap
        broken.read = lambda: (_ for _ in ()).throw(ZeroDivisionError('boom'))
        assert wait_for(lambda: camera.cap is not None and camera.cap is not broken)
        last = camera.latest()[3]
        assert wait_for(lambda: camera.latest(last) is not None)
        assert camera.thread.is_alive()
    finally:
        camera.close()
