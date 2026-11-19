#!/usr/bin/env python3
"""Dual-fisheye to equirectangular stitching from the camera's own factory calibration.

Needs no Insta360 MediaSDK, so it runs on ARM boards (tested on a Raspberry Pi 5 with a ONE X2).

The driver publishes the camera's calibration on /insta360/camera_preview_info. One of its
strings is an omnidirectional (Mei-model) calibration for both lenses: xi, fx, fy, cx, cy, the
lens angles, and k1 k2 k3 p1 p2 distortion, at the sensor's full resolution. This node projects
every output direction through that model into the raw side-by-side frame, so no manual crop,
centre or radius tuning is needed.

In the overlap between the lenses it matches exposure (low-frequency, on luma) and feathers the
seam over blend_deg. Straight below the camera the seam is hard, which hides a pole or tripod in
the blind gap between the lenses.

Optional corrections, both off by default:
  rim:  extra angle near the image rim, per lens [back, front]: theta * (1 + a * (theta/90deg)^4).
        Lens guards shift the rim slightly; ~0.1 suited a ONE X2 with guards.
  rot1: small extra rotation of the back lens, degrees about x, y, z.
"""
import array

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from insta360_ros_driver.msg import CameraPreviewInfo


def parse_cal(cal):
    f = cal.split('_')
    lenses = []
    for i in range(2):
        b = 1 + i * 19
        v = list(map(float, f[b:b + 16]))
        lenses.append(dict(xi=v[0], fx=v[1], fy=v[2], cx=v[3], cy=v[4], ang=v[5:8], d=v[11:16],
                           src_w=float(f[b + 16])))
    return lenses


def is_omni_cal(c):
    """The omnidirectional-model string: model id, then xi close to 1.x, two lenses of 19 fields."""
    f = c.split('_')
    return len(f) >= 40 and f[1].startswith('1.')


def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def lens_rotations(L1, L2):
    """Camera frame (x right, y down, z = front lens axis) to each lens frame."""
    base = rot('z', 90)                       # the sensors are mounted sideways

    def ext(a):
        return rot('z', a[2]) @ rot('y', a[1]) @ rot('x', a[0])
    return ext(L1['ang']) @ base @ rot('y', 180), ext(L2['ang']) @ base


def project(D, L, scale, rim=0.0):
    """Directions in a lens frame to pixels in the raw frame (Mei model with radial-tangential distortion)."""
    X = D / np.linalg.norm(D, axis=-1, keepdims=True)
    theta = np.arccos(np.clip(X[..., 2], -1, 1))
    if rim:
        r = np.maximum(np.hypot(X[..., 0], X[..., 1]), 1e-9)
        t2 = theta * (1 + rim * (theta / (np.pi / 2)) ** 4)
        X = np.stack([X[..., 0] / r * np.sin(t2), X[..., 1] / r * np.sin(t2), np.cos(t2)], -1)
    z = X[..., 2] + L['xi']
    x, y = X[..., 0] / z, X[..., 1] / z
    k1, k2, k3, p1, p2 = L['d']
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    u = (L['fx'] * xd + L['cx']) * scale
    v = (L['fy'] * yd + L['cy']) * scale
    return u.astype(np.float32), v.astype(np.float32), np.degrees(theta)


class LensModel:
    """Maps camera-frame directions to raw-frame pixels for both lenses, plus blend weights."""

    def __init__(self, cal, in_w, blend_deg=5.0, rim=(0.0, 0.0), rot1=(0.0, 0.0, 0.0)):
        self.L1, self.L2 = parse_cal(cal)
        self.scale = in_w / self.L1['src_w']
        R1, R2 = lens_rotations(self.L1, self.L2)
        self.R1 = R1 @ rot('z', rot1[2]) @ rot('y', rot1[1]) @ rot('x', rot1[0])
        self.R2 = R2
        self.rim = rim
        self.blend_deg = blend_deg

    def maps(self, D):
        """D: (h, w, 3) directions. Returns per-lens (u, v) maps and w1, the weight of the back lens."""
        u1, v1, t1 = project(D @ self.R1.T, self.L1, self.scale, self.rim[0])
        u2, v2, t2 = project(D @ self.R2.T, self.L2, self.scale, self.rim[1])
        # y points down, so latitude > 0 is below the horizon. Near straight down the seam is hard.
        latd = np.degrees(np.arcsin(np.clip(D[..., 1] / np.linalg.norm(D, axis=-1), -1, 1)))
        hard = np.clip((latd - 50) / 20, 0, 1)
        blend = self.blend_deg * (1 - hard) + 0.5 * hard
        w1 = np.clip(0.5 + (t2 - t1) / blend, 0, 1).astype(np.float32)
        return (u1, v1), (u2, v2), w1


def maps16(u, v):
    return cv2.convertMaps(np.ascontiguousarray(u), np.ascontiguousarray(v), cv2.CV_16SC2)


class Renderer:
    """One combined remap for the whole output; both lenses are sampled only in tiles on the seam.
    The exposure match between the lenses is low-frequency, so it is computed at 1/8 resolution."""

    TILE = 64
    LO = 8

    def __init__(self, model, D):
        (u1, v1), (u2, v2), w1 = model.maps(D)
        h, w = w1.shape
        self.size = (w, h)
        sel = w1 > 0.5
        self.full = maps16(np.where(sel, u1, u2), np.where(sel, v1, v2))
        lo = (max(1, w // self.LO), max(1, h // self.LO))
        small = lambda a: cv2.resize(a, lo, interpolation=cv2.INTER_AREA)
        self.lo1 = maps16(small(u1), small(v1))
        self.lo2 = maps16(small(u2), small(v2))
        band = (w1 > 0.02) & (w1 < 0.98)
        self.tiles = []
        T = self.TILE
        for y0 in range(0, h, T):
            for x0 in range(0, w, T):
                ys, xs = slice(y0, min(h, y0 + T)), slice(x0, min(w, x0 + T))
                if band[ys, xs].any():
                    self.tiles.append((ys, xs, maps16(u1[ys, xs], v1[ys, xs]), maps16(u2[ys, xs], v2[ys, xs]),
                                       np.ascontiguousarray(w1[ys, xs])[..., None]))

    def __call__(self, img):
        out = cv2.remap(img, *self.full, cv2.INTER_LINEAR)
        if not self.tiles:
            return out
        # luma ratio between the lenses, low-pass, at low resolution
        la = cv2.remap(img, *self.lo1, cv2.INTER_LINEAR).astype(np.float32).mean(axis=2)
        lb = cv2.remap(img, *self.lo2, cv2.INTER_LINEAR).astype(np.float32).mean(axis=2)
        s = 10.0 / self.LO
        r = np.log((cv2.GaussianBlur(lb, (0, 0), s) + 2) / (cv2.GaussianBlur(la, (0, 0), s) + 2))
        r = cv2.resize(r, self.size, interpolation=cv2.INTER_LINEAR)
        for ys, xs, m1, m2, w1 in self.tiles:
            a = cv2.remap(img, *m1, cv2.INTER_LINEAR).astype(np.float32)
            b = cv2.remap(img, *m2, cv2.INTER_LINEAR).astype(np.float32)
            t = 1 - w1                                     # weight of the front lens
            rt = r[ys, xs][..., None]
            a *= np.exp(rt * t)                            # meet in the middle of the seam
            b *= np.exp(-rt * (1 - t))
            out[ys, xs] = np.clip(a * (1 - t) + b * t, 0, 255).astype(np.uint8)
        return out


def equirect_dirs(ow, oh):
    lon = np.radians((np.arange(ow) + 0.5) / ow * 360 - 180)
    lat = np.radians((np.arange(oh) + 0.5) / oh * 180 - 90)
    LO, LA = np.meshgrid(lon, lat)
    return np.stack([np.cos(LA) * np.sin(LO), np.sin(LA), np.cos(LA) * np.cos(LO)], -1)


class CalibratedStitcher(Node):
    def __init__(self):
        super().__init__('calibrated_stitcher')
        self.declare_parameter('input_topic', '/dual_fisheye/image')
        self.declare_parameter('output_topic', '/equirectangular/image')
        self.declare_parameter('out_width', 1920)
        self.declare_parameter('blend_deg', 5.0)
        self.declare_parameter('rim', [0.0, 0.0])
        self.declare_parameter('rot1', [0.0, 0.0, 0.0])
        self.declare_parameter('max_fps', 0.0)            # 0: stitch every input frame
        self.cal = None
        self.renderer = None
        self.key = None
        self.last = 0.0
        info_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraPreviewInfo, '/insta360/camera_preview_info', self.on_info, info_qos)
        self.create_subscription(Image, self.get_parameter('input_topic').value, self.on_image, 1)
        self.pub = self.create_publisher(Image, self.get_parameter('output_topic').value, 1)

    def on_info(self, msg):
        for c in msg.camera_offset:
            if is_omni_cal(c):
                if c != self.cal:
                    self.cal = c
                    self.renderer = None
                    self.get_logger().info("Using the camera's factory calibration")
                return
        self.get_logger().warn('No omnidirectional calibration in camera_preview_info', once=True)

    def on_image(self, msg):
        if self.cal is None:
            self.get_logger().warn('Waiting for /insta360/camera_preview_info', throttle_duration_sec=10)
            return
        max_fps = float(self.get_parameter('max_fps').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        if max_fps > 0 and now - self.last < 1.0 / max_fps:
            return
        self.last = now
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warn(f'Unsupported encoding {msg.encoding}', throttle_duration_sec=10)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        ow = int(self.get_parameter('out_width').value)
        key = (msg.width, ow)
        if self.renderer is None or self.key != key:
            model = LensModel(self.cal, msg.width, float(self.get_parameter('blend_deg').value),
                              tuple(self.get_parameter('rim').value), tuple(self.get_parameter('rot1').value))
            self.renderer = Renderer(model, equirect_dirs(ow, ow // 2))
            self.key = key
            self.get_logger().info(f'Stitching {msg.width}x{msg.height} to {ow}x{ow // 2}')
        out = self.renderer(img)
        o = Image()
        o.header = msg.header
        o.height, o.width = out.shape[:2]
        o.encoding = msg.encoding
        o.step = o.width * 3
        o.data = array.array('B', out.tobytes())       # assigning bytes directly is very slow in rclpy
        self.pub.publish(o)


def main():
    rclpy.init()
    try:
        rclpy.spin(CalibratedStitcher())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
