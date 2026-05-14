#!/usr/bin/env python3
import math
import queue
import struct
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import serial
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header

OOR = -1


class TofSerialNode(Node):
    def __init__(self):
        super().__init__('tof_serial_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('fov_h_deg', 45.0)
        self.declare_parameter('fov_v_deg', 45.0)
        self.declare_parameter('frame_id', 'tof_sensor')

        port     = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._fov_h    = math.radians(self.get_parameter('fov_h_deg').get_parameter_value().double_value)
        self._fov_v    = math.radians(self.get_parameter('fov_v_deg').get_parameter_value().double_value)
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pc_pub   = self.create_publisher(PointCloud2,       '/tof/pointcloud', sensor_qos)
        self._img_pub  = self.create_publisher(Image,             '/tof/heatmap',    sensor_qos)
        self._grid_pub = self.create_publisher(Float32MultiArray, '/tof/grid',       sensor_qos)
        self._bridge   = CvBridge()
        self._queue: queue.Queue = queue.Queue(maxsize=10)
        self._frame_count = 0

        self._ser = self._open_serial(port, baudrate)
        if self._ser is None:
            self.get_logger().error('시리얼 포트 연결 실패. 노드를 종료합니다.')
            return

        self._reader = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader.start()
        self.create_timer(0.05, self._timer_cb)  # 20 Hz

    # ── 시리얼 연결 ───────────────────────────────────────────────────────────

    def _open_serial(self, port: str, baudrate: int):
        try:
            ser = serial.Serial()
            ser.port     = port
            ser.baudrate = baudrate
            ser.timeout  = 2
            ser.dsrdtr   = False   # DTR 신호로 인한 RP2350 자동 리셋 방지
            ser.rtscts   = False
            ser.xonxoff  = False
            ser.open()
            self.get_logger().info(f'시리얼 연결 성공: {port} @ {baudrate}bps')
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f'{port} 연결 실패: {e}')
            return None

    # ── 시리얼 읽기 스레드 ────────────────────────────────────────────────────

    def _serial_reader(self):
        rows: list[int] = []
        parsing = False

        while rclpy.ok():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if line.startswith('Frame'):
                rows    = []
                parsing = True
                continue

            if not parsing:
                continue

            if line.lstrip().startswith('C0'):
                continue

            parts = line.split()
            if not parts or not parts[0].startswith('R'):
                continue

            vals = [self._parse_val(v) for v in parts[1:9]]
            if len(vals) < 8:
                continue
            rows.extend(vals)

            if len(rows) >= 64:
                frame   = rows[:64]
                rows    = []
                parsing = False
                if self._queue.full():
                    try:
                        self._queue.get_nowait()
                    except queue.Empty:
                        pass
                try:
                    self._queue.put_nowait(frame)
                except queue.Full:
                    pass

    @staticmethod
    def _parse_val(s: str) -> int:
        try:
            return int(s)
        except ValueError:
            return OOR

    # ── 타이머 콜백 (20 Hz) ───────────────────────────────────────────────────

    def _timer_cb(self):
        try:
            frame = self._queue.get_nowait()
        except queue.Empty:
            return

        self._frame_count += 1
        stamp = self.get_clock().now().to_msg()

        self._publish_grid(frame, stamp)
        self._publish_pointcloud(frame, stamp)
        self._publish_heatmap(frame, stamp)

        if self._frame_count % 30 == 0:
            valid = [v for v in frame if v != OOR]
            if valid:
                self.get_logger().info(
                    f'Frame #{self._frame_count}  points={len(valid)}  '
                    f'min={min(valid)}mm  max={max(valid)}mm  OOR={frame.count(OOR)}'
                )

    # ── /tof/grid 퍼블리시 (raw 64-float, OOR=-1.0) ──────────────────────────

    def _publish_grid(self, frame: list[int], stamp):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in frame]  # 64개 값, OOR=-1.0
        self._grid_pub.publish(msg)

    # ── PointCloud2 퍼블리시 ──────────────────────────────────────────────────

    def _publish_pointcloud(self, frame: list[int], stamp):
        points = []
        for idx, dist in enumerate(frame):
            if dist == OOR:
                continue
            row, col = divmod(idx, 8)
            dist_m   = dist / 1000.0
            angle_h  = (col - 3.5) / 7.0 * self._fov_h
            angle_v  = (row - 3.5) / 7.0 * self._fov_v
            x = dist_m * math.sin(angle_h)
            y = -dist_m * math.sin(angle_v)
            z = dist_m * math.cos(angle_h) * math.cos(angle_v)
            points.append((x, y, z, float(dist)))

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = bytearray()
        for p in points:
            data += struct.pack('ffff', *p)

        msg = PointCloud2()
        msg.header     = Header(stamp=stamp, frame_id=self._frame_id)
        msg.height     = 1
        msg.width      = len(points)
        msg.fields     = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step   = 16 * len(points)
        msg.data       = bytes(data)
        msg.is_dense   = True
        self._pc_pub.publish(msg)

    # ── 히트맵 이미지 퍼블리시 ────────────────────────────────────────────────

    def _publish_heatmap(self, frame: list[int], stamp):
        grid       = np.array(frame, dtype=np.float32).reshape(8, 8)
        valid_mask = grid != OOR

        img = np.full((8, 8, 3), 128, dtype=np.uint8)  # OOR → 회색

        valid_vals = grid[valid_mask]
        if valid_vals.size > 0:
            mn  = float(valid_vals.min())
            mx  = float(valid_vals.max())
            rng = max(mx - mn, 1.0)

            norm    = np.clip((grid - mn) / rng * 255, 0, 255).astype(np.uint8)
            colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            img[valid_mask] = colored[valid_mask]

        cell = 60
        big  = np.zeros((480, 480, 3), dtype=np.uint8)
        for r in range(8):
            for c in range(8):
                y0, y1 = r * cell, (r + 1) * cell
                x0, x1 = c * cell, (c + 1) * cell
                big[y0:y1, x0:x1] = img[r, c]
                cv2.rectangle(big, (x0, y0), (x1 - 1, y1 - 1), (60, 60, 60), 1)

                val  = frame[r * 8 + c]
                text = '----' if val == OOR else str(val)
                cv2.putText(big, text, (x0 + 4, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1,
                            cv2.LINE_AA)

        ros_img = self._bridge.cv2_to_imgmsg(big, encoding='bgr8')
        ros_img.header = Header(stamp=stamp, frame_id=self._frame_id)
        self._img_pub.publish(ros_img)

    def destroy_node(self):
        if hasattr(self, '_ser') and self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TofSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
