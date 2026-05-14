#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool

CLIFF_ROWS = [5, 6, 7]      # 0-indexed, 하단 3행
CLIFF_DIST_MM = 800.0        # 이 값 이상이면 낭떠러지로 판단
MIN_PIXELS = 4               # 임계 픽셀 수 (8개 중 4개 이상)
DEBOUNCE_FRAMES = 3          # 연속 감지 프레임 수


class CliffDetectorNode(Node):
    def __init__(self):
        super().__init__('cliff_detector_node')

        self.declare_parameter('cliff_rows', CLIFF_ROWS)
        self.declare_parameter('cliff_dist_mm', CLIFF_DIST_MM)
        self.declare_parameter('min_pixels', MIN_PIXELS)
        self.declare_parameter('debounce_frames', DEBOUNCE_FRAMES)

        self._cliff_rows = self.get_parameter('cliff_rows').value
        self._dist_mm    = self.get_parameter('cliff_dist_mm').value
        self._min_pixels = self.get_parameter('min_pixels').value
        self._debounce   = self.get_parameter('debounce_frames').value

        self._consecutive  = 0
        self._cliff_active = False

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._sub = self.create_subscription(
            Float32MultiArray, '/tof/grid', self._grid_cb, sensor_qos)
        self._pub = self.create_publisher(Bool, '/cliff_detected', 10)

        self.get_logger().info(
            f'CliffDetector 시작: rows={self._cliff_rows}, '
            f'dist>={self._dist_mm}mm, pixels>={self._min_pixels}, '
            f'debounce={self._debounce}frames')

    def _grid_cb(self, msg):
        grid = list(msg.data)
        if len(grid) != 64:
            return

        count = 0
        for row in self._cliff_rows:
            for col in range(8):
                v = grid[row * 8 + col]
                if v < 0 or v >= self._dist_mm:  # OOR 또는 임계 이상
                    count += 1

        is_cliff = count >= self._min_pixels

        if is_cliff:
            self._consecutive += 1
        else:
            self._consecutive = 0

        if self._consecutive >= self._debounce and not self._cliff_active:
            self._cliff_active = True
            self.get_logger().warn(
                f'낭떠러지 감지! (임계픽셀={count}/{len(self._cliff_rows)*8})')
            self._pub.publish(Bool(data=True))

        elif not is_cliff and self._cliff_active:
            self._cliff_active = False
            self.get_logger().info('낭떠러지 해제')
            self._pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = CliffDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
