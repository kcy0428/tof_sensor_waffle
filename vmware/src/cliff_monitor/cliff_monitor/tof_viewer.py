#!/usr/bin/env python3
"""터미널에서 ToF 8x8 그리드를 실시간으로 표시"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

RESET = '\033[0m'
OOR   = -1.0   # Out-of-Range 마커
N     = 8

# JET colormap 방향: 낮은 norm(가까움) → 파랑, 높은 norm(멀음) → 빨강
# (상한 norm, 배경색, 글자색)
GRADIENT = [
    ( 32, '\033[48;5;21m',  '\033[97m'),  # 진파랑
    ( 64, '\033[48;5;27m',  '\033[97m'),  # 파랑
    ( 96, '\033[48;5;39m',  '\033[97m'),  # 하늘
    (128, '\033[48;5;46m',  '\033[30m'),  # 초록
    (160, '\033[48;5;226m', '\033[30m'),  # 노랑
    (192, '\033[48;5;208m', '\033[97m'),  # 주황
    (224, '\033[48;5;202m', '\033[97m'),  # 주황빨
    (256, '\033[48;5;196m', '\033[97m'),  # 빨강
]

OOR_CELL = f'\033[48;5;240m\033[97m {"----":>5} {RESET}'  # 회색 배경

# 테두리 (ANSI 코드 없음 → 길이 정확)
_bar = '──────'
TOP  = '    ┌' + '┬'.join(_bar for _ in range(N)) + '┐'
HDR  = '    │' + '│'.join(f'  c{i}   ' if i < N - 1 else f'  c{i}  ' for i in range(N)) + '│'
DIV  = '    ├' + '┼'.join(_bar for _ in range(N)) + '┤'
BOT  = '    └' + '┴'.join(_bar for _ in range(N)) + '┘'


def _pick_color(norm: float):
    for threshold, bg, fg in GRADIENT:
        if norm < threshold:
            return bg, fg
    return GRADIENT[-1][1], GRADIENT[-1][2]


def _cell(norm: float, val: float) -> str:
    """가시 너비 6인 색상 셀: ' NNNN '"""
    bg, fg = _pick_color(norm)
    return f'{bg}{fg}{int(val):5d} {RESET}'


def _legend(mn: float, mx: float) -> str:
    steps = 8
    parts = []
    for i in range(steps):
        norm = i / (steps - 1) * 255
        dist = mn + (mx - mn) * (i / (steps - 1))
        bg, fg = _pick_color(norm)
        parts.append(f'{bg}{fg}{int(dist):5d}{RESET}')
    return ' min ' + '→'.join(parts) + ' max  (mm)'


class TofViewer(Node):
    def __init__(self):
        super().__init__('tof_viewer')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32MultiArray, '/tof/grid', self._cb, qos)
        sys.stdout.write('\033[2J')
        sys.stdout.flush()

    def _cb(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) != 64:
            return

        # OOR(-1.0) 제외한 유효값으로만 min/max 계산
        valid = [v for v in data if v != OOR]
        if not valid:
            return

        mn  = min(valid)
        mx  = max(valid)
        rng = mx - mn if mx != mn else 1.0

        def norm(v):
            return (v - mn) / rng * 255

        out = ['\033[H',
               f'  ToF 8×8 Distance Map    min={int(mn)} mm  max={int(mx)} mm',
               '',
               TOP,
               HDR,
               DIV]

        for row in range(N):
            cells = '│'.join(
                OOR_CELL if data[row * N + col] == OOR
                else _cell(norm(data[row * N + col]), data[row * N + col])
                for col in range(N)
            )
            out.append(f'r{row}  │{cells}│')
            if row < N - 1:
                out.append(DIV)

        out.append(BOT)
        out.append('')
        out.append(_legend(mn, mx))
        out.append('')

        sys.stdout.write('\n'.join(out))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TofViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
