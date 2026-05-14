#!/usr/bin/env python3
import tempfile
import threading

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from gtts import gTTS


class CliffManagerNode(Node):
    def __init__(self):
        super().__init__('cliff_manager_node')

        pygame.mixer.init()

        self._audio = {}
        self._prepare_audio('detected', '낭떠러지가 감지되어서 우회하겠습니다')
        self._prepare_audio('cleared',  '낭떠러지가 해제되었습니다')
        self.get_logger().info('음성 파일 준비 완료')

        self._motor_cli = self.create_client(SetBool, '/motor_power')
        self.get_logger().info('/motor_power 서비스 대기 중...')
        while not self._motor_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/motor_power 서비스 연결 대기 중...')
        self.get_logger().info('/motor_power 서비스 연결됨')

        self._torque_on = True
        self.create_subscription(Bool, '/cliff_detected', self._cliff_cb, 10)
        self.get_logger().info('CliffManager 시작 — /cliff_detected 대기 중')

    def _prepare_audio(self, key: str, text: str):
        f = tempfile.NamedTemporaryFile(suffix='.mp3', delete=False)
        gTTS(text, lang='ko').save(f.name)
        self._audio[key] = f.name

    def _play(self, key: str):
        threading.Thread(target=self._play_worker, args=(key,), daemon=True).start()

    def _play_worker(self, key: str):
        pygame.mixer.music.load(self._audio[key])
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)

    def _cliff_cb(self, msg: Bool):
        if msg.data and self._torque_on:
            self.get_logger().warn('낭떠러지 감지 — DXL 토크 OFF')
            self._play('detected')
            self._set_motor_power(False)

        elif not msg.data and not self._torque_on:
            self.get_logger().info('낭떠러지 해제 — DXL 토크 ON')
            self._play('cleared')
            self._set_motor_power(True)

    def _set_motor_power(self, enable: bool):
        req = SetBool.Request()
        req.data = enable
        future = self._motor_cli.call_async(req)
        future.add_done_callback(lambda f: self._motor_done(f, enable))

    def _motor_done(self, future, enable: bool):
        try:
            res = future.result()
            self._torque_on = enable
            state = 'ON' if enable else 'OFF'
            self.get_logger().info(f'DXL 토크 {state}: {res.message}')
        except Exception as e:
            self.get_logger().error(f'motor_power 서비스 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CliffManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
