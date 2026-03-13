#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import RPi.GPIO as GPIO
import time
import signal
import sys

class StepperNode(Node):
    def __init__(self):
        super().__init__('stepper_joint3')

        # ---------- Parameters (ปรับได้จาก launch/CLI) ----------
        self.declare_parameter('pin_ena', 22)     # ENA+ (ENA- → GND)
        self.declare_parameter('pin_dir', 27)      # DIR+ (DIR- → GND)
        self.declare_parameter('pin_pul', 17)      # PUL+ (PUL- → GND)
        self.declare_parameter('ena_active_high', False)   # บางไดรเวอร์ Enable = LOW
        self.declare_parameter('step_delay', 0.0010)      # หน่วงต่อครึ่งคาบ (ควบคุมความเร็ว)
        self.declare_parameter('test_on_start', False)     # ทดสอบหมุนตอนเริ่ม

        self.PIN_ENA = int(self.get_parameter('pin_ena').value)
        self.PIN_DIR = int(self.get_parameter('pin_dir').value)
        self.PIN_PUL = int(self.get_parameter('pin_pul').value)
        self.ENA_HIGH = bool(self.get_parameter('ena_active_high').value)
        self.step_delay = float(self.get_parameter('step_delay').value)
        self.test_on_start = bool(self.get_parameter('test_on_start').value)

        # ---------- GPIO init ----------
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN_ENA, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.PIN_DIR, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.PIN_PUL, GPIO.OUT, initial=GPIO.LOW)

        self.enable_motor(True)

        # ---------- Subscriber ----------
        # ส่งจำนวนก้าวที่ topic /stepper_cmd (Int32), + หมุนไปข้างหน้า, - ถอยหลัง
        self.sub = self.create_subscription(Int32, '/stepper_joint3_cmd', self.cmd_cb, 10)

        self.get_logger().info(
            f'GPIO pins → ENA={self.PIN_ENA}, DIR={self.PIN_DIR}, PUL={self.PIN_PUL} | '
            f'ENA active {"HIGH" if self.ENA_HIGH else "LOW"} | step_delay={self.step_delay}s'
        )

        if self.test_on_start:
            self.get_logger().info('Running startup test: 800 steps forward/backward...')
            # เดินหน้า
            GPIO.output(self.PIN_DIR, GPIO.HIGH)
            self.step(800, self.step_delay)
            time.sleep(0.3)
            # ถอยหลัง
            GPIO.output(self.PIN_DIR, GPIO.LOW)
            self.step(800, self.step_delay)
            self.get_logger().info('Startup test done.')

    # ---------- Core control ----------
    def enable_motor(self, enable: bool):
        """เปิด/ปิดการควบคุมมอเตอร์ (ตามโพลาริตี้ของ ENA)"""
        level = GPIO.HIGH if (enable == self.ENA_HIGH) else GPIO.LOW
        GPIO.output(self.PIN_ENA, level)

    def step(self, n_steps: int, delay: float):
        """ส่งพัลส์ n_steps ครั้ง, delay = หน่วงต่อครึ่งคาบ (ขึ้น/ลง)"""
        # safety: delay ต้องมากพอ ไม่งั้นมอเตอร์อาจไม่รับพัลส์
        d = max(delay, 0.00001)  # 50 µs ต่ำสุดแบบระวัง ๆ (ขึ้นกับไดรเวอร์/โหลดจริง)
        for _ in range(n_steps):
            GPIO.output(self.PIN_PUL, GPIO.HIGH)
            time.sleep(d)
            GPIO.output(self.PIN_PUL, GPIO.LOW)
            time.sleep(d)

    # ---------- ROS callback ----------
    def cmd_cb(self, msg: Int32):
        steps = int(msg.data)
        if steps == 0:
            self.get_logger().info('Received 0 steps: do nothing.')
            return

        # กำหนดทิศ
        if steps > 0:
            GPIO.output(self.PIN_DIR, GPIO.HIGH)
        else:
            GPIO.output(self.PIN_DIR, GPIO.LOW)
            steps = -steps

        self.get_logger().info(f'Executing move: {steps} steps, delay={self.step_delay}')
        try:
            self.step(steps, self.step_delay)
            self.get_logger().info('Move done.')
        except Exception as e:
            self.get_logger().error(f'Error while stepping: {e}')

    # ---------- Cleanup ----------
    def destroy_node(self):
        try:
            self.enable_motor(False)  # ปิด enable ก่อน
        except Exception:
            pass
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()

    node = StepperNode()

    # จัดการสัญญาณ Ctrl+C ให้ cleanup สวย ๆ
    def _sigint_handler(signum, frame):
        node.get_logger().info('SIGINT received, shutting down...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
