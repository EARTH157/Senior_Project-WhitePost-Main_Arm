import rclpy
from rclpy.node import Node
from gpiozero import Button

class LimitSwitchNode(Node):
    def __init__(self):
        super().__init__('limit_switch_node')
        
        self.limit_pins = {
            'Limit_1': 18,
            'Limit_2': 23,
            'Limit_3': 24,
            'Limit_4': 25,
            'Limit_5': 16,
        }
        
        self.switches = {}
        self.switch_states = {} # เก็บสถานะเก่าไว้เทียบว่ามีการเปลี่ยนแปลงไหม

        # Setup GPIO
        for name, pin in self.limit_pins.items():
            # ไม่ใช้ bounce_time ในนี้ เพราะเราจะคุมจังหวะเองด้วย Timer
            btn = Button(pin, pull_up=True) 
            self.switches[name] = btn
            self.switch_states[name] = False # เริ่มต้นสมมติว่ายังไม่กด
            
            self.get_logger().info(f'Setup {name} on GPIO {pin}')

        # สร้าง Timer ให้ทำงานทุกๆ 0.05 วินาที (20 Hz)
        # ถ้า Noise มาสั้นกว่า 0.05 วิ โค้ดนี้จะมองไม่เห็น (ซึ่งดี!)
        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        for name, btn in self.switches.items():
            # อ่านค่าปัจจุบัน (True = กดอยู่, False = ไม่กด)
            is_pressed_now = btn.is_pressed
            
            # ตรวจสอบว่าสถานะเปลี่ยนไปจากรอบที่แล้วหรือไม่
            if is_pressed_now != self.switch_states[name]:
                
                if is_pressed_now:
                    # ถ้าเปลี่ยนเป็นกด -> แจ้งเตือน
                    self.get_logger().info(f'{name} : PRESSED (Detected)')
                    # TODO: ส่งคำสั่งหยุดหุ่นยนต์ที่นี่
                else:
                    # ถ้าเปลี่ยนเป็นปล่อย -> แจ้งเตือน
                    self.get_logger().info(f'{name} : RELEASED')

                # อัปเดตสถานะล่าสุดเก็บไว้
                self.switch_states[name] = is_pressed_now

def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()