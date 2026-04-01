#! /usr/bin/python3
# import RPi.GPIO as GPIO
import lgpio as lg
# Change this to the actual pin that will be used for the leak sensor
INPUT_PIN = 12

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LeakSense(Node):
    def __init__(self):
        super().__init__("leak_sensor")
        self.publisher_ = self.create_publisher(Bool, "leak_sensor", 10)

        timer_period = 1
        self.time = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("begin gpio init")
        self.gpio_handle = lg.gpiochip_open(4)
        self.get_logger().info(f"Handle {self.gpio_handle} : Gpiochip data{lg.gpio_get_chip_info(self.gpio_handle)}")
        lg.gpio_claim_input(self.gpio_handle, INPUT_PIN)
        self.get_logger().info(f"Gpio mode{lg.gpio_get_mode(self.gpio_handle, INPUT_PIN)}")
        
        


    def timer_callback(self):
        msg = Bool()
        leak_status = lg.gpio_read(self.gpio_handle, INPUT_PIN)
        msg.data = bool(leak_status)
        self.get_logger().info(f"Leak status: {leak_status}")
        self.publisher_.publish(msg)
    
    def close_gpio(self):
        lg.gpio_free(self.gpio_handle, INPUT_PIN)
        lg.gpiochip_close(0)


def main(args=None):
    rclpy.init(args=args)
    leak_sense = LeakSense()
    try:
        rclpy.spin(leak_sense)
    except KeyboardInterrupt:
        leak_sense.close_gpio()
        rclpy.shutdown()
        leak_sense.destroy_node()
    


if __name__ == "__main__":
    main()

    
