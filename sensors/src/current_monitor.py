try:
    import smbus2 as smbus
except:
    print("Try sudo apt-get install python-smbus2")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class INA260(object):
    # TODO: Change the bus number and address as well
    def __init__(self, bus_number=1, address=0x40):
        self.bus = smbus.SMBus(bus_number)
        self.address = address

    def read_current(self):
        # Placeholder for reading current from the sensor
        # Replace with actual register addresses and conversion logic
        raw_data = self.bus.read_word_data(self.address, 0x00)
        current = self._convert_raw_to_current(raw_data)
        return current

    def _convert_raw_to_current(self, raw_data):
        # Placeholder conversion logic
        # TODO: change the conversion factor
        return raw_data * 0.1  # Example conversion factor

class CurrentMonitor(Node):
    def __init__(self):
        super().__init__("current_monitor")
        self.publisher_ = self.create_publisher(Float64, "current", 10)
        timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        try:
            self.sensor = INA260()  # Initialize INA260 sensor
            self.get_logger().info("Current sensor connected")
        except Exception as e:
            self.get_logger().info(f"Current sensor not found: {e}")
            self.sensor = None
            exit(1)

    def timer_callback(self):
        msg = Float64()
        msg.data = self.sensor.read_current()

        # publishes current in Amperes at a rate of 1Hz
        self.publisher_.publish(msg)
        self.get_logger().info(f"Current: {round(msg.data, 3)} A")


def main(args=None):
    rclpy.init(args=args)
    current_monitor = CurrentMonitor()

    rclpy.spin(current_monitor)

    current_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()