import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from shared_msgs.msg import SensorCoordination
import ADT7410

DEPTH_COMPLETE = (False, True, False, True)
TEMP_READING = (False, False, True, False)
TEMP_COMPLETE = (False, False, True, True)

class HatTempSensor(Node):
    def __init__(self):
        super().__init__("hat_temp_sensor")
        self.publisher_ = self.create_publisher(Float32, "hat_temp", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sensor = ADT7410()
        self.sensor.init()

        self.coord_publisher_ = self.create_publisher(SensorCoordination, "sensor_coordination", 10)
        self.coord_subscriber = self.create_subscription(SensorCoordination, "sensor_coordination", self.coord_callback, 10)


    def timer_callback(self):
        if self.i2c_status == DEPTH_COMPLETE:
            msg = Float32()
            #coordination
            self.i2c_status = TEMP_READING
            self.coord_publisher_.publish(self.i2c_status)

            #read temperature
            msg.data = self.read_temperature()
            self.publisher_.publish(msg)
        elif self.i2c_status == TEMP_READING:
            self.i2c_status = TEMP_COMPLETE
            self.coord_publisher_.publish(self.i2c_status)


    def coord_callback(self, msg):
        self.i2c_status = tuple(msg.i2c_status)
 

def main(args=None):
    rclpy.init(args=args)
    hat_temp_sensor = HatTempSensor()
    rclpy.spin(hat_temp_sensor)
    hat_temp_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
