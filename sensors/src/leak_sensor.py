#! /usr/bin/python3
# import RPi.GPIO as GPIO
import lgpio as lg
# Change this to the actual pin that will be used for the leak sensor
INPUT_PIN = 12
handle = -1
# Set this to true if you want to run the node without using ros
ROSless = False

if not ROSless:
    import rclpy
    from std_msgs.msg import Bool
else:
    from time import sleep


def setup():
    global pub, node
    # Try to setup ROS node
    if not ROSless:
        try:
            rclpy.init()
            node = rclpy.create_node("leak_sensor")
            pub = node.create_publisher(Bool, "leak_sensor", 10)
            data_thread = node.create_timer(1, pub_data)
        except:
            print("Error initializing ROS")
            return False

    # Try to setup GPIO
    try:
        #change library
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(INPUT_PIN, GPIO.IN)
        handle = lg.gpiochip.open(0)
        lg.gpio_claim_input(handle, INPUT_PIN)

    except:
        print("Error initializing GPIO")
        return False
    return True


def pub_data():
    # leak_status = GPIO.input(INPUT_PIN)
    leak_status = lg.gpio_read(handle, INPUT_PIN)
    new_msg = Bool()
    new_msg.data = leak_status == 1
    pub.publish(new_msg)


if __name__ == "__main__":
    if not setup():
        print("Setup failed. Exiting.")
        exit()

    if not ROSless:
        try:
            rclpy.spin(node)
        except Exception as e:
            print(e)
            print("Exiting")
        finally:
            # GPIO.cleanup()
            lg.gpio_close()
            rclpy.shutdown()

    else:
        while True:
            # Print the status of the sensor
            print(lg.gpio_read(INPUT_PIN))
            sleep(0.5)
