#! /usr/bin/python3
import sys
import can
import rclpy
from rclpy.node import Node
from shared_msgs.msg import CanMsg

# can_bus - This is a ROS node that handles all CAN hardware communication
#           Arguments: can_bus.py accepts one optional argument: the CAN interface name. Usually can0 or vcan0.
#
#           can_bus reads on the can_tx topic and writes contents to pi CAN line (or vcan)
#           can_bus reads the pi CAN line (or vcan) and writes contents to can_rx topic.

global node
global can_bus
global pub
global sub
        
# Subscriber: Called when topic message is received
def topic_message_received(msg):
    # This runs on a seperate thread from the pub
    data_list = list()
    shift = 64
    for i in range(0, 8):
        shift -= 8
        data_list.append((msg.data >> shift) % 256)
    data = bytearray(data_list)
    node.get_logger().info('Topic Message Received: ' + str(msg.id) + ':' + str(list(data)))
    can_tx = can.Message(arbitration_id=msg.id, data=data, extended_id=False)
    try:
        can_bus.send(can_tx, timeout=1)
    except can.CanError as cerr:
        print("CAN TIMEOUT")
        pass


# Publisher: Called when can bus message is received
def bus_message_received(can_rx):
    data_list = list(can_rx.data)
    data = 0
    shift = len(data_list) * 8
    for i in data_list:
        shift -= 8
        data = data + (i << shift)
    node.get_logger().info('Can Message Received: ' + str(can_rx.arbitration_id) + ':' + str(list(can_rx.data)))
    can_rx = CanMsg(can_rx.arbitration_id, data)
    pub.publish(can_rx)

def timer_callback():
    for can_rx in can_bus:
        bus_message_received(can_rx)

if __name__ == "__main__":
    rclpy.init()

    node  = rclpy.create_node('can_node')
    #rospy.init_node('can_node')

    channel = 'can0'
    if len(sys.argv) == 2:
        channel = sys.argv[1]
    filters = []
    can_bus = can.interface.Bus(channel=channel, interface="socketcan", can_filters=filters)

    pub = node.create_publisher(CanMsg, 'can_rx', 100)
    #pub = rospy.Publisher('can_rx', can_msg, 100)
    sub = node.create_subscription(CanMsg, 'can_tx', topic_message_received, 100)
    #sub = rospy.Subscriber('can_tx', can_msg, topic_message_received)

    node.get_logger().info('Started \'can_node\' on channel: ' + channel)

    timer_period = 0.05
    node.create_timer(timer_period, timer_callback)
    # Performs publishing on can bus readrom can import Bus
    #while rclpy.ok():
    #while not rospy.is_shutdown():
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
