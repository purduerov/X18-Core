#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from shared_msgs.msg import CanMsg, FinalThrustMsg, TempMsg

VALUE_DIFF = 1

# Currently testing values are put in such that there are two boards each with four thrusters
can_ids = [0x201, 0x201, 0x203, 0x202, 0x202, 0x203, 0x203, 0x202]  # can IDs for esc
can_pos = [5, 6, 7, 5, 6, 4, 5, 7]  # positions in data packet

can_better_map = {  # TODO
    0x201: [3, 7, 1, 5],
    0x202: [8, 4, 6, 2],
    0x203: [0, 0, 0, 0]
}


class ThrustToCanNode(Node):
    can_pow = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral
    can_last = {
        0x201: None,
        0x202: None,
        0x203: None
    }

    def __init__(self):
        super().__init__('thrust_to_can')

        # Publishers to the CAN hardware transmitter
        self.can_pub = self.create_publisher(CanMsg, 'can_tx', 10)

        # Subscribe to final_thrust and start callback function
        self.sub = self.create_subscription(
            FinalThrustMsg,
            'final_thrust',
            self.message_received,
            10
        )

    def message_received(self, msg):  # called in subscription object initialization
        # Separate FinalThrustMsg
        self.can_pow = msg.thrusters  # defines can_pow as type uint8 (thrust vector)

        base_board = min(can_ids)
        max_board = max(can_ids)

        for cid in range(base_board, max_board + 1):
            data_list = 0
            board = can_better_map[cid]

            for thruster in board:
                if thruster:
                    data_list += self.can_pow[thruster - 1]
                else:
                    data_list += 127
                # shifts thrust vector to correct position in uint64 message
                data_list = data_list << 8

            data_list = data_list >> 8

            # Publish Message
            new_msg = CanMsg()  # defines new msg as struct with uint32 for id and uint64 for data
            new_msg.id = cid  # sets id for new message
            new_msg.data = int(data_list)  # sets data from thrusters in new message

            if self.check_change(new_msg):  # if the message has changed publish the new message
                self.can_pub.publish(new_msg)  # uses publishing object to send updated message to can

    def check_change(self, new_msg):
        print("{}:".format(new_msg.id))  # prints CAN ID for new message
        if self.can_last[new_msg.id] is None:  # if msg ID has not been changed return true
            print("New message, true")
            self.can_last[new_msg.id] = new_msg  # sets last msg to new_msg so it can check for change in next pass
            return True

        else:
            test = new_msg.data
            comp = self.can_last[new_msg.id].data
            for i in range(4):
                a = abs(((test >> 8 * i) & 0xff) - ((comp >> 8 * i) & 0xff))

                if a > VALUE_DIFF:  # if different in past and current values is significant, updates to new value
                    self.can_last[new_msg.id] = new_msg
                    print("  changed")
                    return True

            print("  not changed")
            return False  # if value is unchanged, returns false to skip publish


def main(args=None):
    rclpy.init(args=args)
    node = ThrustToCanNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
