#! /usr/bin/python3
import rclpy
import rclpy.node as Node
from shared_msgs.msg import CanMsg, FinalThrustMsg, TempMsg

VALUE_DIFF = 1

# Currently testing values are put in such that there are two boards each with four thrusters
global can_pub
global node
can_ids = [0x201, 0x201, 0x203, 0x202, 0x202, 0x203, 0x203, 0x202]  # can IDs for esc
can_pos = [5, 6, 7, 5, 6, 4, 5, 7]  # positions in data packet

can_better_map = {
    0x201: [ 3, 7, 1, 5 ],
    0x202: [ 8, 4, 6, 2 ],
    0x203: [ 0, 0, 0, 0 ]
}

can_pow = [127,127,127,127,127,127,127,127]  # power of thrusters --> 127 is neutral
can_last = {
    0x201: None,
    0x202: None,
    0x203: None
}


def message_received(msg): #called in subscription object initialization
    global can_pub
    global can_ids
    global can_pos
    global can_pow
    global can_last
    global can_better_map

    # Seperate FinalThrustMsg
    can_pow = msg.thrusters #defines can_pow as type uint8 (thrust vector)

    base_board = min(can_ids)
    max_board = max(can_ids)

    for cid in range(base_board, max_board + 1):
        data_list = 0
        board = can_better_map[cid]

        for thruster in board:
            if thruster:
                data_list += can_pow[thruster - 1]
            else:
                data_list += 127
            #shifts thrust vector to correct position in uint64 message
            data_list = data_list << 8 

        data_list = data_list >> 8


        # Publish Message
        new_msg = CanMsg() #defines new msg as struct with uint32 for id and uint64 for data
        new_msg.id = cid  #sets id for new message
        new_msg.data = int(data_list) #sets data from thrusters in new message
        
        if checkChange(new_msg): #if the message has changed publish the new message
            can_pub.publish(new_msg) #uses publishing object to send updated message to can
    pass

def checkChange(new_msg):
    global can_last
    
    print("{}:".format(new_msg.id)) #prints CAN ID for new message
    if can_last[new_msg.id] is None: #if msg ID has not been changed return true
        print("New message, true")
        can_last[new_msg.id] = new_msg #sets last msg to new_msg so it can check for change in next pass
        return True

    else:
        test = new_msg.data
        comp = can_last[new_msg.id].data
        for i in range(4):
            a = abs(((test >> 8 * i) & 0xff) - ((comp >> 8 * i) & 0xff))

            if a > VALUE_DIFF: #if different in past and current values is significant, updates to new value
                can_last[new_msg.id] = new_msg
                print("  changed")
                return True

        print("  not changed")
        return False #if value is unchanged, returns false to skip publish

if __name__ == "__main__":
    rclpy.init() #initializes ros communication
    node = rclpy.create_node('thrust_to_can') #creates an instance of a node with name thrust_proc

    # Publishers to the CAN hardware transmitter
    can_pub = node.create_publisher(CanMsg, 'can_tx', 10) 

    # Subscribe to final_thrust and start callback function
    sub = node.create_subscription(
                                    FinalThrustMsg, 
                                    'final_thrust',
                                    message_received, 
                                    10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()