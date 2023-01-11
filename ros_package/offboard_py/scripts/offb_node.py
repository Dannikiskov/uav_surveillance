#! /usr/bin/env python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""

import rospy, socket, threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class Drone:
    def __init__(self):

        self.current_state = State()

        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)

        rospy.init_node("offb_node_py")

        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)

        # self.image_sub = rospy.Subscriber('iris/camera_link/down_raw_image')

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        self.offb_set_mode = SetModeRequest()

        self.arm_cmd = CommandBoolRequest()

        host = '127.0.0.1'
        port = 5000
        self.client = NetworkClient(host, port)

    def state_cb(self, msg):
        self.current_state = msg

    def local_position_callback(self, data):
        self.local_position = data

    def start(self):
        pose = PoseStamped()

        pose.pose.position.x = 20
        pose.pose.position.y = 20
        pose.pose.position.z = 20

        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd.value = True

        last_req = rospy.Time.now()

        next_mission = 10

        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(self.offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arming_client.call(self.arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            if (rospy.Time.now() - last_req) > rospy.Duration(
                    5.0) and 8 <= self.local_position.pose.position.x <= 11 and 8 <= self.local_position.pose.position.y <= 11 and 8 <= self.local_position.pose.position.z <= 11 and 8 <= self.local_position.pose.position.z <= 11:
                pose.pose.position.x = 20
                pose.pose.position.y = 20
                pose.pose.position.z = 20
                if next_mission == 10:
                    msg = "reached 10"
                    rospy.loginfo(msg)
                    last_req = rospy.Time.now()
                    self.client.send_message(msg)
                next_mission = 20
            elif (rospy.Time.now() - last_req) > rospy.Duration(
                    5.0) and 18 <= self.local_position.pose.position.x <= 21 and 18 <= self.local_position.pose.position.y <= 21 and 18 <= self.local_position.pose.position.z <= 21 and 18 <= self.local_position.pose.position.z <= 21:
                pose.pose.position.x = 10
                pose.pose.position.y = 10
                pose.pose.position.z = 10
                if next_mission == 20:
                    msg = "reached 20"
                    rospy.loginfo(msg)
                    last_req = rospy.Time.now()
                    self.client.send_message(msg)
                next_mission = 10
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        # disconnect from server
        self.client.close()


class NetworkClient(threading.Thread):
    def __init__(self, host, port):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        # Create a client socket
        self.client = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        # Connect it to the server
        self.client.connect((host, port))

        self.inbox_empty = True
        self.inbox = []

    # Send message to server
    def send_message(self, message):
        self.client.send(message.encode())

    def recieve_message(self):
        # Receive message string from server, at a time 1024 B
        message = self.client.recv(1024)
        while message:
            print('Received: ' + message.decode())
        self.inbox.append(message)
        self.inbox_empty = False

    # Close connection
    def close(self):
        self.client.close()


def main():
    drone = Drone()
    drone.start()


if __name__ == '__main__':
    main()

