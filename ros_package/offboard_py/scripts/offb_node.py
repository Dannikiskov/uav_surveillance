#! /usr/bin/env python3

import rospy, socket, threading, ast, cv2, select, math
from geometry_msgs.msg import PoseStamped, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
from people_detector import PeopleDetector


class Drone:
    def __init__(self):

        # Initialize the detector
        self.people_detector = PeopleDetector()

        self.current_state = State()

        # Saves current GPS loctation
        self.gps_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.gps_cb)

        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_cb)

        rospy.init_node('offb_node_py')

        self.state_sub = rospy.Subscriber('mavros/state', State, callback=self.state_cb)

        # People detector is deactivated until it reaches its first waypoint in its flight route
        self.people_detector_activated = False
        # Camera feed
        self.image_sub = rospy.Subscriber('iris/camera_link/down_raw_image', Image, self.image_cb)

        # Publisher to set waypoint
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Setting setpoint publishing rate
        self.rate = rospy.Rate(20)

        # Waiting for flight controller to connect
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        self.offb_set_mode = SetModeRequest()

        self.arm_cmd = CommandBoolRequest()

        # Base location
        self.HOME_COORDINATES = [0, 0, 4]

        # Setting up client to communicate with the server
        HOST = 'localhost'
        PORT = 8888
        self.client = NetworkClient(HOST, PORT)

    def state_cb(self, msg):
        self.current_state = msg

    # Camera feed that detects people, if it the detector is activated
    def image_cb(self, image):
        if self.people_detector_activated:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')
            # Gathering the boundingboxes for the people in the image
            boundingboxes = self.people_detector.detect(cv_image)
            # Sending the boundingboxes and the GPS location, where they were found
            self.client.outbox.append(str([self.gps_location.latitude, self.gps_location.longitude]) + ', ' + str(boundingboxes))

    def gps_cb(self, gps_location):
        self.gps_location = gps_location

    def local_position_cb(self, data):
        self.local_position = data

    # Returns True if the drone is at the given location, within a threshold
    def at_location(self, location, threshold=0.2):
        return location[0] - threshold <= self.local_position.pose.position.x <= location[0] + threshold and \
               location[1] - threshold <= self.local_position.pose.position.y <= location[1] + threshold and \
               location[2] - threshold <= self.local_position.pose.position.z <= location[2] + threshold

    def start(self):
        # Start the inbox and outbox features of the client
        self.client.start()

        pose = PoseStamped()
        pose.pose.position.x = self.HOME_COORDINATES[0]
        pose.pose.position.y = self.HOME_COORDINATES[1]
        pose.pose.position.z = self.HOME_COORDINATES[2]

        # Sending a few setpoints to home base before starting
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd.value = True

        last_req = rospy.Time.now()

        waypoints = []

        # Turning is used is indicate when the drone is turning, to turn of the people detector while turning.
        # Otherwise it could detect people from far away, when turning
        turning = True

        # Main logic loop that runs until connection to the server closes
        while self.client.connected:
            if rospy.is_shutdown():
                break
            # Initializing offboard mode
            if self.current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(self.offb_set_mode).mode_sent:
                    rospy.loginfo('OFFBOARD enabled')
                last_req = rospy.Time.now()
            # Arming the drone
            elif not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.arming_client.call(self.arm_cmd).success:
                    rospy.loginfo('Vehicle armed')
                    # Once it is in offboard mode and armed, it can listen for its GPS location,
                    # to set its saved home GPS location
                    self.HOME_COORDINATES_GPS = [self.gps_location.latitude, self.gps_location.longitude]
                last_req = rospy.Time.now()

            # Read the client inbox for a new flight route
            if self.client.inbox:
                for _ in range(len(self.client.inbox)):
                    message = self.client.inbox.pop(0)
                    message_split = message.split('=')
                    if message_split[0] == 'route':
                        route = ast.literal_eval(message_split[1])
                        # Transform the given GPS waypoint to local positions instead, by using the home GPS location
                        route = [[111319.5 * math.cos(0.01745329 * self.HOME_COORDINATES_GPS[0]) * lon - 111319.5 * math.cos(0.01745329 * self.HOME_COORDINATES_GPS[0]) * self.HOME_COORDINATES_GPS[1], (lat - self.HOME_COORDINATES_GPS[0]) * 111319.5, self.HOME_COORDINATES[2]] for [lat, lon] in route]
                        waypoints += route
            # If it has a flight route, then it sets its next setpoint to be the next waypoint in the route
            if waypoints:
                pose.pose.position.x = waypoints[0][0]
                pose.pose.position.y = waypoints[0][1]
                # Case it has reached the next waypoint
                if (rospy.Time.now() - last_req) > rospy.Duration(5.0) and self.at_location(waypoints[0]):
                    # Switch the turning mode, because at every second waypoint, the drone is turning
                    turning = not turning
                    if not turning:
                        self.people_detector_activated = True
                    elif turning:
                        self.people_detector_activated = False
                    waypoint = waypoints.pop(0)
                    rospy.loginfo('Waypoint ' + str(waypoint) + ' reached')
                # If it has reached all of its waypoints, then it sets the next setpoint to its home
                if not waypoints:
                    self.people_detector_activated = False
                    pose.pose.position.x = self.HOME_COORDINATES[0]
                    pose.pose.position.y = self.HOME_COORDINATES[1]
                    pose.pose.position.z = self.HOME_COORDINATES[2]
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
            # Check that the connection to the server is still alive
            self.client.outbox.append('Connection check')
        # Close client network
        self.client.close()

        # Sets next setpoint to home
        pose.pose.position.x = self.HOME_COORDINATES[0]
        pose.pose.position.y = self.HOME_COORDINATES[1]
        pose.pose.position.z = self.HOME_COORDINATES[2]

        # Travels to home location
        while not rospy.is_shutdown() and not self.at_location(self.HOME_COORDINATES):
            if self.current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(self.offb_set_mode).mode_sent:
                    rospy.loginfo('OFFBOARD enabled')
                last_req = rospy.Time.now()
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        # Once home location is reached it lands
        self.offb_set_mode.custom_mode = 'AUTO.LAND'
        while not rospy.is_shutdown():
            if self.current_state.mode != 'AUTO.LAND' and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(self.offb_set_mode).mode_sent:
                    rospy.loginfo('AUTO.LAND enabled')
                last_req = rospy.Time.now()
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        # Disconnect from server if it is still connected
        if self.client.connected:
            self.client.close()


# Networking thread that saves incoming messages to the inbox
class Receiver(threading.Thread):
    def __init__(self, network):
        threading.Thread.__init__(self)
        self.network = network
        self.alive = True

    def run(self):
        try:
            # Keep listening for messages, while it is alive
            while self.alive:
                ready = select.select([self.network.client], [], [])
                if ready[0]:
                    message = self.network.client.recv(2048)
                    self.network.inbox.append(str(message.decode()))
        except socket.error as e:
            rospy.loginfo('Connection to the server lost')
            self.network.connected = False


# Networking thread that send messages from the outbox
class Sender(threading.Thread):
    def __init__(self, network):
        threading.Thread.__init__(self)
        self.network = network
        self.alive = True

    def run(self):
        try:
            # Keep sending, while it is alive
            while self.alive:
                if self.network.outbox:
                    self.network.client.send(self.network.outbox.pop(0).encode())
        except socket.error as e:
            rospy.loginfo('Connection to the server lost')
            self.network.connected = False


# A network wrapper class, such that networking is handled in a multithreaded way
class NetworkClient:

    def __init__(self, host, port):

        self.inbox = []
        self.outbox = []

        self.host = host
        self.port = port
        # Create a client socket
        self.client = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        # Connect it to the server
        self.connected = False
        while not self.connected:
            try:
                self.client.connect((host, port))
                self.connected = True
                self.client.send('drone'.encode())
            except socket.error as e:
                pass

        self.sender = Sender(self)
        self.receiver = Receiver(self)

    # Start sender and receiver threads
    def start(self):
        self.sender.start()
        self.receiver.start()

    # Close connection
    def close(self):
        # Close threads
        self.sender.alive = False
        self.receiver.alive = False
        self.sender.join()
        self.receiver.join()

        # Close connection if it is still alive
        if self.connected:
            self.client.close()
            self.connected = False
        rospy.loginfo('Closing network client')


def main():
    drone = Drone()
    drone.start()


if __name__ == '__main__':
    main()

