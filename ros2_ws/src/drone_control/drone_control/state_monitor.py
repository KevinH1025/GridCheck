from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node

class StateMonitor:
    def __init__(self, node:Node):
        self.curr_lat = None
        self.curr_lon = None
        self.rel_alt = 0
        self.mode = ""
        self.armed = False

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        node.create_subscription(State, '/mavros/state', self.state_cb, 10)
        node.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, qos)
        node.create_subscription(Altitude, '/mavros/altitude', self.alt_cb, qos)

    def state_cb(self, msg:State):
        self.armed = msg.armed
        self.mode = msg.mode

    def gps_cb(self, msg:NavSatFix):
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude

    def alt_cb(self, msg:Altitude):
        self.rel_alt = msg.relative