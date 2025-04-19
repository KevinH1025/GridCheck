from rclpy.node import Node
from mavros_msgs.msg import GlobalPositionTarget

class MissionPublisher:
    def __init__(self, node:Node):
    
        # publisher for GPS position -> tells the drone where to go
        self.setpoint_pub = node.create_publisher(GlobalPositionTarget, 
                                                  '/mavros/setpoint_raw/global', 
                                                  10)

    def publish_coordinates(self, lat, lon, alt):
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = 4088  # position only
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.setpoint_pub.publish(msg)
