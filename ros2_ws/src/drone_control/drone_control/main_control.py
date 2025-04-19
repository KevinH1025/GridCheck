import rclpy
from enum import Enum
from rclpy.node import Node
from .mission_publisher import MissionPublisher 
from .hover_control import HoverController
from .state_monitor import StateMonitor
from .drone_interface import DroneInterface
from .utils import load_mission, convert_mission, reached
from .config import TAKEOFF_THRESHHOLD, HOVER_DURATION, CONTROL_LOOP_RATE

class MainControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        self.state_monitor = StateMonitor(self)
        self.mission_pub = MissionPublisher(self)
        self.drone_interface = DroneInterface(self)
        self.mission = load_mission('mission.yaml')

        self.state = 'INIT'
        self.wp_index = -1
        self.wp_list = []

        self.timer = self.create_timer(CONTROL_LOOP_RATE, self.loop)

    def loop(self):
        if self.state == 'INIT':
            self.transition("ARM")

        elif self.state == 'ARM':
            self.transition("AUTO.TAKEOFF")

        elif self.state == 'TAKEOFF':
            if self.state_monitor.rel_alt >= TAKEOFF_THRESHHOLD:
                self.wp_list = convert_mission(self.state_monitor.curr_lat, self.state_monitor.curr_lon, self.mission)
                self.transition("AUTO.LOITER")
                HoverController(self, HOVER_DURATION, len(self.wp_list))
            else:
                pass

        # This part will be repeated until all waypoints are visited
        elif self.state == 'HOVER':
            wp = self.wp_list[self.wp_index]
            self.mission_pub.publish_coordinates(*wp)

        elif self.state == 'OFFBOARD':
            wp = self.wp_list[self.wp_index]
            self.mission_pub.publish_coordinates(*wp)
            if reached(wp, self.state_monitor.curr_lat, self.state_monitor.curr_lon, self.state_monitor.rel_alt):
                self.transition("AUTO.LOITER")
                HoverController(self, HOVER_DURATION, len(self.wp_list))

        elif self.state == 'DONE':
            self.transition("AUTO.RTL")

    # -------helper functions--------
    def transition(self, state):
        if state == "ARM":
            self.drone_interface.arm(self)
        else:
            self.drone_interface.set_mode(self, state)
            
        self.print_mode(state)
        self.state = state

    def print_mode(self, mode):
        print(f"[MODE] -> {mode}")

# Is part of the templet (required)
def main(args=None):
    rclpy.init(args=args)
    node = MainControl()  # Create DroneControl node
    rclpy.spin(node)  # Keep the node alive

if __name__ == '__main__':
    main()