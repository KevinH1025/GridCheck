from .main_control import MainControl

class HoverController:
    def __init__(self, node:MainControl, duration, mission_len):
        self.node = node
        self.timer = node.create_timer(duration, self.done)
        self.len = mission_len

    def done(self):
        self.node.wp_index += 1
        if self.node.wp_index >= self.len:
            self.node.state = 'DONE'
        else:
            self.node.transition("OFFBOARD")
        self.timer.cancel()