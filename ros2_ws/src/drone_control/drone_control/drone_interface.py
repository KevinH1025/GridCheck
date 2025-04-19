from rclpy.node import Node
from rclpy.task import Future
from mavros_msgs.srv import SetMode, CommandBool

class DroneInterface():
    def __init__(self, node:Node):
        self.arm_client = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = node.create_client(SetMode, '/mavros/set_mode')

    def arm(self, node:Node):
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for arming service...')

        req = CommandBool.Request(value=True)
        send_req = self.arm_client.call_async(req)

        def callback(future:Future):
            if future.result().success:
                node.get_logger().info('Drone armed successfully.')
            else:
                node.get_logger().warn('Arming failed.')

        send_req.add_done_callback(callback)

    def set_mode(self, node:Node, mode):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for mode service...')
            
        req = SetMode.Request(custom_mode=mode)
        send_req = self.mode_client.call_async(req)

        def callback(future:Future):
            if future.result().mode_sent:
                node.get_logger().info(f'Mode changed to {mode}.')
            else:
                node.get_logger().warn(f'Mode change to {mode} failed.')

        send_req.add_done_callback(callback)