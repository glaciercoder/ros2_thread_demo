import sys
from threading import Thread
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class MinimalClientSync(Node):

    def __init__(self):
        super().__init__('minimal_client_sync')
        cli_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = ReentrantCallbackGroup()
        self.cli = self.create_client(AddTwoInts, 'add_two_ints', callback_group=cli_cb_group)
        self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        return self.cli.call(self.req)
        # This only works because rclpy.spin() is called in a separate thread below.
        # Another configuration, like spinning later in main() or calling this method from a timer callback, would result in a deadlock.

    def _timer_cb(self):
        self.get_logger().info('Sending request')
        response = self.send_request()
        self.get_logger().info('Received response')

def main():
    rclpy.init()
    
    

    minimal_client = MinimalClientSync()
    # Deadlock if use single thread executor
    executor = SingleThreadedExecutor()

    # # Work when use multi thread executor
    # executor = MultiThreadedExecutor()

    executor.add_node(minimal_client)
    executor.spin()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()