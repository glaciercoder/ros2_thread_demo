import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        cli_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(AddTwoInts, 'add_two_ints', callback_group=cli_cb_group)
        self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)

        self.response = None
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

    def _timer_cb(self):
        self.get_logger().info('Sending request')
        future = self.send_request(int(sys.argv[1]), int(sys.argv[2]))
        future.add_done_callback(self.future_done_cb)
        if self.response:
            self.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (int(sys.argv[1]), int(sys.argv[2]), self.response.sum))

    def future_done_cb(self, future):
        self.response = future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)  

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()