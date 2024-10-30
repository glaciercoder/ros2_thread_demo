import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import asyncio

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        cli_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(AddTwoInts, 'add_two_ints', callback_group=cli_cb_group)
        self.call_timer = self.create_timer(1, self.send_request, callback_group=timer_cb_group)
        self.is_running = False
        self.get_result = False
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    async def send_request(self):
        self.is_running = True
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.get_logger().info(
                'do one time: for %d + %d' %
                (int(sys.argv[1]), int(sys.argv[2])))
        future = self.cli.call_async(self.req)
        result = await future
        self.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (int(sys.argv[1]), int(sys.argv[2]), result.sum))
        if result is not None:
            self.get_logger().info(
                'do one more time: for %d + %d' %
                (int(sys.argv[1])+1, int(sys.argv[2])+1))
            self.req.a = int(sys.argv[1]) + 1
            self.req.b = int(sys.argv[2])+1
            future = self.cli.call_async(self.req)
            result = await future
            self.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (int(sys.argv[1])+1, int(sys.argv[2])+1, result.sum))
        self.get_result = True

        
    

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)  

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()