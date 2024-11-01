import sys
from threading import Thread
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientSync(Node):

    def __init__(self, cb_group):
        super().__init__('minimal_client_sync')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints', callback_group=cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        return self.cli.call(self.req)
        # This only works because rclpy.spin() is called in a separate thread below.
        # Another configuration, like spinning later in main() or calling this method from a timer callback, would result in a deadlock.

def main():
    rclpy.init()
    ##################################################
    

    ####### Use another thread for spin ##########
    client_cb_group = MutuallyExclusiveCallbackGroup()
    minimal_client = MinimalClientSync(client_cb_group)
    spin_thread = Thread(target=rclpy.spin, args=(minimal_client,))
    spin_thread.start()
    response = minimal_client.send_request()

    ######## Or you can use a reetrant callback group, but this will not work #####
    # client_cb_group = ReentrantCallbackGroup()
    # minimal_client = MinimalClientSync(client_cb_group)
    # response = minimal_client.send_request()
    # print("This Line will not be printed")
    # rclpy.spin(minimal_client)
    


    ##############################################
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()