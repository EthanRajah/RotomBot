#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        # Create clients for each service
        self.launch_client = self.create_client(Trigger, '/rob498_drone_4/comm/launch')
        self.test_client   = self.create_client(Trigger, '/rob498_drone_4/comm/test')
        self.land_client   = self.create_client(Trigger, '/rob498_drone_4/comm/land')
        self.abort_client  = self.create_client(Trigger, '/rob498_drone_4/comm/abort')

    def call_service(self, client, service_name: str):
        # Wait for the service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting...')
        self.get_logger().info(f'Calling {service_name} service...')
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'{service_name} response: success={response.success}, message="{response.message}"'
            )
        else:
            self.get_logger().error(f'Exception calling {service_name} service: {future.exception()}')

def main(args=None):
    rclpy.init(args=args)
    tester = ServiceTester()

    # Call each service individually for testing
    time.sleep(5)
    tester.call_service(tester.launch_client, 'Launch')
    time.sleep(10)
    tester.call_service(tester.test_client,   'Test')
    time.sleep(90)
    tester.call_service(tester.land_client,   'Land')
    # tester.call_service(tester.abort_client,  'Abort')

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
