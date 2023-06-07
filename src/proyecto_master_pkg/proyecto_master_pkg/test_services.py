from proyecto_interfaces.srv import StartNavigationTest
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(StartNavigationTest, '/group_8/start_navigation_test_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StartNavigationTest.Request()

    def send_request(self, a, b):
        self.req.x = a
        self.req.y = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    posf=input("ingrese la posicion final (x,y): ")
    posf=posf.split(",")
    print(posf)
    
    response = minimal_client.send_request(float(posf[0]), float(posf[1]))
    print(response.answer)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
