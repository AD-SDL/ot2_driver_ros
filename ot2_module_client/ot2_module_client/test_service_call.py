import rclpy
from rclpy.node import Node
from wei_services.srv import WeiActions
import yaml


class ServiceTestNode(Node):

    def __init__(self):
        super().__init__('serviceTestNode')

    def run(self):
        
        test_client = self.create_client(WeiActions, 'ot2Node/actions')

        while not test_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ot2Node/actions service not available, waiting again...')
        
        weiReq = WeiActions.Request()

        weiReq.action_handle = "execute"

        config_str = str(yaml.safe_load(open("/root/protocol_config.yaml")))
        
        weiReq.vars = str({'config' : config_str})

        self.get_logger().info("Calling service")

        future = test_client.call_async(weiReq)

        rclpy.spin_until_future_complete(self, future) 


if __name__ == "__main__":

    rclpy.init()
    srvTestNode = ServiceTestNode()
    srvTestNode.run()
    