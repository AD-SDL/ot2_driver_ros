import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rclpy.clock import clock

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from ot2_driver.ot2_driver_http import OT2_Config, OT2_Driver  #TODO: USE THIS WHEN IT IS READY

class OT2DescriptionClient(Node):

    def __init__(self, TEMP_NODE_NAME = 'OT2DescriptionNode'):


        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.declare_parameter("ip_list","127.0.0.1")
        self.declare_parameter("robot_list","OT2_Alpha")


        # Receiving the real IP and PORT from the launch parameters
        self.ip_list =  self.get_parameter("ip_list").get_parameter_value().string_value
        self.robot_list =  self.get_parameter("robot_list").get_parameter_value().string_values

        self.get_logger().info("Received IP list: " + self.ip_list + " Robot list: " + str(self.robot_list))

        timer_period = 0.1  # seconds

        self.state = "UNKNOWN"
        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, self.node_name + '/state',10)
        # self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)

        self.joint_publisher = self.create_publisher(JointState,'joint_states', 10, callback_group = joint_cb_group)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback, callback_group = joint_cb_group)
    
    
    def stateCallback(self):
        '''
        Publishes the ot2_description state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing State: "%s"' % msg.data)
        self.state = "READY"


    def joint_state_publisher_callback(self):
        
        # self.get_logger().info("BUGG")
        # joint_states = self.ot2.refresh_joint_state() #TODO: USE THIS WHEN IT IS READY
        joint_states = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        ot2_joint_msg = JointState()
        ot2_joint_msg.header = Header()
        ot2_joint_msg.header.stamp = self.get_clock().now().to_msg()
        ot2_joint_msg.name = ['OT2_1_Pipette_Joint1_alpha', 'OT2_1_Pipette_Joint2_alpha', 'OT2_1_Single_Pipette_alpha', 'OT2_1_8_Channel_Pipette_alpha', 'OT2_1_Pipette_Joint1_betha','OT2_1_Pipette_Joint2_betha', 'OT2_1_Single_Pipette_betha', 'OT2_1_8_Channel_Pipette_betha','OT2_1_Pipette_Joint1_gamma','OT2_1_Pipette_Joint2_gamma', 'OT2_1_Single_Pipette_gamma', 'OT2_1_8_Channel_Pipette_gamma']
        ot2_joint_msg.position = joint_states
        # print(joint_states)

        # ot2_joint_msg.position = [0.01, -1.34, 1.86, -3.03, 0.05, 0.05, 0.91]
        ot2_joint_msg.velocity = []
        ot2_joint_msg.effort = []

        self.joint_publisher.publish(ot2_joint_msg)
        self.get_logger().info('Publishing joint states: "%s"' % str(joint_states))


def main(args=None):
    rclpy.init(args=args)
    try:
        ot2_joint_state_publisher = OT2DescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(ot2_joint_state_publisher)

        try:
            ot2_joint_state_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            ot2_joint_state_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            ot2_joint_state_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()