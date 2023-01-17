#! /usr/bin/env python3
"""OT2 Node"""

import yaml
from typing import List, Tuple
from pathlib import Path
from datetime import datetime
from copy import deepcopy
from time import sleep

import rclpy
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from wei_services.srv import WeiActions, WeiDescription
from std_msgs.msg import String

from ot2_driver.ot2_driver_http import OT2_Config, OT2_Driver
import opentrons.simulate
from opentrons.simulate import format_runlog




import os
import time


class OT2Client(Node):

    """
    The init function is neccesary for the OT2Client class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    """

    def __init__(self, TEMP_NODE_NAME = "OT2_Node"):
        """Setup OT2 node"""

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.declare_parameter("ip","127.0.0.1")

        # Receiving the real IP and PORT from the launch parameters
        self.ip =  self.get_parameter("ip").get_parameter_value().string_value

        self.get_logger().info("Received IP: " + self.ip + " Robot name: " + str(self.node_name))
        self.state = "UNKNOWN"
        self.connect_robot()

        self.description = {
            "name": self.node_name,
            "type": "",
            "actions": {
                "execute": "config : %s",  ## takes in the yaml content as second string arg
                "run_protocol": "config_path: %s",  ## Temp inclusion
            },
        }

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        self.timer_period = 1  # seconds

        ## Creating state Msg as a instance variable
        # self.stateMsg = String()
        # self.state = "READY"  ## If we get here without error, the client is initialized
        # self.stateMsg.data = "State %s" % self.state

        # Publisher for ot2 state
        self.statePub = self.create_publisher(String, self.node_name + "/ot2_state", 10)

        # Timer callback publishes state to namespaced ot2_state
        self.stateTimer = self.create_timer(self.timer_period, self.stateCallback, callback_group = state_cb_group)

        # Control and discovery services
        self.actionSrv = self.create_service(
            WeiActions, self.node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group
        )
        self.descriptionSrv = self.create_service(
            WeiDescription, self.node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group
        )

    def connect_robot(self):
        try:
            self.ot2 = OT2_Driver(OT2_Config(ip = self.ip))

        except Exception as error_msg:
            self.state = "OT2 CONNECTION ERROR"
            self.get_logger().error("------- OT2 " + str(self.node_name) + " Error message: " + str(error_msg) +  " -------")

        else:
            self.get_logger().info("OT2 "+ str(self.node_name) + " online")



    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.
        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: Tuple[str, List]
            The actions a robot can do, will be populated during execution
        Returns
        -------
        Tuple[str, List]
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    async def actionCallback(self, request: str, response: str) -> None:

        """The actions the robot can perform, also performs them
        Parameters:
        -----------
        request: str
            Request to the robot to perform an action
        respone: bool
            If action is performed
        Returns
        -------
        None
        """
        if self.state == "OT2 CONNECTION ERROR":
            self.get_logger.error("Can not accept the job! OT2 CONNECTION ERROR")
            return

        self.manager_command = request.action_handle
        self.manager_vars = eval(request.vars)

        self.get_logger().info(f"In action callback, command: {self.manager_command}")

        # if "execute" == self.manager_command:

        #     protocol_config = self.manager_vars.get("config", None)
        #     if protocol_config:
        #         payload = self.manager_vars.get("payload", None)
        #         self.get_logger().info(f"{self.manager_vars=}")
        #         self.get_logger().info(f"ot2 {payload=}")
        #         config_file_path = self.download_config(protocol_config)
        #         response = self.execute(config_file_path, payload)
        #     else:
        #         self.get_logger().error("Required 'config' was not specified in request.vars")

        ## Actual API
        if "run_protocol" == self.manager_command:

            # protocol_config = self.manager_vars.get("config", None)
            protocol_config = self.manager_vars.get("config_path", None)
            if protocol_config:
                config_file_path = self.download_config(protocol_config)
                payload = deepcopy(self.manager_vars)
                payload.pop("config_path")
                self.get_logger().info(f"{self.manager_vars=}")
                self.get_logger().info(f"ot2 {payload=}")
                self.get_logger().info(f"config_file_path: {config_file_path}")
                res = self.execute(config_file_path, payload)
                response.action_response = 1
                response.action_msg = "all good ot2"
                self.get_logger().info("Finished Action: " + request.action_handle)
                return response

            else:
                response.action_msg = (
                    "Required 'config' was not specified in request.vars"
                )
                response.action_response = 0
                self.get_logger().error(response.action_msg)
                return response

    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""
 

        if self.state != "OT2 CONNECTION ERROR":
            msg = String()
            ID_run = "1" #TODO: Get the actual run ID 
            state = self.ot2.check_run_status(run_id=ID_run)
            if state == "IDLE":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                sleep(0.5)

            elif state == "RUNNING" or state == "FINISHING":
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                sleep(0.5)

            elif state == "SUCCEEDED":
                self.state = "COMPLETED"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                sleep(0.5)

            elif state == "FAILED":
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                sleep(0.5)

        else: 
            msg = String()
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! IP: " + self.ip)
            self.connect_robot()

    def download_config(self, protocol_config: str):
        """
        Saves protocol_config string to a local yaml file locaton

        Parameters:
        -----------
        protocol_config: str
            String contents of yaml protocol file

        Returns
        -----------
        config_file_path: str
            Absolute path to generated yaml file
        """

        # config_dir_path = '/root/config/temp'
        # config_file_path = config_dir_path + "/pc_document.yaml"

        config_dir_path = Path.home().resolve() / ".ot2_temp"
        config_dir_path.mkdir(exist_ok=True, parents=True)
        config_file_path = (
            config_dir_path
            / f"protocol-{datetime.now().strftime('%Y%m%d-%H%m%s')}.yaml"
        )

        self.get_logger().info(
            "Writing protocol config to {} ...".format(str(config_file_path))
        )

        with open(config_file_path, "w", encoding="utf-8") as pc_file:
            yaml.dump(protocol_config, pc_file, indent=4, sort_keys=False)

        return config_file_path

    def execute(self, protocol_path, payload=None):
        """
        Compiles the yaml at protocol_path into .py file;
        Transfers and Exececutes the .py file

        Parameters:
        -----------
        protocol_path: str
            absolute path to the yaml protocol

        Returns
        -----------
        response: bool
            If the ot2 execution was successful
        """

        ## TODO Should first check that the ot2/node is not in process
        ## TODO must check for /IDLE/AVAILABLE or COMPLETED state

        self.state = "BUSY"
        self.stateCallback()

        try:
            (
                self.protocol_file_path,
                self.resource_file_path,
            ) = self.ot2.compile_protocol(protocol_path, payload=payload)
            protocol_file_path = Path(self.protocol_file_path)
            self.get_logger().info(f"{protocol_file_path.resolve()=}")
            self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
            resp = self.ot2.execute(self.run_id)

            if resp["data"]["status"] == "succeeded":
                self.stateCallback()
                # self.poll_OT2_until_run_completion()
                return True

        # except FileNotFoundError:
        #     from pathlib import Path

        #     response_msg = "Could not find protocol config file at {}, {}".format(protocol_path, Path(protocol_path).exists())
        #     self.get_logger().error(response_msg)
        #     self.stateCallback()

        except Exception as e:
            self.state = "ERROR"
            self.stateCallback()

            if "no route to host" in str(e.args).lower():
                response_msg = "No route to host error. Ensure that this container \
                has network access to the robot and that the environment \
                variable, robot_ip, matches the ip of the connected robot \
                on the shared LAN."
                self.get_logger().error(response_msg)

            response_msg = f"Error: {e}"
            self.get_logger().error(response_msg)

            rclpy.shutdown()  ## TODO: Could alternatively indent into the if block.
            ## TODO: Changed to as is to forestall any unexpected exceptions

    def poll_OT2_until_run_completion(self):
        """Queries the OT2 run state until reported as 'succeeded'"""

        self.get_logger().info("Polling OT2 run until completion")
        while self.state != "IDLE":

            run_status = self.ot2.get_run(self.run_id)

            if (
                run_status["data"]["status"]
                and run_status["data"]["status"] == "succeeded"
            ):
                self.state = "COMPLETED"
                self.get_logger().info("Stopping Poll")

            elif (
                run_status["data"]["status"]
                and run_status["data"]["status"] == "running"
            ):
                self.state = "BUSY"

            self.stateCallback()
            time.sleep(self.timer_period)


def main(args=None):


    rclpy.init(args=args)  # initialize Ros2 communication
    try:
        ot2_client = OT2Client()
        executor = MultiThreadedExecutor()
        executor.add_node(ot2_client)

        try:
            ot2_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            ot2_client.get_logger().info('Keyboard interrupt, shutting down.\n')
            ot2_client.ot2.change_lights_status(False)

        finally:
            executor.shutdown()
            ot2_client.destroy_node()
    finally:
        rclpy.shutdown()

    # else:
    #     print("The robot_ip environment variable has not been specified.")
    #     ## NOTE: TODO TO Verify that the IP Address is correct?


if __name__ == "__main__":

    main()
