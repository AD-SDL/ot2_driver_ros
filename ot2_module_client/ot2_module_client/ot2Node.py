#! /usr/bin/env python3
"""OT2 Node"""

import yaml
from typing import List, Tuple
from pathlib import Path
from datetime import datetime 


import rclpy  
from ot2_driver.ot2_driver_http import OT2_Config, OT2_Driver
import opentrons.simulate
from opentrons.simulate import format_runlog

from rclpy.node import Node 
from wei_services.srv import WeiActions, WeiDescription
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

import os
import time


class ot2Node(Node):

    """
    The init function is neccesary for the ot2Node class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    """


    def __init__(self, ROBOT_IP = "", NODE_NAME = "ot2Node"):
        """Setup OT2 node"""

        super().__init__(NODE_NAME)

        self.ot2 = OT2_Driver(OT2_Config(ip=ROBOT_IP))            
        self.get_logger().info("OT2 is online") # Wakeup Message
        
        self.description = {
            'name': NODE_NAME,
            'type':'',
            'actions':
            {
                'execute': "config : %s", ## takes in the yaml content as second string arg
                'run_protocol': "config_path: %s",  ## Temp inclusion
            }
        }

        self.timer_period = 1   # seconds
        
        ## Creating state Msg as a instance variable
        self.stateMsg = String()    
        self.state = "READY"        ## If we get here without error, the client is initialized
        self.stateMsg.data = "State %s" % self.state

        # Publisher for ot2 state
        self.statePub = self.create_publisher(String, "ot2_state", 10)   

        # Timer callback publishes state to namespaced ot2_state
        self.stateTimer = self.create_timer(self.timer_period, self.stateCallback)   

        # Control and discovery services
        self.actionSrv = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)
        self.descriptionSrv = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)


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
        
        self.manager_command = request.action_handle        
        self.manager_vars = eval(request.vars) 

        self.get_logger().info(f"In action callback, command: {self.manager_command}")

        if "execute" == self.manager_command:
            
            protocol_config = self.manager_vars.get("config", None) 
            if protocol_config:
                config_file_path = self.download_config(protocol_config)
                response.action_response = self.execute(config_file_path)
            else:
                self.get_logger().error("Required 'config' was not specified in request.vars")

        ## Temp inclusion
        elif "run_protocol" == self.manager_command:

            # protocol_config = self.manager_vars.get("config", None)
            protocol_config = self.manager_vars.get("config_path", None) 
     
            if protocol_config:

                config_file_path = self.download_config(protocol_config)

                response.action_response = self.execute(config_file_path)

            else:
                self.get_logger().error("Required 'config' was not specified in request.vars")

        return response



    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""
        self.stateMsg.data = "State %s" % self.state
        self.statePub.publish(self.stateMsg)
        self.get_logger().info('Publishing: "%s"' % self.stateMsg.data)


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
        
        config_dir_path = Path.home().resolve() / "ot2_temp"
        config_dir_path.mkdir(exist_ok=True, parents=True)
        config_file_path = config_dir_path / f"protocol-{datetime.now().strftime('%Y%m%d-%H%m%s')}.yaml"
        
        self.get_logger().info("Writing protocol config to {} ...".format(str(config_file_path)))
        
        with open(config_file_path,'w',encoding = "utf-8") as pc_file:
            yaml.dump(protocol_config, pc_file, indent=4, sort_keys=False)

        return config_file_path



    def execute(self, protocol_path):
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

        response = False
        self.state = "BUSY"
        self.stateCallback()


        try:
            self.protocol_file_path, self.resource_file_path = self.ot2.compile_protocol(protocol_path)     
            self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
            resp = self.ot2.execute(self.run_id)
    
            if resp["data"]["status"] == "succeeded":
                self.state = "COMPLETED"
                self.stateCallback()
                # self.poll_OT2_until_run_completion()
                response = True

            self.state  = "READY"

        except FileNotFoundError:
            self.state = "ERROR"
            response_msg = "Could not find protocol config file at {}".format(protocol_path)
            self.get_logger().error(response_msg)
            self.stateCallback()

            self.state  = "READY"
            
        except Exception as e:
            self.state = "ERROR"
            self.stateCallback()

            if "no route to host" in str(e.args).lower():
                response_msg = "No route to host error. Ensure that this container \
                has network access to the robot and that the environment \
                variable, robot_ip, matches the ip of the connected robot \
                on the shared LAN."
                self.get_logger().error(response_msg)

            response_msg = f'Error: {e}'
            self.get_logger().error(response_msg)
            
            rclpy.shutdown()  ## TODO: Could alternatively indent into the if block. 
                              ## TODO: Changed to as is to forestall any unexpected exceptions

        
        return response



    def poll_OT2_until_run_completion(self):
        """ Queries the OT2 run state until reported as 'succeeded' """

        self.get_logger().info("Polling OT2 run until completion")
        while self.state != "COMPLETED":

            run_status = self.ot2.get_run(self.run_id)

            if run_status["data"]["status"] and run_status["data"]["status"] == "succeeded":
                self.state = "COMPLETED"
                self.get_logger().info("Stopping Poll")

            elif run_status["data"]["status"] and run_status["data"]["status"] == "running":
                self.state = "BUSY"
            
            self.stateCallback()
            time.sleep(self.timer_period)


def main(args=None): 

    ip = os.getenv("robot_ip", None)

    if ip:
    
        node_name = "ot2Node"   # Node name for peeler   
        rclpy.init(args=args)  # initialize Ros2 communication
        node = ot2Node(ROBOT_IP=ip, NODE_NAME=node_name)
        try:
            rclpy.spin(node)    # keep Ros2 communication open for action node
        except KeyboardInterrupt:
            pass
    
    else:
        print("The robot_ip environment variable has not been specified.")
        ## NOTE: TODO TO Verify that the IP Address is correct?


if __name__ == "__main__":

    main()
