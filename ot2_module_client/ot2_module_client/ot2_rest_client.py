#! /usr/bin/env python3
"""OT2 Node"""
import os
import glob
import json
import traceback
import yaml
from typing import List, Tuple
from pathlib import Path
from datetime import datetime
from copy import deepcopy
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from wei_services.srv import WeiActions, WeiDescription
from std_msgs.msg import String

from ot2_driver.ot2_driver_http import OT2_Config, OT2_Driver
import opentrons.simulate
from opentrons.simulate import format_runlog
from urllib.error import HTTPError, URLError
from urllib3.exceptions import ConnectionError, ConnectTimeoutError
from urllib3.connection import HTTPException, HTTPConnection
import requests

#! /usr/bin/env python3




from time import sleep
import threading
import asyncio

from platecrane_driver.sciclops_driver import SCICLOPS # import sciclops driver

from time import sleep
import json

from threading import Thread


"""The server that takes incoming WEI flow requests from the experiment application"""
import json
from argparse import ArgumentParser
from contextlib import asynccontextmanager
import time
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse

workcell = None
global sealer, state
serial_port = '/dev/ttyUSB0'
local_ip = 'parker.alcf.anl.gov'
local_port = '8000'

global ot2
resources_folder_path = ""
protocols_folder_path = ""
node_name = ""

def check_resources_folder(self):
        """
        Description: Checks if the resources folder path exists. Creates the resource folder path if it doesn't alIDLE exists
        """
        global resources_folder_path
        isPathExist = os.path.exists(resources_folder_path)
        if not isPathExist:
            os.makedirs(resources_folder_path)
            get_logger().warn("Resource path doesn't exists")
            print("Creating: " + resources_folder_path)
            
def check_protocols_folder(self):
        """
        Description: Checks if the protocols folder path exists. Creates the resource folder path if it doesn't alIDLE exists
        """
        global protocols_folder_path
        isPathExist = os.path.exists(protocols_folder_path)
        if not isPathExist:
            os.makedirs(protocols_folder_path)
           # get_logger().warn("Protocols path doesn't exists")
            print("Creating: " + protocols_folder_path)

def connect_robot(self):
        global ot2, state
        ip = "127.0.0.1"
        try:
            ot2 = OT2_Driver(OT2_Config(ip = ip))

        except ConnectTimeoutError as connection_err:
            state = "ERROR"
            print("Connection error code: " + connection_err)

        except HTTPError as http_error:
            print("HTTP error code: " +  http_error)
            
        except URLError as url_err:
            print("Url error code: " +  url_err)

        except requests.exceptions.ConnectionError as conn_err: 
            print("Connection error code: "+ str(conn_err))
            
        except Exception as error_msg:
            state = "ERROR"
            print("-------" + str(error_msg) +  " -------")

        else:
            print(str(node_name) + " online")
def download_config_files(self, protocol_config: str, resource_config = None):
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

    config_dir_path = Path.home().resolve() / protocols_folder_path
    config_dir_path.mkdir(exist_ok=True, parents=True)
    
    resource_dir_path = Path.home().resolve() / resources_folder_path
    resource_dir_path.mkdir(exist_ok=True, parents=True)
    
    time_str = datetime.now().strftime('%Y%m%d-%H%m%s')
    config_file_path = (
        config_dir_path
        / f"protocol-{time_str}.yaml"
    )

    print(
        "Writing protocol config to {} ...".format(str(config_file_path))
    )

    with open(config_file_path, "w", encoding="utf-8") as pc_file:
        yaml.dump(protocol_config, pc_file, indent=4, sort_keys=False)
    if resource_config:
        resource_file_path = resource_dir_path / f"resource-{node_name}-{time_str}.json"
        with open(resource_config) as resource_content:
            content = json.load(resource_content)
        json.dump(content, resource_file_path.open("w"))
        return config_file_path, resource_file_path
    else:
        return config_file_path, None
    
def execute(self, protocol_path, payload=None, resource_config = None):
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

    global run_id
    try:
        (
            protocol_file_path,
            resource_file_path,
        ) = ot2.compile_protocol(protocol_path, payload=payload, resource_file = resource_config, resource_path = resources_folder_path, protocol_out_path = protocols_folder_path) 
        protocol_file_path = Path(protocol_file_path)
        print(f"{protocol_file_path.resolve()=}")
        protocol_id, run_id = ot2.transfer(protocol_file_path)
        print("OT2 " + node_name + " protocol transfer successful")
        resp = ot2.execute(run_id)
        print("OT2 "+ node_name +" executed a protocol")
        # get_logger().warn(str(resp))

        if resp["data"]["status"] == "succeeded":
            # poll_OT2_until_run_completion()
            response_msg = "OT2 "+ node_name +" successfully IDLE running a protocol"
            return True, response_msg

        else: 
            response_msg = "OT2 "+ node_name +" failed running a protocol"
            return False, response_msg

    # except FileNotFoundError:
    #     from pathlib import Path

    #     response_msg = "Could not find protocol config file at {}, {}".format(protocol_path, Path(protocol_path).exists())
    #     print(response_msg)
    #     stateCallback()

    except Exception as err:

        if "no route to host" in str(err.args).lower():
            response_msg = "No route to host error. Ensure that this container \
            has network access to the robot and that the environment \
            variable, robot_ip, matches the ip of the connected robot \
            on the shared LAN."
            print(response_msg)

        response_msg = f"Error: {traceback.format_exc()}"
        print(response_msg)
        return False, response_msg

        # rclpy.shutdown()  ## TODO: Could alternatively indent into the if block.
        ## TODO: Changed to as is to forestall any unexpected exceptions

def poll_OT2_until_run_completion(self):
    """Queries the OT2 run state until reported as 'succeeded'"""
    global run_id
    print("Polling OT2 run until completion")
    while state != "IDLE":

        run_status = ot2.get_run(run_id)

        if (
            run_status["data"]["status"]
            and run_status["data"]["status"] == "succeeded"
        ):
            state = "IDLE"
            print("Stopping Poll")

        elif (
            run_status["data"]["status"]
            and run_status["data"]["status"] == "running"
        ):
            state = "BUSY"

@asynccontextmanager
async def lifespan(app: FastAPI):
        global ot2, state
        """Initial run function for the app, parses the worcell argument
            Parameters
            ----------
            app : FastApi
            The REST API app being initialized

            Returns
            -------
            None"""

        state = "UNKNOWN"
        resources_folder_path = '/home/rpl/.ot2_temp/' + node_name + "/" + "resources/"  
        protocols_folder_path = '/home/rpl/.ot2_temp/' + node_name + "/" + "protocols/"  
        
        check_resources_folder()
        check_protocols_folder()
        connect_robot()

        description = {
            "name": node_name,
            "type": "",
            "actions": {
                "execute": "config : %s",  ## takes in the yaml content as second string arg
                "run_protocol": "config_path: %s",  ## Temp inclusion
            },
        }


app = FastAPI(lifespan=lifespan, )

@app.get("/state")
async def get_state():
    global sealer
    return JSONResponse(content={"State":state})

@app.get("/description")
async def description():
    global state
    return JSONResponse(content={"State": state})

@app.get("/resources")
async def resources():
    global sealer
    return JSONResponse(content={"State": sealer.get_status() })


@app.post("/action")
async def do_action(
    action_handle: str,
    action_vars: dict, 
):  
        global ot2, state
        response = {}
        if state == "ERROR":
                msg = "Can not accept the job! OT2 CONNECTION ERROR"
               # get_logger.error(msg)
                response["action_response"] = -1
                response["action_msg"] = msg
                return response

        while state != "IDLE":
         #   get_logger().warn("Waiting for OT2 to switch IDLE state...")
            time.sleep(0.5)
        
        state="BUSY"
        action_command = action_handle
        action_vars = json.loads(action_vars)
        print(f"{action_vars=}")

        print(f"In action callback, command: {action_command}")

        if "run_protocol" == action_command:

            protocol_config = action_vars.get("config_path", None)
            resource_config = action_vars.get("resource_path", None) #TODO: This will be enbaled in the future 
            resource_file_flag = action_vars.get("use_existing_resources", "False") #Returns True to use a resource file or False to not use a resource file. 
            
            if resource_file_flag:
                try:
                    list_of_files = glob.glob(resources_folder_path + '*.json') #Get list of files
                    if len(list_of_files) > 0: 
                        resource_config = max(list_of_files, key=os.path.getctime) #Finding the latest added file
                        print("Using the resource file: " + resource_config)

                except Exception as er:
                    print(er)
            if protocol_config:
                config_file_path, resource_config_path = download_config_files(protocol_config, resource_config)
                payload = deepcopy(action_vars)
                payload.pop("config_path")

                print(f"ot2 {payload=}")
                print(f"config_file_path: {config_file_path}")

                response_flag, response_msg = execute(config_file_path, payload, resource_config_path)
                
                if response_flag == True:
                    state = "IDLE"
                    response["action_response"] = 0
                    response["action_msg"] = response_msg
                    if resource_config_path:
                        response.resources = str(resource_config_path)

                elif response_flag == False:
                    state = "ERROR"
                    response["action_response"] = -1
                    response["action_msg"] = response_msg
                    if resource_config_path:
                        response.resources = str(resource_config_path)

                print("Finished Action: " + action_handle)
                return response

            else:
                response["action_msg"] = (
                    "Required 'config' was not specified in action_vars"
                )
                response["action_response"] = -1
                print(response["action_msg"])
                state = "ERROR"

                return response
        else:
            msg = "UNKOWN ACTION REQUEST! Available actions: run_protocol"
            response["action_response"] = -1
            response["action_msg"]= msg
            print('Error: ' + msg)
            state = "IDLE"

            return response
   


if __name__ == "__main__":
    import uvicorn
    print("asdfsaf")
    uvicorn.run("a4s_sealer_REST:app", host=local_ip, port=local_port, reload=True, ws_max_size=100000000000000000000000000000000000000)
