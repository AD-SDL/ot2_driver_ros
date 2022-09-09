# OT2 Driver + Module Client

This repository combines :

- the OT2 Driver (as a submodule): `ot2_driver`
- the ROS2-enabled OT2 Module Client: `ot2_module_client`
- and the docker context and configuration to build the image to be deployed on the OT2s' terminals: `docker` and `Dockerfile`

**NOTE: Any changes made to the submodule has to be pulled in afresh, and verifed and recommitted to git.**

## Docker Instructions

### Build Docker Image from Source




1. Clone this repository and automatically initialize and update the submodule:

   `git clone --recurse-submodules https://github.com/AD-SDL/ot2_driver_ros.git `
   
   


2. Build the docker container:

   `docker build [--no-cache] -t <IMAGE_NAME> .` in the root directory

   where <IMAGE_NAME> is the name given to the image and will be referred to at runtime

   eg `docker build -t ot2_module_client .`



### Run Docker Container

To run the docker container as intended, 

- Connect your robot terminal (NUC/Jetson Nano/other) to the same LAN and subnet as the OT2 robot, either wirelessly or by direct-link.

- Note the <IP_ADDRESS> of the OT2 robot on the shared network. This can be looked up on the Opentrons App, but might require direct-link to auto-discover the exact OT2 in question.

- Optional: Note the <NAMESPACE> with which to launch the ot2_module_client/ot2Node 



1. Run the docker container, expose all the host's port to the container, and specify the required robot_ip and optional robot_name environment variables:

   `docker run --net=host -it -e robot_ip=<IP_ADDRESS> [-e robot_name=<NAMESPACE>] <IMAGE_NAME>`


   	Specifying a `robot_name=<NAMESPACE>`, overrides the default "ot2_module" namespace.

   	eg `docker run --net=host -it -e robot_ip=192.168.50.197 -e robot_name=ot2_alpha ot2_module_client`



NOTE: **Running the docker container automatically launches the launch/ot2_module.launch.py launchfile**. Stop the launched processes and the container with Ctrl + C.



### Test Service Call [OPTIONAL]

The docker image is built with a sample python file and sample protocol configuration yaml file for testing the `ot2Node/actions` service 

With the docker container running,



1. Note the <CONTAINER_NAME> of the recently activated docker container.

   Type `docker ps` in a different terminal shell.



2. Run docker exec on the running container to open a new bash terminal inside the running container:

   `docker exec -it <CONTAINER_NAME> bash `



3. Call the sample python file, which reads the content of the sample yaml file, compiles it into experiment instructions and then transfers and executes it on the OT2:

​		`python3 /root/overlay_ws/src/ot2_module_client/ot2_module_client/test_service_call.py`



The OT2 should then execute the sample protocol.