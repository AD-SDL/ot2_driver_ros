FROM kwelbeck/base-ros2-with-empty-overlay:latest

## 1 #######################################
## Install system packages
## Reserved for what would be installed with apt-get
## Commented example below updates sources lists, and installs <package1> and <package2> via apt-get
##
## Uncomment and amend the list for installation

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     <package1> \
#     <package2> \
#     && rm -rf /var/lib/apt/lists/*




## 2 ########################################
## Make any necessary directories required during runtime
## Commented example below creates a directory at 
## /absolute/path/to/new/directory/1/ and at 
## /absolute/path/to/new/directory/2/ including all parent directories 
## along the path
##
## Uncomment and replicate as needed

# RUN mkdir -p /absolute/path/to/new/directory/1/ \
#              /absolute/path/to/new/directory/2/

RUN mkdir -p /root/config/temp/




## 3 ########################################
## Copy/Add drivers and other packages from repository root directory 
## into /root directory of image 
## Commented example below downloads ot2_driver from repository root then
## installs it, including python dependencies using pip3 on the 
## requirements.txt dependency list. 
## Also upgrades numpy to fix nested dependency
##
## Uncomment and replicate as needed
## Alternatively install python packages with `python3 setup.py install`

WORKDIR /root
COPY ot2_driver/ ot2_driver/
RUN pip3 install -r ot2_driver/requirements.txt \
    && pip3 install -e ot2_driver \
    && pip install numpy --upgrade 




## 4 ######################################
## Copy any resource or configuration files into image
## Commented example copies contents of the resources/ directory to /root/
##
## Uncomment and ammend as needed

WORKDIR /root
COPY docker/resources/ .




## 5 ######################################
## Copy/Add ros packages to the images' /root/overlay_ws/src directory
## including from other git sources via vcs
## Vcs take a yaml file of repository sources with version specifications, if 
## necessary,and then downloads for colcon build alongside the local packages
## Commented example downloads the ot2_module_client local package and the wei_services 
## package via vcs then builds, installs and sources updated workspace 
##
## Uncomment and ammend to explicitly download ros packages
## Ammend repos file, specifying commit version if necessary

WORKDIR $ROS_WS/src
COPY ot2_module_client ot2_module_client
COPY docker/repos repos
RUN vcs import < repos
WORKDIR $ROS_WS
SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install && source $ROS_WS/install/setup.bash





## 6 #######################################
## Final execution commands, including copying in scripts to be run before 
## the CMD instruction.
## Commented example, copies in the ros_entrypoint.sh script that sources the
## workspace, changes the working directory to where generated protocols will 
## be stored, executes the script as an entrypoint and then runs the 
## ot2_module_client
## The CMD instruction will be overridden by any command appended to the docker run command from terminal.
## By contrast, the ENTRYPOINT instruction will not be overridden.
##
##

COPY docker/ros_entrypoint.sh /
WORKDIR /root/config/temp/_compiled_protocols
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["ros2", "launch", "ot2_module_client", "ot2_module.launch.py"]
