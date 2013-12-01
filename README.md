robotiq_s_model_utils
=====================

Overview
---------------------------------------------

Libraries for interfacing with the Robotiq S Model hand.



Installing
---------------------------------------------

### From source ###

Make sure you have a working catkin workspace, as described at:
http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

Also make sure you have git installed:

    sudo apt-get install git-core

Change directory to the source folder of your catkin workspace.
If, for instance, your workspace is `~/catkin_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/catkin_ws/src

Download the metapackage from the github repository (<ros_distro> may be `groovy` or `hydro`):

    git clone -b <ros_distro> https://github.com/kth-ros-pkg/robotiq_s_model_utils.git

Make sure you also have the robotiq metapackage:

    git clone -b groovy-devel https://github.com/ros-industrial/robotiq.git

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make



robotiq_s_model_control_client
---------------------------------------------


This C++ library provides a simple interface to the SModelTcpNode.py server from the `robotiq_s_model_control` package for controlling an S-Model Robotiq hand.

### Example ###
You can run the example robotiq_control_client_example by doing the following:

1. Make sure you are connected to the hand via ethernet and that your network card is set to a **fixed IP address** different from the IP address of the Robotiq hand (e.g. `192.168.1.30`). Set the **netmask** to `255.255.255.0` and the **gateway** to `0.0.0.0`. 

2. Bring up a roscore:
    
        roscore

3. Run the SModelTcpNode.py:

        rosrun robotiq_s_model_control SModelTcpNode.py <ip_address>

    Where \<ip_address\> is the **IP address** of the Robotiq hand (tipically `192.168.1.11`). 

4. Run the example:
   
        rosrun robotiq_s_model_control_client robotiq_control_client_example



robotiq_s_model_joint_state_publisher
---------------------------------------------

The `robotiq_dummy_joint_state_publisher` node publishes dummy joint states for the S-Model Robotiq hand to the `/joint_state` topic.
