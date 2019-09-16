# A Simple Golang AWS Restful API

This package implements two simple HTTP request (POST, GET) using Golang on Amazon Web Services (AWS) in order to create an item on DynamoDB service through POST request as well as fetch information about and specific item of DynamoDB service through GET request. The package applies AWS Lambda function in order to interact with AWS interface.

## Prerequisites

You need to have prerequisites below to run the scenario:

- OS: Ubuntu 16.04 LTS
- <a href="http://wiki.ros.org/kinetic/Installation/Ubuntu">ROS Kinetic</a>
- Python 2.7

## Required ROS Packages

### Install Rosbridge Server

To install this package, run:

```bash:
$ sudo apt-get install ros-kinetic-rosbridge-server
```

### Install SIGVerse Rosbridge Server

Please see below.  
https://github.com/SIGVerse/ros_package/tree/master/sigverse_ros_bridge

## Make

Make the package using catkin:

```bash
cd ~/catkin_ws
catkin_make
```

## Running the Scenario

Launch `scenario.launch` which is in launch folder. This file calls `human_scenario.py` script which is located in scripts folder and it's the main file of Human Navigation scenario:

```bash
roslaunch human_nav_scenario scenario.launch
```

## Testing the Scenario

Given that Oculus Rift may not be available at test time, there are 2 python scripts to complete testing of Human Navigation scenario. First of all, Run the scenario. Then, start simulation in Windows side (Unity). To send 'Guidance_request' to robot, run `guidance_msg_pub.py` in a new terminal:

```bash
rosrun human_nav_scenario guidance_msg_pub.py
```

After running this script, the robot will receive 'Guidance_request' message and will give instructions to the test subject (human).

In order to test ability of the scenario to handle switching between sessions, you can use `giveup_publisher.py`. To run this script:

```bash
rosrun human_nav_scenario giveup_publisher.py
```

## Author

Mohammad Mojrian - Navigation & Scenario
