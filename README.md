<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
-->

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab1">
    <img src="images/logo_orizzontale_COLORE.png" width="400" height="">
  </a>

  <h3 align="center">Assignment 2 - Pet Behavioural Architecture</h3>
</p> 


<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#software-architecture">Software Architecture</a>
      <ul>
        <li><a href="#components-architecture">Components Architecture</a></li>
        <li><a href="#action-servers">Action Servers</a></li>
        <li><a href="#state-machine">State Machine</a></li>
        <li><a href="#ros-topics">ROS Topics</a></li>
        <li><a href="#ros-messages">ROS Messages</a></li>
        <li><a href="#rqt">Rqt_graphs</a></li>
      </ul>
    </li>
    <li><a href="#repository-organization">Repository Organization</a></li>
    <li><a href="#prerequisites">Prerequisites</a></li>
    <ul>
        <li><a href="#ros">ROS</a></li>
        <li><a href="#python">Python</a></li>
     </ul>
    <li><a href="#installation">Installation</a></li>
    <li><a href="#working-hypo">Working hypothesis and environment</a></li>
    <ul>
        <li><a href="#system-features">System's features</a></li>
        <li><a href="#system-limitations">System's limitations</a></li>
        <li><a href="#future-work">Future work</a></li>
    </ul>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This system is built to simulate an environment where a human, a robot and a ball are spawned. The robot should follow three different behaviours: Normal, Sleep and Play. The robot is a wheeled dog having a camera located on top of its head. The human simulated represents the user that controls the movements of the ball within the environment. The human is not controlling the motion of the ball, since this last one is defined as a robot with one link capable of moving within the environment using specific topics. The human can move the ball around, by giving a command position to reach and make the ball disappear.
The objective of this project was to modify the provided robot model by using additional links and joints, build a suitable ROS architecture to implement the robot???s behaviours and simulate the system on Gazebo.
In this project, the pet robot can assume three behaviours:
* NORMAL, when it moves randomly within the environment
* SLEEP, when it moves to a predefined position and stops there for a given time;
* PLAY, when it sees the ball and start playing with it.


### Built With
The project was build with:

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Python3](https://www.python.org/downloads/)
* [SMACH](http://wiki.ros.org/smach)


## Software Architecture

### Simulation 
Our project is simulated in a 3D [**Gazebo**](http://gazebosim.org/) environment. Gazebo is a 3D dynamic simulator that efficiently simulates robots in complex indoor and outdoor environments. It provides physics simulation having a rich library of robot models and environments, a wide variety of sensors, and a user-friendly graphical interface.

### Robot model 
We developed a rough and simple 3D model to simulate a pet robot within Gazebo. The robot is made up of two classic wheels and a spherical one that allow it to move. We added a cylinder to simulate the neck and fixed it to the chassis, while we use a cube to simulate the robot head. The head is connected to the neck through a revolute joint, that can perform a rotation of 180 degrees.  A camera has been applied to the robot's head to make it capable of detecting the ball.
If we want to simulate a robot on Gazebo or any other simulation software, we need to add physical and collision properties, such as the dimension of the geometry to calculate the possible collisions, the weight that will give us the inertia, and so on. To do so we use two files: a URDF and a XACRO (???XML Macros) file.
URDF is an XML file format used in ROS to describe all elements of a robot. In this file, additional simulation-specific tags can be added to work properly with Gazebo as explained in this [link](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros). URDF files can only specify the kinematic and dynamic properties of a single robot in isolation and not the pose of the robot itself within a world. The XACRO file instead helps in reducing the overall size of URDF files and makes it easier to read and maintain the packages. We can use it to create modules that can be reused in the URDF, in case of repeated structures, such as the wheels of a robot. 

<p align="center">
<a>
    <img src="images/gazebo.png" width="400" height="">
</a>
</p>

To interact with the simulation environment, we use ROS plugins which can be used to add functionalities for simulation purposes, such as controlling the robot mode or adding sensors like cameras.
In our project, we use ROS_Control which consists of a set of packages for controller interface, controller manager, transmissions, hardware interfaces and control toolbox. It can be used to control the joint actuators of the robot.

### State Machine

<p align="center">
<a>
    <img src="images/State_Machine.png" width="400" height="">
</a>
</p>

The wheeled dog has three behaviours:

* NORMAL BEHAVIOR: when the robot assumes this behaviour, it starts moving randomly within the environment. The robot goes from normal behaviour to play behaviour when he sees the ball within the environment. Otherwise, when it is moving, the sleep timer is activated and the robot should assume SLEEP behaviour;
* SLEEP BEHAVIOR: the robot moves to a predefined position which indicates "home position" and stops there for a given time interval. After a certain time, it should "wake up" and assume NORMAL behaviour; </li>
* PLAY BEHAVIOR: 
    * It starts following the ball; </li>
    * When the ball stops, it moves the head to the left of 45 degrees, then to the right, it keeps it there for some seconds, then again it moves it to the centre.
    *  Once it moved the head, it keeps tracking the ball until it stops again.
    *  The robot goes back to normal behaviour when it cannot find the ball for a certain amount of time.

### Components Architecture

<p align="center">
<a>
    <img src="images/draft_architecture.png" width="600" height="">
</a>
</p>

**Components**  
* **Behavior Command Manager:** this component simulates the Finite State Machine (FSM) and controls the switching between the three robot behaviours (Normal, Sleep, Play) described in detail in the section _**State Machine**_. It is also responsible to move the robot head when in front of the ball
* **OpenCV Tracking:** it makes use of the OpenCV library to detect the ball. [_OpenCV_](https://opencv.org/) is a library used for real-time computer vision. ROS can be interfaced to OpenCV by using [CvBridge](http://wiki.ros.org/cv_bridge) and convert ROS images(see this [link](cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython - ROS Wiki))into OpenCV images, or vice versa. This library is used to determine if the ball is contained in the robot camera range or not. This component subscribes to the robot camera topic given by _/robot/camera1/image_raw/compressed_. 
  * Once the ball is detected, it communicates with the Behavior Manager using the parameter _ballVisible_ of the custom message _ballstate_. This way the behaviour controller can switch the behaviour from Normal to Play. 
  * When the robot switch to Play behaviour, this component also publishes the robot velocity on the topic _/cmd_vel_. This velocity is then applied to the model in Gazebo. Furthermore, the robot stops when the ball stops. 
  * When this happens, the tracking is stopped as well, and OpenCV communicates with _Behavior Manager_, which is responsible for making the robot move its head, first toward the right and then left and returns the head to the centre. 
  * Once it has moved the head, a message is sent over the topic _head_state_ which is subscribed by OpenCV tracking. Then the robot starts following or tracking the ball again, depending on the ball position.
* **Human Interface Simulator:** this component is used to simulate a human moving the ball within the gazebo environment. To do so, it sends a goal position to the ball, using a SimpleActionClient which stops once the goal is reached. Once the command is sent, this component stops sending commands for a random number of seconds so that the ball can remain still for a certain time to let the robot track it and reach it. This node can send a goal position to the ball or make it disappear by moving it underground.

### Action Servers
**ROS Actions** have a client-to-server communication relationship with a specified protocol. The actions use ROS topics to send goal messages from a client to the server. They are implemented in ROS using the **actionlib** package, whose documentation can be found at this [link](http://wiki.ros.org/actionlib). This allows a request/reply interaction between two nodes, the action client and the action server, that communicate via a ROS Action Protocol, which is built on top of ROS messages. The client and server then provide a simple API for users to request goals (on the client-side) or to execute goals (on the server-side) via function calls and callbacks.
<p align="center">
<a>
    <img src="images/expserver.png" width="400" height="">
</a>
</p>
Like ROS services, actions are defined in text files and they contain the following information:

* Goal: request sent by the action client to the action server, like "move the robot joint of 45 degrees";
* Cancel: used to send cancel request to the server
* Status: used to notify the client of the current state of every goal in the system
* Feedback: feedback information used by the action server to the action client while the request is being processed, such as the current value of the joint being moved.
* Result: final information sent by the action server to the action client once the request has been fulfilled

In particular, we have implemented the following action servers:

* **Go To Point Robot:** it receives a goal position from the motion component and it publishes the robot velocity to the /robot/cmd_vel topic to move the robot in Gazebo. When the robot has reached the given goal position, it sends back a feedback message to the Behavior Manager component. This action server can perform the following: 
  * adjust the yaw angle of the robot so that it can reach the given position correctly
  * move the robot in a straight direction
  * stop the robot when its goal is reached.
* **Go To Point Ball:**  this action server controls the ball movements instead of the ones of the robot. It controls the linear velocity of the ball along the three axes and in this case, there is no need to fix or compute the yaw. This action server can perform the following:
  *  movement towards a goal position as if it was given by the user 
  *  command to stop when the goal is reached.


### ROS Topics
As shown in the UML graph or the system architecture, the system makes use of the following topics:
* /cmd_vel: topic published by the OpenCV tracking component to set the robot velocity in Gazebo. This topic is also published by the Go-To Point Robot Action server 
* /robot/odom: topic used to get by the robot odometry from Gazebo simulator. It is subscribed by the Go to Point Robot Action server
* /robot/camera1/compressed_image: topic over which camera information is sent
* /head_state: used to tell OpenCV tracking if the robot has finished moving its head and can go back tracking the ball
* /ball_state: used to tell Behavior manager if the robot is in front of the ball and it can move its head or not
* /robot/reaching_goal: a set of topics used by the action server Go to Point Robot and defined by:
  * /result
  * /status
  * /feedback
  * /goal  
* /ball/cmd_vel: topic published by the OpenCV tracking component to set the robot velocity in Gazebo. This topic is also published by the Go-To Point Robot Action server 
* /ball/odom: topic used to get by the robot odometry from Gazebo simulator. It is subscribed by the Go to Point Robot Action server
* /ball/reaching_goal: set of topics used by the action server Go to Point Robot and defined by:
  * /result
  * /status
  * /feedback
  * /goal  

### Rqt_graphs 

<p align="center">
<a>
    <img src="images/rqt.PNG" width="800" height="">
</a>
</p>

## Repository Organization
The repository contains the following folders:

* **Documentation**: it contains the HTML and latex documentation produced with Doxygen
* **action**: it contains the definition of the action message used by the two action servers
* **config**: contains configuration file related to the Ros control plugin
* **images**: contains .png images used in the README.md file
* **launch**: contains launch file:
  *  pet_behavior.launch: it starts the behaviour architecture of the project
* **msg**: contains the ball_state.msg and head_state.msg custom messages
* **scripts**: it contains the python ROS nodes which implement two action servers to move the ball and the robot:
  *  go_to_point_ball.py
  *  go_to_point_robot.py
* **src**: contains the relative script of the main components of the architecture:
  *  behavior_manager.py
  *  human_simulator.py
  *  opencv_tracking.py
* * **urdf**: it contains the description of a robot model, a human and a ball;
* **worlds**: it contains the world used for implementing the simulation;
* 
## Prerequisites

### Ros
This project is developed using [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Follow [instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) for installation.

### Python
This project is developed using [Python](https://www.python.org/downloads/). Follow [instructions](https://www.python.org/downloads/) for installation.

### Smach
This project uses the smach library to use the FSM.
 ```sh
 $ sudo apt-get install ros-kinetic-smach-viewer
 ```
### Ros Control plugin
It is necessary to install ros control plugin using the commands:
 ```sh
 $ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
 $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
 ```
 ### OpenCV library
 Install **OpenCV** library using:
  ```sh
  $ sudo apt-get install python-opencv
 ```
### numpy library
Install **numpy** library using the following:
  ```sh
  $ pip install numpy
  ```
  
## Installation
This instructions assumes that you have installed **catkin**, if not it is necessary to follow the instruction at [catkin installation](https://wiki.ros.org/catkin#Installing_catkin). After installation source the environment:
 ```sh
 $ source /opt/ros/kinetic/setup.bash
 ```
 1. Create a workspace in ROS following this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 2. Enter the folder src and  clone the repository
   ```sh
   git clone https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab_2.git
   ```
  2. Rename the cloned package as _assignment2_
  3. Enter the repository at the top level and run
   ```sh
    $ catkin_make
    $ source devel/setup.bash
   ```
   4. Enter the following to start the program:
   ```sh
    $ roslaunch assignment2 pet_beaviour.launch
   ``` 

### Systems features and limitations

To show the system functioning:

* **Case 1**: in this [video](https://web.microsoftstream.com/video/6ac40e18-aa04-4dde-8356-dac2f946a499) the robot is moving normally and it detects the ball, starting following it
* **Case 2**: in this [video](https://web.microsoftstream.com/video/fb8dfb36-9bcf-46db-a742-4b921472b172) the ball is not visible, the robot moves randomly in a normal state. Once it detects the ball and enters play state

The velocity of the ball was set after performing different tests and seeing how the system was behaving in those situations. Initially, the velocity of the ball was too high and the robot was losing sight of it while following it. Therefore we lowered the maximum velocity of the ball in the Action Server and increased a bit the velocity of the robot during the ball tracking. The robot speed still needs some improvements, since it may happen that if the ball moves at full speed towards the robot, it may overturn when trying to avoid it.
Another limitation is the fixed neck of the robot when moving in a normal state. The camera range is reduced and make it more difficult to detect the ball quickly
Some improvements can be:
* use a more realistic model
* add a system for obstacle avoidance
* implement a feature to move the robot's neck and expand the camera range when moving in normal mode

<!-- CONTACT -->
## Contact

Serena Roncagliolo - s4233330@studenti.unige.it

Project Link: [https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab2](https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab2)



