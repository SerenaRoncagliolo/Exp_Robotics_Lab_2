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

  <h3 align="center">Assignment 2 - Behavioural Architecture</h3>
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

This system is built to simulate and environment where a human, a robot and a ball are spawned. The robot should follow three different behaviours: Normal, Sleep and Play. The robot is a wheeled dog having a camera located on top of its head. The human simulated represents the user that controls the movements of the ball within the environment. The human is not actually controlling the motion of the ball, since this last one is defined as a robot with one link capable of moving within the environment using speficic topics. The human can move the ball around, by giving a command position to reach, and make the ball disappear.
The objective of this project was to modify the provided robot model by using additional links and joints, build a suitable ROS architecture to implement the robot’s behaviors and simulate the system behavior on Gazebo.

### Built With

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Python3](https://www.python.org/downloads/)
* [SMACH](http://wiki.ros.org/smach)


## Software Architecture

### Components Architecture

<p align="center">
<a>
    <img src="images/draft_architecture.png" width="600" height="">
</a>
</p>

**Components**  
* **Behavior Command Manager:** this component simulate the Finite State Machine (FSM) and control the switching between the robot behaviors: Normal, Sleep and Play; this component subscribe to the /ball_detected topic to switch between Normal to Play and to the /home_reached topic to switch between Sleep and Normal; it publishes the new behavior as a ROS message on the topic /behavior. The different behavior are explained later in this report.
* **Motion:** this component moves the robot when it assumes behavior Normal or Sleep; it subscribes to the /behaviour topic in order operate according to the behavior given; it also instantiate a SimpleActionClient which communicates with the _Go To Point Robot_ Action Server in order to send the correct goal position the robot has to reach; when the robot is found in Normal state, it makes it move randomly, by choosing a random goal position within the environment and waits for the Server to report if the robot has reached the goal; in the Sleep state, instead, the motion controller moves the robot to _home position_ and when it's reached, it report it to the Behavior Controller.
* **OpenCv Ball Tracking:** it make use of the OpenCv library to detect the ball within the environment, so that the robot can follow it in Play behavior; it subscribes to the robot camera topic; when the ball is detected, it publishes on the /ball_detected topic, this way the behavior controller can switch the behavior from Normal to ; when the robot switch to Play behavior, the component pushlishes on the topic /robot/cmd_vel the velocity to apply to the model in Gazebo; the robot should stop when the ball stop, thus the tracking is stopped as well, the robot rotates its head and then, when it returns to its default position, the tracking starts again.

### State Machine

<p align="center">
<a>
    <img src="images/State_Machine.png" width="400" height="">
</a>
</p>
The wheeled dog has three behaviors:
 <ol>
  <li> NORMAL BEHAVIOR: when the robot assumes this behavior, it starts moving randomly within the environment. The robot goes from the normal behavior to the play behavior when he sees the ball within the environment. Otherwise, when it is moving, the sleep timer is activated and the robot should assume SLEEP behavior;</li>
  <li> SLEEP BEHAVIOR: the robot moves to a predefined position which indicates "home position" and stops there for a given time interval. After a certain time, it should "wake up" and assume NORMAL behavior; </li>
  <li> PLAY BEHAVIOR: 
      <ol>
        <li> It starts following the ball; </li>
        <li> when the ball stops, it moves the head to the left of 45 degrees, it keeps the head in that position for a number of seconds, then it moves the head on the right, it keeps it there for a number of seconds, then again it moves it to the center.</li>
        <li> Move to new target; </li>
       <li> Once it moved the head, it keeps tracking the ball until it stops again. </li>
     </ol> 
    The robot goes back in the normal behavior when it cannot find the ball for a certain amount of time.
    </li>
 </ol> 

### ROS Topics

### ROS Messages

### Rqt_graphs 

## Repository Organization
The repository contains the following folders:
* **world**: it contains the world used for implementing the simulation;
* **urdf**: it contains the description of a robot model, a human and a ball;
* **scripts**: it contains the python ROS node which implement an action server to move the ball around;
* **action**: it contains the definition of a custom action message;
* **launch**: it contains the launch file for executing the simulation;

## Prerequisites

### Ros
This project is developed using [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Follow [instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) for installation.

### Python
This project is developed using [Python3](https://www.python.org/downloads/). Follow [instructions](https://www.python.org/downloads/) for installation.

### Smach
This project make use of the smach library to use the FSM.
 ```sh
 $ sudo apt-get install ros-kinetic-smach-viewer
 ```

## Installation

## Working hypothesis and environment

### Systems features

### Systems limitations

### Future work

<!-- CONTACT -->
## Contact

Serena Roncagliolo - s4233330@studenti.unige.it

Project Link: [https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab2](https://github.com/SerenaRoncagliolo/Exp_Robotics_Lab2)



