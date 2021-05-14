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

  <h3 align="center">Assignment 1 - Behavioural Architecture</h3>
</p> 


<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
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


## Software  Architecture

### Components Architecture

**Components**  

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



