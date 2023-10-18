# Mobile Manipulation Project

## Assignment 5: Mobile Manipulation Project

This assignment is carried out in pairs, and presented orally to the TA's. You must upload your files here before the oral presentation.  
Completing this assignment to an E level by Oct. 13, 17:00 will give you 3 bonus points towards the score of the final exam. Completing it to the C level by the same deadline will give you 5 bonus points, and the A level will give you 7 points.

## Presentation slides
## Introduction

The final project consists of a simulation in [Gazebo](http://gazebosim.org/) of a [TIAGo robot](http://tiago.pal-robotics.com/) in an apartment. The robot is equipped with several onboard sensors and a manipulator arm which enable it to autonomously navigate and interact with the environment through simple manipulation tasks. The goal of this project is to implement a mission planner for TIAGo to execute three different missions. **Note that you need to upload your solution here in Canvas before presenting, details below.**

## Format

In a real robotics team, you usually collaborate with other developers who are often responsible uniquely for a single module of a large robot system. Usually things work well until it's time to put them together and prepare the robot for a specific setup... This is the scenario of this project.

You are provided with a real set of working modules (planning, sensing, navigation, manipulation...) and you have to design and integrate a mission planner node with the necessary logic for the robot to carry out a given mission making use of these modules. This node will be responsible for the high-level task planning of the system, commanding where to go and what to do based on the robot current state and the mission specifications.

For this, it will have to utilize the available sensors and actuators through the software packages provided. Unfortunately, just like in many real system integration projects, most of these packages come with very little documentation and your colleagues are not around anymore, so you will have to do some digging.

**"It reminded me of my first day working at Boeing, and it wasn't a good day".**  
<p align="right">- Former student</p>

You are expected to implement and show a working solution consisting of two parts:

1. The mission planner node, within the corresponding files:
    1. "sm_students.py" in which you have to implement a high-level state machine (SM) triggering the right sequence of steps for the robot to achieve the final goal. In order to come up with this sequence, look at the videos provided and go through all the topics, services and actions offered by the modules provided. If you need the robot to do a backflip, most likely there's a service called /backflip_jump or similar. An example of state machine is provided within the working package.
    2. "bt_students.py" contains a [behavior tree](http://wiki.ros.org/py_trees_ros) (BT) equivalent to the state machine above (for grade C or A).
2. A [launch file](http://wiki.ros.org/roslaunch/XML) "launch_project.launch" where you have to deploy and connect the nodes/components of the system to carry out the tasks. For each different task you will have to uncomment the  nodes and provide them with the parameters required (substitute place_holderX with meaningful names).

Use the same node in the launch file to launch either the SM or BT just by changing the name of the python script being called, depending on the task you are solving

`<node pkg="robotics_project" type="sm_students.py" name="logic_state_machine" output="screen">`

The files mentioned are under the **robotics_project** package, available for you in the repository you are about to clone. You do not have to modify any other file than these three. 

There are three different missions with increasing difficulty levels, which will give you a corresponding grade. The reasonable way to go is to start with E and move on from there, since the higher levels build conceptually on top of the previous ones (with some changes in the implementation), but it's ok if you don't want to be reasonable.

Along with the demos, you will be questioned about basic concepts of the solutions you have implemented (see examples below). **The tasks are considered solved if your system works and you are able to answer these questions during the presentation.**

## Task E: Pick&Carry&Place without sensory input

In this scenario, TIAGo is expected to pick an Aruco cube from a known pose on top of table 1, navigate towards table 2 behind it and place the object there. The cameras and laser scan cannot be used for this level.



Implement a state machine which goes through the following main states:

1. Complete picking task
2. Carry cube to second table
3. Complete placing task

Obs: use the ROS tools (rostopic, rosmsg, rosservice, rosrun tf view_frames, etc) to explore the system and figure out which module does what. You will realize this is the most time-consuming part of this level. Once you are familiar with the project, implementing the solution will be straight forward.

Evaluation:

1. Show a working simulation and be able to explain your implementation
2. Be able to reason about this solution, i.e: Can the mission succeed if the cube is displaced before being picked? What if table 2 is moved?

## Task C: Pick&Carry&Place with visual sensing

The task is the same as above. However this time the camera sensor in TIAGo's head has to be used to detect the cube. After, compute a grasp, transport the marker and verify that it has been placed on the second table. You will implement this logic in the form of a behavior tree this time.



Implement a behavior tree which goes through the following main states:
1. Detect cube
2. Complete picking task 
3. Carry cube to second table
4. Complete placing task
5. Cube placed on table?
    1. Yes: end of task
    2. No: go back to initial state in front of table 1

Evaluation:
1. Show a working simulation and be able to explain your implementation
2. Be able to reason about this solution, i.e: Can the mission succeed if the cube is displaced before being picked? What if table 2 is moved? Would the robot be able to transport several cubes (one at a time) with the given solution?

## Task A: Pick&Carry&Place with sensing and navigation
Pick&Carry&Place with visual sensing and [navigation](http://wiki.ros.org/navigation?distro=kinetic):  in this third level, the robot [starts in an unknown pose](https://en.wikipedia.org/wiki/Kidnapped_robot_problem) and must make use of its sensors and a prior map of the room to transport the cube safely among rooms.


Implement a behavior tree that goes through the following main states:

1. Robot has localized itself in the apartment
2. Navigation to picking pose
3. Cube detected
4. Complete picking task 
5. Navigation with cube to second table
6. Complete placing task
7. Cube placed on table?
    1. Yes: end of mission
    2. No: go back to state 2 and repeat until success. For this, you need to respawn the cube to its original pose in case it has fallen.

Obs 1: At any time during the navigation (not after the picking/placing sequences have started), a bad-intentioned TA might kidnap your robot. Your behavior tree must be able to detect this and react to it so that the robot always knows its true position and can avoid collisions. Kidnap the robot yourself during your implementation to test your solution (the simulation can be paused and the robot moved manually, see how [here](http://gazebosim.org/tutorials?tut=guided_b2&cat=)).

Obs 2: The TA might also manually throw away the cube from the robot's gripper during the navigation to see that the robot is able to detect that the placing has failed. As before, you can test this yourselves by dropping the cube through the Gazebo GUI.

Obs 3: The robot uses a particle filter for localization. Use the current state of the distribution of the particles to know when the filter has converged. Other solutions will not be accepted.

Evaluation:

1. Show a working simulation and be able to explain your implementation
2. Be able to reason about this solution, i.e: Why do we ask you to make the robot spin to help the AMCL? What does the distribution of particles tell us? Why does AMCL fail to converge sometimes? When to use/avoid timers while waiting for a result from an action server.

## Install
The following instructions are for the PCs in the lab rooms, which have already been set up for you.
```
# Download the project code:
# From files, download [assignment_5.zip]()

# Unpack it in ~/catkin_ws/src/
# (or using the file browser)
cd ~/catkin_ws/src
unzip ~/Downloads/assignment_5.zip

# Add this line at the end of your .bashrc file and source it:
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/
source ~/.bashrc

# Build the project:
cd ~/catkin_ws
```























Packages for the Assignment 5 of the course DD2410 "Introduction to Robotics".

## Install
You need g++ with c++11 to compile this repo.
Tested on Ubuntu 18.04 and ROS Melodic.
Clone this repository inside your catkin workspace, install the dependencies, build it and run it.
For these instructions, we'll assumed you have created a ws called catkin_ws. 
Run the following commands in a terminal:
```
$ cd ~/catkin_ws/src
$ git clone "this_repo_link" "folder_name"
$ cd 
$ rosdep update
$ rosdep install --from-paths catkin_ws --ignore-src --rosdistro=$ROS_DISTRO -y
$ cd ~/catkin_ws
$ catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo

Add this line at the end of your .bashrc file:
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/

$ source .bashrc
$ source catkin_ws/devel/setup.bash
```
## Run
In order to run the system:
```
$ roslaunch robotics_project gazebo_project.launch
$ roslaunch robotics_project launch_project.launch
```
You should be able to visualize the system in both Rviz and Gazebo and then you're ready to start to work.
See the instructions for the project in Canvas.

## License

This project is licensed under the Modified BSD License - see [LICENSE](https://opensource.org/licenses/BSD-3-Clause) for details

## Acknowledgments

* Based on packages from [PAL Robotics](http://www.pal-robotics.com/en/home/)
