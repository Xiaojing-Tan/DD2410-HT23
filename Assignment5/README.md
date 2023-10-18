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

<video width="465" height="261" preload="metadata" src="https://dub.cdn.nv.instructuremedia.com/originals/o-4RBxaWg8ACwJc7B8KudS3LJbjeKKKZuu/transcodings/t-4SiDKKu69Zk6XV7jSpR6HaF1YuCmR8pQ.mp4?&Expires=1697674771&Signature=MqFUmJaLI86FJwGrzbn~PXEQ9gxACYp3WYKuYve1VmI5uPGyO7LH5GJh1~qtQDNjC73t6UBpxj0INPi9Ku~56nZ1yWjeCBBvo2jL8ZMCnhoBr-wFZdqKF4zXLp~k47Ygp9xFjRu3RbQRksNQlpzChDSMECJ9J4lDpmfPqdlFHzOVaTsgzvREd~mz4xLPtFA79UMv41QBZIS6Y5T2X90HL6BR8q7DDTOsnlik~AjN44FW0rjBTQnGONn9pcIvjivuhyzxbYrjPYuHtULTamrVaSXYuPIKPsbRqZPJvM0CL7upvVXIJw9hwWm9wCAubzx5aCjBjSblZ1Mg~QQMSD2aOA__&Key-Pair-Id=APKAJLP4NHW7VFATZNDQ"></video>



























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
