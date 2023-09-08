#!/usr/bin/env python3
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot, sqrt

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub

    # Call service client with path
    new_path = control_client(path).new_path
    setpoint = control_client(path).setpoint
    # rospy.loginfo("The frame is: %s", setpoint.header.frame_id)

    # Transform Setpoint from service client
    while (new_path.poses is not None) and (setpoint.header.frame_id):
        try:
            trans = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time())
            trans_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # Create Twist message from the transformed Setpoint
        msg = Twist()
        msg.angular.z = 4 * atan2(trans_setpoint.point.y, trans_setpoint.point.x)
        msg.linear.x = 0.5 * sqrt(trans_setpoint.point.x ** 2 + trans_setpoint.point.y ** 2)
        if msg.angular.z > max_angular_velocity:
            msg.angular.z = max_angular_velocity
            msg.linear.x = 0
        # elif msg.angular.z < -max_angular_velocity:
        #     msg.angular.z = -max_angular_velocity
        elif msg.angular.z > 0.8 * max_angular_velocity:
            msg.linear.x = 0
        elif msg.linear.x > max_linear_velocity:
            msg.linear.x = max_linear_velocity
        # elif msg.linear.x < -max_linear_velocity:
        #     msg.linear.x = -max_linear_velocity

        # Publish Twist
        pub.publish(msg)
        rate.sleep()

        # Call service client again if the returned path is not empty and do stuff again
        new_path = control_client(new_path).new_path
        setpoint = control_client(new_path).setpoint

    # Send 0 control Twist to stop robot
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)

    # Get new path from action server
    get_path()


def get_path():
    global goal_client

    # Get path from action server
    goal_client.wait_for_server()
    goal_client.send_goal(None)
    goal_client.wait_for_result()
    if goal_client.get_result().path is None:
        exit() # Done
    # Call move with path from action server
    move(goal_client.get_result().path)


if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")

    # Init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Init tf2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
