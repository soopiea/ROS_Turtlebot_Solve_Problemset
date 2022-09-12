#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):

    if msg.ranges[359] > 1.0:
        move_cmd.angular.z = straight_angular_z
        move_cmd.linear.x = straing_linear_x
    # has obstacle . move left or if to the right has obstacle
    if msg.ranges[359] < 1.0 or msg.ranges[0] < 1.0:
        move_cmd.angular.z = left_turn_angular_z
        move_cmd.linear.x = left_turn_linear_x
    # has obstacle on left . move right
    if msg.ranges[719] < 1.0:
        move_cmd.angular.z = straight_angular_z
        move_cmd.linear.x = straing_linear_x
    pub.publish(move_cmd)


rospy.init_node('topics_quiz_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback)
range_sub = 0  # Initialize global range_sub
move_cmd = Twist()

straight_angular_z = 0.0
straing_linear_x = 1.0

left_turn_angular_z = 1.0
left_turn_linear_x = 0.2

right_turn_angular_z = -1.0
right_turn_linear_x = 0.2

rate = rospy.Rate(2)

rospy.spin()


#Launch file:

# <launch>
#   <!-- turtlebot_teleop_key already has its own built in velocity smoother -->
#   <node pkg="topics_quiz" type="my_script.py" name="topics_quiz_node"  output="screen">
#   </node>
# </launch>