#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

def pose_callback(posedata):
    global x,y,z,yaw
    x = posedata.x
    y = posedata.y
    yaw = posedata.theta


# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

if __name__ == '__main__':
    try:
        # intialise node, node name
        rospy.init_node("my_turtlesim_pose",anonymous=True)

        #call subscriber - read values
        rospy.Subscriber("/turtle1/pose",Pose,pose_callback)
        time.sleep(2)
        rate=rospy.Rate(5)

        while not rospy.is_shutdown():
            print('x: ')
            print(x)
            print('y: ')
            print(y)

            rate.sleep()

        # time.sleep(2)

        # print('x: ')
        # print(x)
        # print('y: ')
        # print(y)

        # rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")

