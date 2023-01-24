#!/usr/bin/env python
# Execute as a python script
# Description: Set linear and angular values for the Turtle.
import rospy				# Needed to create a ROS node -Python client library
# Message type that Turtlesim accepts - usually via the topic cmd_vel
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
from datetime import datetime, timedelta
from turtlesim.srv import TeleportAbsolute


class ControlTurtlesim():

    def __init__(self):
        # self represents the instance of the class.
        # By using the "self" keyword we can access the attributes
        # and methods of the class in python.
        # "__init__" is a reseved method in python classes.
        # It is known as a constructor in object oriented concepts.
        # This method called when an object is created from the class
        # and it allows the class to initialize the attributes of the class.


        rospy.init_node('ControlTurtlesim', anonymous=False)
        # ControlTurtlesim is the name of the node sent to the master
        # You can only have one node in a rospy process,
        # so you can only call rospy.init_node() once!

        # The anonymous argument: The anonymous keyword argument is mainly used for nodes
        # where you normally expect many of them to be running and don't care
        # about their names (e.g. tools, GUIs).
        # It adds a random number to the end of your node's name,
        # to make it unique. Unique names are more important for nodes
        # like drivers, where it is an error if more than one is running.
        # If two nodes with the same name are detected on a ROS graph,
        # the older node is shutdown.

 

		# Message to screen
        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")

        # Keys CTRL + c will stop this script
        rospy.on_shutdown(self.shutdown)

	    # Since we want to move the turtle, this node will pubish a
        # Twist message on topic /turtle1/cmd_vel. lets do that below-

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/turtle1/pose",Pose,self.pose_callback)
        rospy.wait_for_service('turtle1/teleport_absolute')
        turtle1_teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        # rospy.sleep(1)
        # Here we are creating a handle to publish messages to a topic using
        # the rospy.Publisher class.
        # The most common usage for this is to provide the name of the topic
        # and the message class/type of the topic.
        # You can then call publish() on that handle to publish a message.
        # General usage:
        # pub = rospy.Publisher('topic_name', message class, queue_size=10)

	    # Turtlesim will receive our messages 10 times per second.
        rate = rospy.Rate(50);
        # Do not confuse this 10 with the queue_size = 10 above.
	    # 10 Hz is fine as long as the processing does not exceed
        #   1/10 second.

        rospy.loginfo("Set rate 10Hz")
        # We may want the user to specify the rate rather than echo a fixed rate.

        # Twist is geometry_msgs for linear and angular velocity
        # create an object of the class Twist.
        move_cmd = Twist()

	    # Linear speed in x in units/second: positive values imply forward,
        # negative values == backwards
        # move_cmd.linear.x = input('Linear Velocity: ')	# Modify this value to change the Turtle's speed
        move_cmd.linear.x = 2	# Modify this value to change the Turtle's speed
        # Turn at 0 radians/s
        # move_cmd.angular.z = input('Angular Velocity: ')
        move_cmd.angular.z = 3

        # Modify this value to cause rotation rad/s

        first_pose_x = self.pose_x
        first_pose_y = self.pose_y

        delta_time = (2*math.pi) / move_cmd.angular.z
        flag=0

        end_time = datetime.now() + timedelta(seconds=delta_time) 
        error_range = 0.005

        # self.cmd_vel.publish(move_cmd)
        # rospy.sleep(1)
        # distance = 2*(math.pi)*(move_cmd.linear.x/move_cmd.angular.z)
        count = 0

	    # Loop until you type CTRL+c
        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            # rate.sleep()
            if datetime.now() >= end_time:
                #Change Z direction
                print("DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                print('x: ')
                print(round(self.pose_x,4))
                print('y: ')
                print(round(self.pose_y,4))
                self.cmd_vel.publish(Twist())
                # count = count + 1
                # if (count%2) == 0:
                turtle1_teleport(first_pose_x,first_pose_y,0) 

                end_time = datetime.now() + timedelta(seconds=delta_time) 
                # delta_time = delta_time + (-1*error_range)
                move_cmd.angular.z = -move_cmd.angular.z
                self.cmd_vel.publish(move_cmd)
                # self.cmd_vel.publish(Twist())
                # end_time = datetime.now() + timedelta(seconds=delta_time) 
                # rospy.sleep(10)
            rate.sleep()

            


	        #  # publish Twist values to the Turtlesim node /cmd_vel
            #  # handle.publish(message class instance)
            # # self.cmd_vel.publish(move_cmd)
            # # rate.sleep()
            # print('x: ')
            # print(round(self.pose_x,4))
            # print('y: ')
            # print(round(self.pose_y,4))
	        # # wait for 0.1 seconds (10 HZ) and publish again
            # # rate.sleep()

            # # distance = 2*(math.pi)*(move_cmd.linear.x/move_cmd.angular.z)
            # print(distance)
            # print('++++++++++++++++++++++=')
            # # if round(self.pose_x,1) == round(first_pose_x,1) and round(self.pose_y,1) == round(first_pose_y,1):

            # if round(self.pose_x,2) == round(distance,2):
            #     print("DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
            #     print('x: ')
            #     print(round(self.pose_x,4))
            #     print('y: ')
            #     print(round(self.pose_y,4))
            #     rospy.sleep(10)
            
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

    def pose_callback(self, posedata):
        self.pose_x = posedata.x
        self.pose_y = posedata.y
        self.pose_yaw = posedata.theta

    def shutdown(self):
        # You can stop turtlesim_move by publishing an empty Twist message
        rospy.loginfo("Stopping the turtle")

        self.cmd_vel.publish(Twist())

        # Give it some time to stop
        rospy.sleep(1)

if __name__ == '__main__':
    # Try and Except.
    # If an error is encountered, a try block code execution is stopped and
    # transferred down to the except block.
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
