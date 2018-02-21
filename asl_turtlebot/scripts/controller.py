#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi


class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
	rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def gazebo_callback(self, data):
        pose = data.pose[data.name.index("turtlebot3_burger")]
        twist = data.twist[data.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def get_ctrl_output(self):
        # use self.x self.y and self.theta to
	# compute the right control input here

	#This part of of the code incorporates an optimal control law 
	#that was developed in an earlier part of the assignment 
	#the desired position of x_g = 1, y_g = 1, and th_g = 0 are hard coded
        k1 = 1
        k2 = 1
        k3 = 1

        x_g =1
        y_g =1
        th_g =0
        dx = x_g - self.x
        dy = y_g - self.y
        rho = np.hypot(dy,dx)
        alpha = np.arctan2(dy,dx) - self.theta
        delta = np.arctan2(dy,dx) - th_g

        #Define control inputs (V,om) - without saturation constraints
	#control law developed using optimal control theory
        V = k1 * rho * np.cos(alpha)
        om = (k2*alpha)+(((k1*np.sin(alpha)*np.cos(alpha))/alpha)*(alpha+(k3*delta)))

        #with saturation
        V = np.sign(V)*min(0.5, np.abs(V))
        om = np.sign(om)*min(1, np.abs(om))

        cmd_x_dot = V
        cmd_theta_dot = om
	



        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
