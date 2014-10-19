#! /usr/bin/env python

import rospy
from math import pi, sin, cos, sqrt
import math
import rosbag

from geometry_msgs.msg import Twist, Vector3 
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute




def get_state(current_time):

        x=3*sin(4*pi*current_time/T)
        y=3*sin(2*pi*current_time/T)

    	vx=12*pi*cos(4*pi*current_time/T)/T
    	vy=6*pi*cos(2*pi*current_time/T)/T

        ax=-48*pi**2*sin(4*pi*current_time/T)/T**2
        ay=-12*pi**2*sin(2*pi*current_time/T)/T**2

    	return [x,y,vx,vy,ax,ay]

def send_cmd_vel():
   
    	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    	rospy.init_node('send_cmd_vel', anonymous=True)
        
    	r=rospy.Rate(10) 
	frequency =10
	current_time = 0
    

	while not rospy.is_shutdown():			

		current_time = rospy.get_time()
		
                state = []
        	state = get_state(current_time)

        	x= state[0]
        	y= state[1]
        	vx = state[2]
        	vy = state[3]
                ax = state[4]
                ay = state[5]

		theta_dot = (ay*vx-ax*vy)/(vx**2+vy**2)
                v_forward = sqrt(vx**2+vy**2)
        
        	velocities_cmd_vel = Twist(Vector3(v_forward,0,0),Vector3(0,0,theta_dot))
        	rospy.loginfo(velocities_cmd_vel)
        	pub.publish(velocities_cmd_vel)

             
        	r.sleep()

if __name__ == '__main__':
    try:
        global T
	
        T= input('In how many seconds would you like your turtle to complete a figure 8?')
        print('Ok, watch it go in',T,'seconds per lap')

        rospy.wait_for_service('turtle1/teleport_absolute')
        turtle_pos = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        turtle_pos(5.56,5.56,-20)

	
        send_cmd_vel()
    except rospy.ROSInterruptException: pass
