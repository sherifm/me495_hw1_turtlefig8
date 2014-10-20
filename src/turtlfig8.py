#! /usr/bin/env python

import rospy #import the rospy python client library
from math import pi, sin, cos, sqrt #import desired math functions from math library

from geometry_msgs.msg import Twist, Vector3 #from the ROS gemoetry_msgs library import the message types Twist and Vector3
from turtlesim.srv import TeleportAbsolute #from the turtlesim.srv library import the  service TeleportAbsolute

def get_state(current_time): #the function get_state() calculates the cartesian positions, velocities and accelearations for the time value current_time

        x=3*sin(4*pi*current_time/T) #desired reference trajectory in x-direction
        y=3*sin(2*pi*current_time/T) #desired reference trajectory in y-direction

    	vx=12*pi*cos(4*pi*current_time/T)/T #first derivative of x with respect to time 'current_time' to get desired velocities in x direction
    	vy=6*pi*cos(2*pi*current_time/T)/T #same for y direction

        ax=-48*pi**2*sin(4*pi*current_time/T)/T**2 #second derivative of x with respect to time 'current_time' to get desired velocities in x direction
        ay=-12*pi**2*sin(2*pi*current_time/T)/T**2 # same for y direction

    	return [x,y,vx,vy,ax,ay] #return a vector with all the configuration, velocitiy and acceleration values

def send_cmd_vel(): #this function transforms and publishes the velocities vx and vy in a representation that is compatible with the turtlesim kinematic-car model  
   
    	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # create a publisher 'pub' that publishes to the topic /turtle1/cmd_vel a message of the type Twist and queues up to 10 messages in case of downtime
    	rospy.init_node('send_cmd_vel', anonymous=True) # initialize the node 'send_cmd_vel'
        
    	r=rospy.Rate(10)  #create a rospy.Rate convenience class which makes a best effort at maintaining a particular rate for a loop. 
	current_time = 0
    

	while not rospy.is_shutdown():#run a continuous loop unless a shutdown command is detected			

		current_time = rospy.get_time() #get the current ROS time
		
                state = [] #create an emtpy list
        	state = get_state(current_time) #fill the list with the current position, velocity and acceleration

        	x= state[0]
        	y= state[1]
        	vx = state[2]
        	vy = state[3]
                ax = state[4]
                ay = state[5]

		theta_dot = (ay*vx-ax*vy)/(vx**2+vy**2) #calculate the control input theta_dot of the kinematic car model
                v_forward = sqrt(vx**2+vy**2) #calculate the forward velocity 
        
        	velocities_cmd_vel = Twist(Vector3(v_forward,0,0),Vector3(0,0,theta_dot)) #Prepare a Twist that is compatible with the topic turtle1/cmd_vel
        	rospy.loginfo(velocities_cmd_vel) #ROS logging
        	pub.publish(velocities_cmd_vel) #have the publisher pub p ublish the Twist 'celocities_cmd_vel' to the topic turtle1/cmd_vel

             
        	r.sleep() #sleep just long enough time to meet the rate requirements

if __name__ == '__main__':
    try:
        global T
	
        T= input('In how many seconds would you like your turtle to complete a figure 8?') #Ask the user for the desired circuit lap time
        print('Ok, watch it go in',T,'seconds per lap') #ptint a confirmation

        rospy.wait_for_service('turtle1/teleport_absolute') #wait for the service 'teleport_absolute' which allows for start positioning of the turtle 
        turtle_pos = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute) #create a service caller
        turtle_pos(5.56,5.56,-20) #use the service to position the turtle at given  positions

	
        send_cmd_vel() #run the function 'send_cmd_vel() defined above, 
    except rospy.ROSInterruptException: pass #unless interupted 
