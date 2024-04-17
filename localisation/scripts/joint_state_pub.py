#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import JointState

# Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36
a = l/2
g = 9.8
J = 4/3 * (m * a**2)
Tau = 0.0
x1 = 0.0
x2 = 0.0
dt = 0.0

# Callback functions
def callbackTau(msg):
    global Tau
    Tau = msg.data

# Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

# Utilities
def printValues():
    print("====================================")
    if x1 > 0:
        print("x1 =  ", x1)
    else:
        print("x1 = ", x1)

    if x2 > 0:
        print("x2 =  ", x2)
    else:
        print("x2 = ", x2)

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("SLM_Sim")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers 
    tau_sub = rospy.Subscriber("tau", Float32, callbackTau)
    tau_pub = rospy.Publisher("tau", Float32, queue_size = 10)
    arm_pub = rospy.Publisher("joint_states", JointState, queue_size = 10)

    position = JointState()
    header = Header()

    print("The SLM sim is Running")
    start_time = rospy.get_time()

    try:
        while(1):
            dt = rospy.get_time() - start_time

            # SLM governing equation
            x1 += x2*dt

            x2_dot = (1/(J+m*a**2)) * (-m*g*a*np.cos(x1) - k*x2 + Tau)
            x2 += x2_dot*dt
            
            printValues()

            # After publishing Tau input, reset to 0
            if Tau > 0:
                tau_pub.publish(0)

            position.header.stamp = rospy.Time.now()
            position.name = ["joint2"]
            position.position = [x1]
            position.velocity = [x2]
            arm_pub.publish(position)
            
            start_time = rospy.get_time()

            # Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass # Initialise and Setup node