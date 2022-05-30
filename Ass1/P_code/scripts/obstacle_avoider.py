#! /usr/bin/env python

"""
.. module:: obstacle_avoider
	:platform: Unix
	:synopsis: Python modeule for the avoiding obstacles
	
.. moduleauthor:: Pia Saade <piasaade@outlook.com>
subscribes to :
	/remap_cmd_vel
	/scan

publishes to :
	/turtle1/cmd_vel
	
The node controls the robot under specific conditions in a way to keep it safe and far from obstacles


"""


import rospy
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

#initializing the angulare an linear velocities

init = Vector3(0, 0, 0)
repost = Twist( init, init)



def take_action(info):
	# receiving the information from cmd_vel topic and publishet back to the robot under specific conditions
    """
	take_action Function sends the info responsible of the new position of the robot and according to the conditions and ranges fixed the robot will be controlled
	args:
    info(repost): the angular and linear velocity 
    """
    global repost
    pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
    """
    publishing the angular and linear velocity to the robot under specific conditions

    """
    regions = {
    'fright': min(min(info.ranges[144:287]), 10),
    'front':  min(min(info.ranges[288:431]), 10),
    'fleft':  min(min(info.ranges[432:575]), 10),
    }


    if regions['front'] < 0.5:
	# if the distance is less than 0.5 stop the robot from driving forward
        repost.linear.x = 0
        
    elif regions['fleft'] < 0.5 or regions['fright'] < 0.5:
	#if the distance in the left and right regions is less than 0.5 the robot stops its rotation
        repost.angular.z = 0

    pub.publish(repost)

def clbk_remap(info):
    # getting the information from this topic responsible of the angular an linear velocity coming from the robot
    """
    clbk_remap Function getting the velocity and making it a global one to be used in the entire code
    args:
        info(repost): the angular and linear velocity 
    """
    global repost
    repost = info



def main():


    rospy.init_node('obstacle_avoidance')

    rospy.Subscriber('/remap_cmd_vel', Twist, clbk_remap)

    rospy.Subscriber('/scan', LaserScan, take_action)

    rospy.spin()


if __name__ == '__main__':
    main()
