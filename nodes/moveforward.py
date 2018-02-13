#!/usr/bin/env python

'''
Moves the robot specified meters
'''

import rospy
from geometry_msgs.msg import Twist

class TurtlebotMover:
    """TurtlebotMover"""
    def __init__(self):
        rospy.init_node('TurtlebotMover', anonymous=True)
        rospy.loginfo("Stop the node using Ctrl+C")
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10) 
        self.rate = rospy.Rate(10)

    def run(self):
        rospy.log("Moving Turtlebot 1m")
        rospy.log("Rate: %s" % self.rate)


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        TurtlebotMover().run()
    except:
        rospy.loginfo("GoForward node terminated.")
