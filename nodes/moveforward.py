#!/usr/bin/env python

'''
Moves the robot specified meters
'''

import rospy


class TurtlebotMover:
    """TurtlebotMover"""
    def __init__(self):
        rospy.init_node('TurtlebotMover', anonymous=True)
        self.rate = rospy.Rate(10)

    def run(self):
        rospy.log("Moving Turtlebot 1m")
        rospy.log("Rate: %s" % self.rate)


if __name__ == '__main__':
    try:
        TurtlebotMover().run()
    except:
        rospy.loginfo("GoForward node terminated.")
