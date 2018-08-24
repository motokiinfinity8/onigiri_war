#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
# from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

print "scan copy start"
pub=rospy.Publisher('blue_bot/scan2', LaserScan, queue_size=10)
pub_vel=rospy.Publisher('blue_bot/cmd_vel', Twist,   queue_size=10)
# pub=rospy.Publisher('odometrised', Odometry, queue_size=10)

def callback(arg):
# def callback(pose):
    # cov_east = navsatfix.position_covariance[0]
    # cov_north = navsatfix.position_covariance[4]
    # if cov_east>cov_thresh or cov_north>cov_thresh:
    #     return
    scan = arg
    scan.header.frame_id = "blue_bot/laser2"
    pub.publish(scan)
    # odom = Odometry()
    # odom.pose.pose = pose.pose
    # odom.header = pose.header
    # pub.publish(odom)

def callback_cmdvel(arg):
# def callback(pose):
    # cov_east = navsatfix.position_covariance[0]
    # cov_north = navsatfix.position_covariance[4]
    # if cov_east>cov_thresh or cov_north>cov_thresh:
    #     return
    cmdvel = arg
    if cmdvel.linear.x < -0.0 :
       cmdvel.angular.z = cmdvel.angular.z * -1.0
    pub_vel.publish(cmdvel)
    # odom = Odometry()
    # odom.pose.pose = pose.pose
    # odom.header = pose.header
    # pub.publish(odom)

def main():
    rospy.init_node('copy_scan')

    # global cov_thresh
    # cov_thresh = rospy.get_param("~cov_thresh", 10**2)
    rospy.Subscriber("blue_bot/scan", LaserScan, callback)
    rospy.Subscriber("blue_bot/cmd_vel2", Twist, callback_cmdvel)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
