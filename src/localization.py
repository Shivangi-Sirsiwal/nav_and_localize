#!/usr/bin/env python3
from __future__ import annotations
import math
from statistics import mean
import rospy

from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Header, Bool
from path_planner import PathPlanner
from path_client import PathClient
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from tf import TransformListener



class Localization:
    px=0
    py=0
    pth=0
    covariance_threshold = 0.0045
    covariance = 1

    #current_map = OccupancyGrid()

    def __init__(self):
        """
        Class constructor
        """
        ## Initialize the node and call it "exploration"
        rospy.init_node("localization")
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_covariance)

        # Tell ROS that this node publishes PoseStamped messages on the /move_base_simple/goal topic
        self.spin = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # self.initial_pose_Publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.tf_list = TransformListener()

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(2.0)
        rospy.loginfo("Localization node ready")

        self.localize()



    def localize(self):
        while self.covariance > self.covariance_threshold:
            rospy.logdebug(f"Covariance: {self.covariance}")
            self.rotate()
            rospy.sleep(0.1)
        self.rotate(0)
        rospy.logdebug(f"Successfully Localized!!!!!!!!!!")



    @staticmethod
    def request_map() -> OccupancyGrid:
        rospy.loginfo("Requesting Map")
        rospy.wait_for_service(service="/static_map")
        map = rospy.ServiceProxy("/static_map", GetMap)
        return map().map
   


    def update_covariance(self, pose_covariance_stamped: PoseWithCovarianceStamped):
        cov = pose_covariance_stamped.pose.covariance
        self.covariance = sum(cov) / len(cov)


    
    def rotate(self, speed=1):
        twist = Twist()
        twist.angular.z = speed
        self.spin.publish(twist)



    def run(self):
        rospy.spin()    

        

if __name__ == '__main__':
    Localization().run()

