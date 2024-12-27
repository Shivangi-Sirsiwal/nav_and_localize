#!/usr/bin/env python3

from __future__ import annotations
import rospy
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Twist, Point
from path_planner import PathPlanner
from tf.transformations import euler_from_quaternion
from tf import TransformListener
import time

class PathClient:

    px = 0
    py = 0
    pth = 0

    def __init__(self):
        """
        Class constructor
        """

        self.currentMap = OccupancyGrid
        
        ### Initialize node, name it 'service_call'
        rospy.init_node('service_call')

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        rospy.Subscriber("/cspace", OccupancyGrid, self.update_map)
        self.tf_listner = TransformListener()

        self.tf_list = TransformListener()

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.request_path
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.request_path)

        self.complete = rospy.Publisher('/complete', Bool, queue_size=10)

        rospy.sleep(0.5)

    #ROS Communication:

    def update_map(self, mapdata: OccupancyGrid):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.currentMap = mapdata 
        print("new map!!!!!!!!!!")

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]lf.pth   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### Make a new Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        ### Publish the message
        self.cmd_vel.publish(twist)



    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        print(f"FRAME ID: {msg.header.frame_id}")
        ### REQUIRED CREDIT
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time(0)
        pose.pose = msg.pose.pose
        self.tf_listner.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(3))

        transform_pose = self.tf_listner.transformPose("/map", pose)
        self.px =transform_pose.pose.position.x
        self.py = transform_pose.pose.position.y
        print(f"{self.px}, {self.py}")
        quat_orig = transform_pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw



    def request_path(self, goal: PoseStamped):

        print("path client requesting path")
        start = PoseStamped()
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        print(f"({self.px}, {self.py}) -> ({goal.pose.position.x}, {goal.pose.position.y})")
        rospy.wait_for_service(service="/path_plan")
        request = rospy.ServiceProxy("/path_plan", GetPlan)
        response = request(start, goal, 0)
        self.follow_path(response.plan.poses)
        self.complete.publish(Bool(False))


    @staticmethod
    def remove_duplicates(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        seen = set()
        unique_points = []
        for point in points:
            if point not in seen:
                unique_points.append(point)
                seen.add(point)
        return unique_points



#### Path Housekeeping --------------------------------------------------------------------------------------


#### Calculators --------------------------------------------------------------------------------------------

    @staticmethod
    def fix_heading(current_heading, goal_heading):
        """
        Calculate the heading angle error between the provided current heading and goal heading
        :param current_heading [float] current heading angle
        :param goal_heading    [float] desired heading angle
        """
        # Find the heading error
        heading_error = goal_heading - current_heading
        # Normalize the heading error to [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        return heading_error
    
    @staticmethod
    def angle_between(x1: float, y1: float, x2: float, y2: float):
        return math.atan2((y2-y1), (x2-x1))


    def calculate_dist(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def world_to_robot(self, x_w: float, y_w: float) -> tuple[float,float]:
        x_r = (x_w-self.px)*math.cos(self.pth) + (y_w-self.py)*math.sin(self.pth)
        y_r = (y_w-self.py)*math.cos(self.pth) - (x_w-self.px)*math.sin(self.pth)
        return (x_r,y_r)

    def calc_speed(self, error, prev_error, prev_t, integrator, kp, ki, kd, max_speed):
        """
        Applies PID controller to derive speed
        :param error      [float] Linear or angular error
        :param prev_error [float] The last linear/angular error
        :param prev_t     [float] The last time this function was called for linear or angular speed
        :param integrator [float] The current integrator for linear/angular speed
        :param kp         [float] Proportional control value
        :param ki         [float] Integral control value
        :param kd         [float] Derivative control value
        :param max_speed  [float] Maximum allowed speed
        """
        t = time.time()
        time_diff = t - prev_t
        if time_diff <= 0:  # Avoid division by zero
            time_diff = 1e-3

        integrator += error * time_diff
        derivative = (error - prev_error) / time_diff
        speed = kp * error + ki * integrator + kd * derivative

        # Clamp speed and integrator
        speed = max(min(speed, max_speed), -max_speed)
        integrator = max(min(integrator, 10), -10)

        return t, integrator, speed


    def drive_controller(self, goal_x, goal_y):
        """
        Calculate linear and angular speeds using PID and send speeds to the robot to move to the provided goal point
        :param x_goal [float] x coordinate of the destination point
        :param y_goal [float] y coordinate of the destionation pont
        """
        # Init all PID values
        lin_integrator = 0
        ang_integrator = 0
        prev_lin_error = 0
        prev_ang_error = 0

        # Init break condition
        goal_reached = False
        #Init linear and angular previos times
        prev_lt = time.time()
        prev_at = time.time()

        while not goal_reached:
            # Check heading
            desired_heading = math.atan2(goal_y - self.py, goal_x - self.px)
            heading_error = self.fix_heading(self.pth, desired_heading)

            # Check if the heading error meets some threshold for correction
            if abs(heading_error) >= 0.04:
                # Get angular speed from PID controller
                t, ang_integrator, ang_speed = self.calc_speed(heading_error, prev_ang_error, prev_at, ang_integrator, 0.15, 0.08, 1.0, 1.0)

                # PID parameter updates
                prev_ang_error = heading_error
                prev_at = t
            else:
                ang_speed = 0
            rospy.sleep(0.05) 
            # Calculate current distance to goal
            lin_error = self.calculate_dist(self.px, self.py, goal_x, goal_y)

            # Check break condition with a deadband
            if lin_error <= 0.03:
                print("\t\t I'm here!")
                rospy.sleep(0.05)
                goal_reached = True
                break

            # Get speed from the PID controller
            t, lin_integrator, lin_speed = self.calc_speed(lin_error, prev_lin_error, prev_lt, lin_integrator, 0.1, 0.1, 1.0, 0.1)

            # PID parameter updates
            prev_lin_error = lin_error
            prev_lt = t
            
            #scales down linear speed based on angular speed --> higher angular speed = lower linear speed
            lin_speed = max(0, lin_speed - (1.2*lin_speed*ang_speed))
            ang_speed = max(-0.17, ang_speed)
            ang_speed = min(0.17, ang_speed)

            self.send_speed(0.02, ang_speed)
            rospy.sleep(0.05)

        # Ensure the robot is stopped
        rospy.sleep(0.05)

    def rotate_to_heading(self, heading: float):
        print("Enter Rotate")
        ang_integrator = 0
        prev_ang_error = 0
        heading_reached = False
        prev_at = time.time()

        print(f"Start Heading : {round(math.degrees(self.pth), 5)}, Goal Heading : {round(math.degrees(heading), 5)}")

        while not heading_reached:
            heading_error = self.fix_heading(self.pth, heading)

            # Check if the heading error meets some threshold for correction
            if abs(heading_error) >= 0.1:  # Introduce a small dead zone to prevent oscillation
                # Get angular speed from PID controller
                t, ang_integrator, ang_speed = self.calc_speed(heading_error, prev_ang_error, prev_at, ang_integrator, 0.15, 0.08, 0.7, 1.0)  # Reduced Kp and adjusted Ki and Kd

                # PID parameter updates
                prev_ang_error = heading_error
                prev_at = t
            else:
                ang_speed = 0
                heading_reached = True
            ang_speed = max(-0.1, ang_speed)
            ang_speed = min(0.1, ang_speed)
            self.send_speed(0, ang_speed)
            rospy.sleep(0.05)
        rospy.sleep(0.05)
        print("Leave Rotate")

#### Movers -------------------------------------------------------------------------------------------------

    def follow_path(self, path: list[PoseStamped]):
        """
        Drives the robot along a path.
        Assumes that the robot is starting at the first PoseStamped.
        :param path [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        done = False
        while not done:
            min_dist = 100000.0;
            lookahead_dist = 0.02
            closest_index = 0
            for i in range(len(path)):
                dist = PathPlanner.euclidean_distance((self.px,self.py),(path[i].pose.position.x,path[i].pose.position.y))
                if dist < min_dist:
                    min_dist = dist
                    closest_index = i
            #closest_to_lookahead = 0
            #min_lookahead_error = 100000.0
            #for i in range(closest_index,len(path)):
            #    dist = PathPlanner.euclidean_distance((self.px,self.py),(path[i].pose.position.x,path[i].pose.position.y))
            #    if (abs(dist - lookahead_dist) < min_lookahead_error):
            #        closest_to_lookahead = i

            goal_pose = path[min((closest_index + 7), (len(path) - 1))]
            
            if abs(self.angle_between(self.px,self.py,goal_pose.pose.position.x,goal_pose.pose.position.y)) > 0.1:
                desired_heading = math.atan2((goal_pose.pose.position.y - self.py), (goal_pose.pose.position.x - self.px))
                self.rotate_to_heading(desired_heading)
            self.drive_controller(goal_pose.pose.position.x, goal_pose.pose.position.y)
            rospy.sleep(0.2)
            if PathPlanner.euclidean_distance((self.px,self.py),(path[-1].pose.position.x,path[-1].pose.position.y)) < 0.06: 
                done = True
        self.send_speed(0,0)


    



    def run(self):
        rospy.spin()



if __name__ == '__main__':
    PathClient().run()