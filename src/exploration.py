#!/usr/bin/env python3
from __future__ import annotations
import math
from statistics import mean
import rospy
from priority_queue import PriorityQueue

from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header, Bool
from path_planner import PathPlanner
from tf.transformations import euler_from_quaternion
from tf import TransformListener



unexplored = -1
unoccupied_thresh = 30
occupied_thresh = 65



class Exploration:
    px=0
    py=0
    pth=0
    current_map = OccupancyGrid()
    current_cspace = OccupancyGrid()
    def __init__(self):
        """
        Class constructor
        """
        ## Initialize the node and call it "exploration"
        rospy.init_node("exploration")

        # Tell ROS that this node publishes PoseStamped messages on the /move_base_simple/goal topic
        self.target_frontier = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Tell ROS that this node subscribes to bool messages on the '/complete' topic
        # When a message is received, call self.pick_new_frontier
        rospy.Subscriber('/complete', Bool, self.pick_new_frontier)
      
        self.explore_publisher = rospy.Publisher('/exploring', Bool, queue_size = 10)
        self.complete = False
        self.wait_for_cspace = False
        self.go_home = False
        self.tf_listner = TransformListener()
        self.no_frontier_count = 0 

        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        rospy.Subscriber("/cspace", OccupancyGrid, self.update_cspace)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # self.frontier_publisher = rospy.Publisher(name='/path_planner/explored', data_class=GridCells, queue_size=10)
        # self.centroid_publisher = rospy.Publisher(name='/path_planner/frontier', data_class=GridCells, queue_size=10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(2.0)
        rospy.loginfo("Exploration node ready")
        self.starting_place = PathPlanner.world_to_grid(self.current_map, Point(x=self.px, y=self.py))

        self.pick_new_frontier()


    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time(0)
        pose.pose = msg.pose.pose
        self.tf_listner.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(3))

        transform_pose = self.tf_listner.transformPose("/map", pose)
        self.px =transform_pose.pose.position.x
        self.py = transform_pose.pose.position.y
        quat_orig = transform_pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

    
    def update_map(self, mapdata: OccupancyGrid):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.current_map = mapdata
        self.wait_for_cspace = False

    def update_cspace(self, mapdata: OccupancyGrid):
        self.current_cspace = mapdata


    def request_cspace_update(self):
        print("exploration requesting cspace")
        rospy.wait_for_service(service='/cspace_service')
        request = rospy.ServiceProxy("/cspace_service", GetMap)
        if request is not None:
            response = request()
            occupancy = response.map
            return occupancy
        else:
            return self.current_map

    def pick_new_frontier(self, complete=True):

        new_map = self.request_cspace_update()
        rospy.sleep (0.3)

        if not self.complete and new_map is not None:

            print("Start self not complete")
            frontiers = self.find_frontiers(new_map)
            rospy.sleep(1)
            selected_frontier = self.select_frontier(PathPlanner.world_to_grid(new_map, Point(x=self.px, y=self.py)), frontiers)
            if self.current_cspace.data[int(PathPlanner.grid_to_index(self.current_cspace, selected_frontier))] >= 80:
                found_point = False
                counter = 0

                x = selected_frontier[0]
                y = selected_frontier[0]
                while not found_point:
                    counter +=1
                    up = (x, y+counter)
                    down = (x, y-counter)
                    left = (x - counter, y)
                    right = (x + counter, y)
                    up_val = self.current_cspace.data[int(PathPlanner.grid_to_index(self.current_cspace, up))]
                    down_val = self.current_cspace.data[int(PathPlanner.grid_to_index(self.current_cspace, down))]
                    left_val = self.current_cspace.data[int(PathPlanner.grid_to_index(self.current_cspace, left))]
                    right_val =self.current_cspace.data[int(PathPlanner.grid_to_index(self.current_cspace, right))]

                    if up_val < 80: 
                        selected_frontier = up
                        found_point = True
                        break
                    if down_val < 80: 
                        selected_frontier = down
                        found_point = True
                        break
                    if left_val < 80: 
                        selected_frontier = left
                        found_point = True
                        break 
                    if right_val < 80: 
                        selected_frontier = right
                        found_point = True
                        break
            rospy.sleep(3)

            target = PoseStamped()
            target.pose.position.x = PathPlanner.grid_to_world(new_map, selected_frontier).x
            target.pose.position.y = PathPlanner.grid_to_world(new_map, selected_frontier).y

            if not self.go_home:
                target.header.frame_id = "frontier"
            print(f"\t\t\texploration finds grid coordinate : {selected_frontier}")
            self.target_frontier.publish(target)

            rospy.sleep(0.1)
            #print(f"complete status : {self.complete}")
            
            self.wait_for_cspace = True
        self.explore_publisher.publish(True)



    def select_frontier(self, p: tuple[int, int], frontiers: list[tuple[tuple[int, int], int]]) -> tuple[int, int]:
        """
        Selects the optimal frontier to travel to, given the centroid and size of each frontier.
        Optimal frontiers to travel to:
        1. Have a large size;
        2. Are somewhat close by.
        :param p [(int, int)]:                  The x,y grid coordinate of the current location.
        :param frontiers [[((int, int), int)]]: A list of the centroids (in grid coordinates) 
            and sizes (in number of grid cells) for each detected frontier.
        :return [(int, int)]:                   The x,y grid coordinate of the selected frontier.
        """
        print("Enter select frontier...")
        max_weight = float('-inf')
        max_frontier = self.starting_place
        for centroid, size in frontiers:
            distance = PathPlanner.euclidean_distance(p, centroid)
            # TODO: determination of weight is arbitrary and needs to be adjusted
            weight = 6.5 * size - distance
            if weight > max_weight:
                max_weight = weight
                max_frontier = centroid
        if max_weight == float('-inf'):
            self.no_frontier_count += 1
            #if self.no_frontier_count > 10:
            self.complete = True
            self.explore_publisher.publish(False)
            print("No more frontiers, map complete")
            self.go_home = True
        return max_frontier
        

    
    def find_frontiers(self, mapdata: OccupancyGrid) -> list[tuple[tuple[int, int], int]]:
        """
        Given an occupancy grid, find the centroids and sizes of each frontier.
        :param mapdata [OccupancyGrid]: The occupancy grid where free space is 0-30, 
            obstacles are 65-100, and unexplored space is -1.
        :return [[((int, int), int)]]:  A list of the centroids (in grid coordinates) 
            and sizes (in number of grid cells) for each detected frontier.
        """

        mapdata.data = list(mapdata.data)

        # Find all the frontier cells
        frontier_cells = set()     # A set of all frontier cells where the values are the points
        for i in range(mapdata.info.width):
            for j in range(mapdata.info.height):
                p = (i, j)
                if Exploration.is_cell_frontier(mapdata, p):
                    frontier_cells.add(p)

        frontier_grid_cells = GridCells(header=Header(frame_id='map'), cell_width=mapdata.info.resolution, cell_height=mapdata.info.resolution, 
                                        cells=[PathPlanner.grid_to_world(mapdata, p) for p in frontier_cells])
        # self.frontier_publisher.publish(frontier_grid_cells)

        # Group together neighboring frontier cells
        frontier_cells = Exploration.group_frontiers(mapdata, frontier_cells)

        # calculate the centroids and sizes of each frontier
        data = [(Exploration.centroid(frontier), len(frontier)) for frontier in frontier_cells]

        centroid_grid_cells = GridCells(header=Header(frame_id='map'), cell_width=mapdata.info.resolution, cell_height=mapdata.info.resolution, 
                                        cells=[PathPlanner.grid_to_world(mapdata, p[0]) for p in data])
        # self.centroid_publisher.publish(centroid_grid_cells)

        return data
    


    @staticmethod
    def group_frontiers(mapdata: OccupancyGrid, frontier_cells: set[tuple[int, int]]) -> list[list[tuple[int, int]]]:
        """
        Groups together neighboring frontier cells.
        :param  mapdata [OccupancyGrid]:      The occupancy grid where free 
            space is 0-30, obstacles are 65-100, and unexplored space is -1.
        :param frontier_cells [{(int, int)}]: A set of all frontier cells.
        :return [[[(int, int)]]]:             A list of all frontier cells containing
            lists of frontier cells that are grouped together.
        """

        frontier_groups = []
        
        while len(frontier_cells) > 0:

            queue = [frontier_cells.pop()]

            group = []

            while len(queue) > 0:

                cell = queue.pop(0)
                group.append(cell)

                neighbors = PathPlanner.neighbors_of_8(mapdata, cell, 80)

                for neighbor in neighbors:
                    if neighbor in frontier_cells:
                        frontier_cells.remove(neighbor)
                        queue.append(neighbor)

            frontier_groups.append(group)
        
        return frontier_groups
                        

    
    @staticmethod
    def centroid(frontier: list[tuple[int, int]]) -> tuple[int, int]:
        """
        Finds the centroid of a frontier.
        :param frontier [(int, int)]: A list of x, y grid coordinates that make up the frontier.
        :return (int, int):           The x, y grid coordinate of the frontier's centroid.
        """
        return (mean([p[0] for p in frontier]), mean([p[1] for p in frontier]))
    


    @staticmethod
    def is_cell_frontier(mapdata: OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        Determines if a cell is a frontier cell.
        A cell is a frontier if all of these conditions are true:
        1. The cell itself is walkable (its value is above 65);
        2. It has at least one neighbor that is walkable (its value is above 65);
        3. It has at least one neighbor that is unexplored (its value is -1).
        :param mapdata [OccupancyGrid]: The map information.
        :param p       [(int, int)]   : The coordinate in the grid.
        :return        [bool]         : True if the cell is a frontier, false otherwise.
        """

        neighbors = [(p[0]-1, p[1]), (p[0]+1, p[1]), (p[0], p[1]-1), (p[0], p[1]+1)]
        walkable_neighbors = [neighbor for neighbor in neighbors if PathPlanner.is_cell_walkable(mapdata, neighbor, 80)]
        unexplored_neighbors = [neighbor for neighbor in neighbors if Exploration.is_cell_unexplored(mapdata, neighbor)]

        return not Exploration.is_cell_unexplored(mapdata, p) and PathPlanner.is_cell_walkable(mapdata, p, 80) and len(walkable_neighbors) > 0 and len(unexplored_neighbors) > 0
    


    @staticmethod
    def is_cell_unexplored(mapdata: OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        A cell is unexplored if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. Its value is -1, meaning it is unexplored.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is unknown, False otherwise
        """
        ### REQUIRED CREDIT
        in_x_bounds = p[0] >= 0 and p[0] < mapdata.info.height
        in_y_bounds = p[1] >= 0 and p[1] < mapdata.info.width
        is_unexplored = in_x_bounds and in_y_bounds and mapdata.data[PathPlanner.grid_to_index(mapdata, p)] <= unexplored
        return is_unexplored
    
  

    def run(self):
        rospy.spin()



if __name__ == '__main__':
    Exploration().run()