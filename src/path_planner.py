#!/usr/bin/env python3
from __future__ import annotations
import math
import rospy
from priority_queue import PriorityQueue

from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header,  Bool

unexplored = -1
occupied_thresh = 65

class PathPlanner:
    currentMap = OccupancyGrid()
    current_gradient = OccupancyGrid()
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("path_planner")

        self.service = rospy.Service(name="path_plan", service_class=GetPlan, handler=self.plan_path)

        rospy.Subscriber("/cspace", OccupancyGrid, self.update_map)
        rospy.Subscriber("/gradient", OccupancyGrid, self.update_gradient)
        rospy.Subscriber('/exploring', Bool, self.update_exploring)

        self.path_publisher = rospy.Publisher(name="/path_planner/path", data_class=Path, queue_size=10)
        self.is_exploring = True     

        self.grid_cells_publisher = rospy.Publisher(name='/path_planner/frontier', data_class=GridCells, queue_size=10)

        self.request_counter = 0
        
        rospy.sleep(1.0)




#### Transformations ---------------------------------------------------------------------------------------

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        return mapdata.info.width * p[1] + p[0]


    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """mapdata.data
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
        origin_x = mapdata.info.origin.position.x
        origin_y = mapdata.info.origin.position.y
        resolution = mapdata.info.resolution
        world_x = origin_x + (float(p[0]) + 0.5) * resolution
        world_y = origin_y + (float(p[1]) + 0.5) * resolution
        return Point(x=world_x, y=world_y)


    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        
        origin_x = mapdata.info.origin.position.x
        origin_y = mapdata.info.origin.position.y
        resolution = mapdata.info.resolution
        grid_x = 0
        grid_y = 0
        if(resolution):
            grid_x = int((wp.x - origin_x) / resolution)
            grid_y = int((wp.y - origin_y) / resolution)
        return (grid_x, grid_y)



#### ROS Translators ---------------------------------------------------------------------------------------
    
    def request_cspace_update(self):

        print("exploration requesting cspace")
        rospy.wait_for_service(service='/cspace_service')
        request = rospy.ServiceProxy("/cspace_service", GetMap)
        response = request()
        occupancy = response.map
        return occupancy

    def update_exploring(self, exploring: Bool):
        self.is_exploring = exploring


    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        
        pose_list = []
        for p in path:
            pose = Pose(position=PathPlanner.grid_to_world(mapdata, p))
            pose_list.append(PoseStamped(pose=pose))
        return pose_list


    def update_map(self, mapdata: OccupancyGrid):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.currentMap = mapdata 

    def update_gradient(self, mapdata: OccupancyGrid):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.current_gradient = mapdata 
        


    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        
        return Path(header=Header(frame_id="map"), poses=self.path_to_poses(mapdata, path))
    
    def is_exploring(self, complete : Bool):
        if complete:
            self.is_exploring = False
        else:
            self.is_exploring = True




#### Neighborhood Functions --------------------------------------------------------------------------------

    @staticmethod
    def is_cell_walkable(mapdata: OccupancyGrid, p: tuple[int, int], occupied_threshold: int) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        
        in_x_bounds = p[0] >= 0 and p[0] < mapdata.info.height
        in_y_bounds = p[1] >= 0 and p[1] < mapdata.info.width
        return in_x_bounds and in_y_bounds and mapdata.data[PathPlanner.grid_to_index(mapdata, p)] < occupied_threshold

       

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int], occupied_thresh: int) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        x = int(p[0])
        y = int(p[1])
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        return [neighbor for neighbor in neighbors if PathPlanner.is_cell_walkable(mapdata, neighbor, occupied_thresh)]
      

    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int], occupied_thresh: int) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x, y) in the occupancy grid.
        :param mapdata: The map information (nav_msgs/OccupancyGrid).
        :param p: The coordinate in the grid (Tuple[int, int]).
        :return: A list of walkable 8-neighbors (List[Tuple[int, int]]).
        """
        x = int(p[0])
        y = int(p[1])
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), 
                     (x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)]
        return [neighbor for neighbor in neighbors if PathPlanner.is_cell_walkable(mapdata, neighbor, occupied_thresh)]



#### Map Manipulation --------------------------------------------------------------------------------------

    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int], path_break: int) -> list[tuple[int, int]]:
        
        resolution = mapdata.info.resolution
        rospy.loginfo("entering A*")
        # Keep track of the frontier with a priority queue
        frontier = PriorityQueue()
        frontier.put(start, 0)

        cost_dict = {start: 0}          # The cost of the current shortest known paths
        path_dict = {start: [start]}    # The current shortest known path
        visited = set()                 # The list of visited nodes


        while not frontier.empty():
            node = frontier.get()
            visited.add(node)

            # Exit while loop once the target node is reached
            if node == goal:
                break

            else:
                # Iterate over the current unvisited nodes neighbors
                unvisited_neighbors = set(self.neighbors_of_4(mapdata, node, 65)) - visited
                for neighbor in unvisited_neighbors:
                    # New cost = cost of going to the current node + cost of going from current node to its neighbor
                    cost_to_node = cost_dict[node] + 1

                    # If the cost is less than what's in the table, update the table and enqueue the neighbor
                    if (neighbor not in cost_dict) or (cost_to_node < cost_dict[neighbor]):
                        cost_dict[neighbor] = cost_to_node
                        path_dict[neighbor] = path_dict[node] + [neighbor]
                        #print(f"update path dict : {path_dict}")

                        # The priority when the neighbor is pushed to the priority queue is 
                        # the cost to get to the current node + the estimated cost to get to the goal
                        heuristic = self.euclidean_distance(neighbor, goal) + (self.current_gradient.data[self.grid_to_index(self.current_gradient, neighbor)]**2)
                        estimated_cost = 1*cost_to_node + 1*heuristic
                        frontier.put(neighbor, estimated_cost)
        path = path_dict[goal]
        if path_break > 0:
            del path[path_break:]
        return path



#### Path Management----------------------------------------------------------------------------------------

    @staticmethod
    def dump_waypoints(path_break, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """

        if path_break > 0:
            del path[path_break:]

        # Add first point
        optimized_path = [path[0]]

        # Add all "corner" points
        for i in range(1, len(path)-1):
            previous_point = path[i-1]
            point = path[i]
            next_point = path[i+1]

            # prev_x == current_x and current_y == next_y
            if point[0] == previous_point[0] and point[1] != previous_point[1]:
                if point[0] != next_point[0] and point[1] == next_point[1]:
                    optimized_path.append(point)

            # prev_y == current_y and current_x == next_x
            elif point[0] != previous_point[0] and point[1] == previous_point[1]:
                if point[0] == next_point[0] and point[1] != next_point[1]:
                    optimized_path.append(point)
        
        # Add last point
        optimized_path.append(path[-1])

        return optimized_path

    @staticmethod
    def remove_duplicates(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        seen = set()
        unique_points = []
        for point in points:
            if point not in seen:
                unique_points.append(point)
                seen.add(point)
        return unique_points


    @staticmethod
    def simplify_path(path: list[tuple[int, int]], mapdata) -> list[tuple[int, int]]:
        
        line_path = [path[0]]

        n = 0
        while n < len(path) - 2:
            viable_line_found = False
            for i in range(n + 2, len(path)):
                
                x0, y0 = path[n]
                x2, y2 = path[i]

                my_indexes = []

                # Check for vertical line
                if abs(x2 - x0) <= 0.002:
                    y_range = range(y0, y2 + 1) if y0 < y2 else range(y0, y2 - 1, -1)
                    for y in y_range:
                        my_indexes.append(PathPlanner.grid_to_index(mapdata, (x0, y)))
                else:
                    dx = abs(x2 - x0)
                    dy = abs(y2 - y0)
                    sx = 1 if x0 < x2 else -1
                    sy = 1 if y0 < y2 else -1
                    err = dx - dy

                    x, y = x0, y0
                    diagonal = True
                    while diagonal == True:
                        my_indexes.append(PathPlanner.grid_to_index(mapdata, (x, y)))
                        if x == x2 and y == y2:
                            diagonal = False
                            break
                        e2 = 2 * err
                        if e2 > -dy:
                            err -= dy
                            x += sx
                        if e2 < dx:
                            err += dx
                            y += sy

                        step_size  = 0.1 #smaller is more padding on the diag
                        for a in range(7):
                            #print(f"my x,y = {(x, y)}")
                            fractional_x = x + (sx * a)
                            fractional_y = y + (sy * a)
                            #print(f"\t\t\t\t\t {(fractional_x, fractional_y)}")
                            my_indexes.append(round(PathPlanner.grid_to_index(mapdata, (fractional_x, fractional_y))))
                        if diagonal == False: break

                viable_line = True
                for index in my_indexes:
                    if mapdata.data[index] >= 20:
                        viable_line = False
                        line_path.append(path[n + 1])
                        break

                if viable_line:
                    # If a viable line is found, skip the intermediate points
                  
                    viable_line_found = True
                    break

            if not viable_line_found:
                line_path.append(path[n + 1])
            n = n+1
            rospy.sleep(0.05)

        # Add the last point
        line_path.append(path[-1])

        simplified = PathPlanner.remove_duplicates(line_path)
        return simplified


    @staticmethod
    def no_wiggles(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        
        print("\tEnter no wiggles.....")
        no_wiggle_path = [path[0]]

        n = 0
        for n in range(len(path)-1):
            x0, y0 = path[n]
            x2, y2 = path[n + 1]

            dist = PathPlanner.euclidean_distance((x0, y0), (x2, y2))
            print(f"\t\t\t{dist}")

            if dist >= 1.5:
                no_wiggle_path.append(path[n+1])
                print("add point")
            else:
                print("cut wiggle")
            rospy.sleep(0.05)

        # Add the last point

        simplified = PathPlanner.remove_duplicates(no_wiggle_path)


        return simplified
    

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        
        self.request_counter += 1
        ## Request the map
        ## In case of error, return an empty path
        mapdata = self.request_cspace_update()
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        header = msg.goal.header.frame_id
        goal_cost = mapdata.data[PathPlanner.grid_to_index(mapdata, goal)]
        if mapdata.data[int(PathPlanner.grid_to_index(mapdata, start))] >= 80:
                found_point = False
                counter = 0
                x = start[0] 
                y = start[1] 
                while not found_point:
                    counter +=1
                    up = (x, y+counter)
                    down = (x, y-counter)
                    left = (x - counter, y)
                    right = (x + counter, y)
                    up_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, up))]
                    down_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, down))]
                    left_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, left))]
                    right_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, right))]

                    if up_val < 80: 
                        start = up
                        found_point = True
                        break
                    if down_val < 80: 
                        start = down
                        start = True
                        break
                    if left_val < 80: 
                        start = left
                        found_point = True
                        break 
                    if right_val < 80: 
                        start = right
                        found_point = True
                        break
        if mapdata.data[int(PathPlanner.grid_to_index(mapdata, goal))] >= 80:
                found_point = False
                counter = 0
                x = start[0] 
                y = start[1] 
                while not found_point:
                    counter +=1
                    up = (x, y+counter)
                    down = (x, y-counter)
                    left = (x - counter, y)
                    right = (x + counter, y)
                    up_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, up))]
                    down_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, down))]
                    left_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, left))]
                    right_val = mapdata.data[int(PathPlanner.grid_to_index(mapdata, right))]

                    if up_val < 80: 
                        goal = up
                        found_point = True
                        break
                    if down_val < 80: 
                        goal = down
                        star = True
                        break
                    if left_val < 80: 
                        goal = left
                        found_point = True
                        break 
                    if right_val < 80: 
                        goal = right
                        found_point = True
                        break
        if header == "frontier":
            print("\t\t break path")
            path_break = 25
        else:
            path_break = 0

        print(f"\t\tGoal cost = {goal_cost}")

        print(f"\t\t\tA-star reads grid coordinate : {goal}")

        path  = self.a_star(mapdata, start, goal, path_break)
        ## Optimize waypoints
        print("dumping waypoints ...")
        #waypoints = self.dump_waypoints(path_break, path)
        print("simplifying path ...")
        #direct = self.simplify_path(waypoints, mapdata)
        ## Return a Path message
        print("path sending ...")
        #wiggle_free = self.no_wiggles(path)
        path_msg = self.path_to_message(mapdata, path)
        self.path_publisher.publish(path_msg)
        print("Path planner sent path")
        return path_msg


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
      #  self.calc_cspace(self.currentMap, 1)
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
