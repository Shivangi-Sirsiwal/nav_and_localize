#!/usr/bin/env python3
import rospy
from nav_msgs.msg import GridCells, OccupancyGrid
from nav_msgs.srv import GetMap
from path_planner import PathPlanner
occupied_thresh = 65

class CSpaceCalc:
    current_map = None
    current_cspace = None
    current_gradient = None
    def __init__(self):
         rospy.init_node('cspace_calc')

         self.cspace_publisher = rospy.Publisher('/cspace', OccupancyGrid, queue_size=10)
         self.gradient_publisher = rospy.Publisher('/gradient', OccupancyGrid, queue_size=10)
         self.service = rospy.Service(name='/cspace_service', service_class=GetMap, handler=self.publish_cspace)
         rospy.sleep(1.0)
         rospy.Subscriber('/map', OccupancyGrid, self.calc_cspace)     
         print("Cspace node ready")     

    def publish_cspace(self, josh=4):
        if self.current_cspace is not None:
            self.cspace_publisher.publish(self.current_cspace)
        rospy.sleep(0.05)
        if self.current_gradient is not None:
            self.gradient_publisher.publish(self.current_gradient)
            print("published c-space")
            return self.current_cspace
        else:
            return OccupancyGrid()

    def calc_cspace(self, mapdata: OccupancyGrid):
            """
            Calculates the C-Space, i.e., makes the obstacles in the map thicker.
            Publishes the list of cells that were added to the original map.
            :param mapdata: The map data (nav_msgs/OccupancyGrid).
            :param padding: The number of cells around the obstacles.
            :return: The C-Space (OccupancyGrid).
            """
            self.current_map = mapdata
            if self.current_map is not None:
                padding = 2
                print("Calculating c-space")
                cspace = OccupancyGrid()
                cspace.data = list(mapdata.data)
                cspace.info = mapdata.info
                gradient = OccupancyGrid()
                gradient.info = mapdata.info
                gradient.data = list(mapdata.data)

                ## Go through each cell in the occupancy grid
                obstacles = set()
                for i in range(cspace.info.width):
                    for j in range(cspace.info.height):
                        if cspace.data[PathPlanner.grid_to_index(cspace, (i, j))] > occupied_thresh:
                            obstacles.add((i, j))

                ## Inflate the obstacles where necessary
                for p in range(padding + 14):
                    new_obstacles = set()
                    for obstacle in obstacles:
                        neighbors = PathPlanner.neighbors_of_8(gradient, obstacle, 10) 
                        for neighbor in neighbors:
                            if(p>padding):
                                gradient.data[PathPlanner.grid_to_index(gradient, neighbor)] = (10 - (p-padding))*5 #max value of 63, allowing everything outside of padding to be navigated through, with modified cost
                            else:
                                cspace.data[PathPlanner.grid_to_index(cspace, neighbor)] = 80
                                gradient.data[PathPlanner.grid_to_index(gradient, neighbor)] = 80
                        new_obstacles.update(neighbors)
                    obstacles.update(new_obstacles)

                ## Return the C-space
                self.current_cspace = cspace
                self.current_gradient = gradient
                self.cspace_publisher.publish(cspace)
                rospy.sleep(0.05)
                self.gradient_publisher.publish(gradient)
                print("published c-space")



    def run(self):
        rospy.spin()



if __name__ == '__main__':
    CSpaceCalc().run()
