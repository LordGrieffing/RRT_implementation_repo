#!/usr/bin/python3

# If you found this file inside of the RRT_implementation_repo it is because I forgot to move it. 
# This script will not work inside of RRT_implementation_repo, it was intended for ROS-Multi-robot-exploration-turtlebot3

# -- python imports
import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys , heapq , math , copy
from collections import defaultdict
import networkx as nx

# -- ROS imports
# ---- ROS framework
import rospy , tf2_ros

# ---- ROS messages
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point, Transform, PoseStamped
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID

# ---- ROS services
from turtlebot3_exploration_multi.srv import ConvertMapGraph

# -- local imports
from int_wrapper import Wrapper
from bresenham_line import check_obstacles
from get_exploration_metrics import ExplorationMetrics
from multi_robot_next_target import find_target_nrobots
import main

class maptoimage:

    def __init__(self):

        
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
        rospy.sleep(0.4) 
        #print(self.occ_map)

    def callback_map(self, msg: OccupancyGrid) -> None:
        ''' Callback function for map data '''
        self.occ_map = msg

        self.resolution = msg.info.resolution # resolution of the map
        
        self.origin = msg.info.origin.position # coordinate of the map origin of (0,0) left bottom in odom coordinates
                                               # Is a Point object

        map_width = msg.info.width
        map_height = msg.info.height

        self.occ_map_data_array = (  np.array(msg.data)  ).reshape(map_height, map_width)

    def convert_to_two_color(self, map_array):
        map_binary = np.zeros(np.shape(map_array), dtype=np.uint8)
        map_binary[np.where(map_array == 0)] = 255
        return map_binary  
    

    def handle_graph_conversion(self, msg):
        res = msg.info.resolution
        map_width = msg.info.width
        map_height = msg.info.height
        global origin_x, origin_y
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        map_data = msg.data
        map_data_array = np.array(map_data)
        map_data_array = map_data_array.reshape(map_height, map_width)

         # convert to uint8 image
        t1 = rospy.get_time()
        map_data_uint8 = self.convert_to_two_color(map_data_array)
        t2 = rospy.get_time()
        rospy.loginfo("Time to convert:: to two color: "+ str(t2 - t1))

        return map_data_uint8
    
    def display_map(self, map_binary):
        plt.imshow(map_binary, cmap="gray")
        plt.title("Map from Rviz")
        plt.axis('off')
        plt.show()

    # -- Not sure if this function is needed but it might do more in the future, Right now its just to help organize
    def make_map_cv2(self, map_binary):
        img = cv2.cvtColor(map_binary, cv2.COLOR_BGR2RGB)
        return img

    

def main():

    # -- Get map from map message
    map = maptoimage()
    map_data = map.handle_graph_conversion(map.occ_map)

    # -- Convert map to cv2 img
    img = map.make_map_cv2(map_data)

    # -- Initialize variables
    tree = nx.Graph
    stepSize = 10
    

if __name__ == "__main__":
    main()