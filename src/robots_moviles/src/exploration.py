#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np

class MapExplorer:
    def __init__(self):
        rospy.init_node('map_explorer', anonymous=True)
        self.map_data = None
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.explored_cells = set()
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.robot_speed = 2  # Ajusta la velocidad del robot según lo desees

    def map_callback(self, map_msg):
        rospy.loginfo('Mapa recibido')
        self.map_data = map_msg
        self.map_resolution = map_msg.info.resolution
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y
        self.explore()

    def explore(self):
        if self.map_data is not None:
            rospy.loginfo('Explorando el mapa...')
            width = self.map_data.info.width
            height = self.map_data.info.height

            # Definir el tamaño de las regiones a explorar
            region_size = 5  # Ajusta el tamaño según lo desees
            num_regions_x = int(np.ceil(width / region_size))
            num_regions_y = int(np.ceil(height / region_size))

            for i in range(num_regions_x):
                for j in range(num_regions_y):
                    # Obtener las coordenadas de la esquina superior izquierda de la región
                    region_x = i * region_size
                    region_y = j * region_size
                    region_center_x = region_x + region_size / 2
                    region_center_y = region_y + region_size / 2

                    # Verificar si la región ya ha sido explorada
                    if (region_center_x, region_center_y) in self.explored_cells:
                         continue

                    # Navegar hacia el centro de la región
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = (region_center_x * self.map_resolution) + self.map_origin_x
                    goal.target_pose.pose.position.y = (region_center_y * self.map_resolution) + self.map_origin_y
                    goal.target_pose.pose.orientation.w = 1.0

                    self.goal_client.send_goal(goal)
                    self.goal_client.wait_for_result()

                    # Marcar la región como explorada
                    self.explored_cells.add((region_center_x, region_center_y))

            rospy.loginfo('Exploración terminada.')

if __name__ == '__main__':
    try:
        explorer = MapExplorer()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploración terminada.")