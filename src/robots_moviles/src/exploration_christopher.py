#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint
import numpy as np
import time

class MapExplorer:
    def __init__(self):
        rospy.init_node('map_explorer', anonymous=True)
        self.map_data = None
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.explored_cells = set()

    def map_callback(self, map_msg):
        rospy.loginfo('Mapa recibido')
        self.map_data = map_msg
        self.explore()

    def explore(self):
        if self.map_data is not None:
            rospy.loginfo('Explorando el mapa...')
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution

            # Obtener coordenadas de los puntos no explorados
            unexplored_points = []
            for y in range(height):
                for x in range(width):
                    index = int(y * width + x)  # Convertir a entero
                    if self.map_data.data[index] == -1 and (x, y) not in self.explored_cells:
                        unexplored_points.append([x * resolution + self.map_data.info.origin.position.x,
                                                  y * resolution + self.map_data.info.origin.position.y])

            if unexplored_points:
                # Elegir el punto con la mayor cantidad de valores -1
                max_count_point = max(unexplored_points, key=lambda point: self.count_adjacent_cells(point[0], point[1]))

                rospy.loginfo(f"Navegando hacia ({max_count_point[0]}, {max_count_point[1]})...")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = max_count_point[0]
                goal.target_pose.pose.position.y = max_count_point[1]
                goal.target_pose.pose.orientation.w = 1.0

                self.goal_client.send_goal(goal)
                self.goal_client.wait_for_result()

                rospy.loginfo(f"Posición ({max_count_point[0]}, {max_count_point[1]}) alcanzada!")

                # Agregar el punto explorado a la lista de celdas exploradas
                self.explored_cells.add((max_count_point[0], max_count_point[1]))
            else:
                rospy.loginfo("¡Todos los puntos están explorados!")

            rospy.loginfo('Exploración terminada.')

    def count_adjacent_cells(self, x, y):
        # Contar la cantidad de celdas adyacentes con valor -1 al punto (x, y)
        count = 0
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map_data.info.width and 0 <= ny < self.map_data.info.height:
                    index = int(ny * self.map_data.info.width + nx)  # Convertir a entero
                    if self.map_data.data[index] == -1:
                        count += 1
        return count

if __name__ == '__main__':
    try:
        explorer = MapExplorer()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploración terminada.")
