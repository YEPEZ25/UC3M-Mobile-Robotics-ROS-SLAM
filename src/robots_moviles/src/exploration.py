#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint

# Variables globales para mantener un registro de los puntos explorados
explored_points = set()

def map_callback(map_msg):
    print('Mapa recibido')
    map_data = map_msg
    select_and_publish_goal(map_data)

def select_and_publish_goal(map_data):
    if map_data is not None:
        print('Eligiendo destino...')
        width = map_data.info.width
        height = map_data.info.height

        # Variables para rastrear el mejor destino
        best_x = None
        best_y = None
        best_score = float('-inf')  # Inicializado con el peor puntaje posible

        for y in range(height):
            for x in range(width):
                index = y * width + x

                # Verificar si el punto no ha sido explorado y es un área potencialmente interesante (-1)
                if map_data.data[index] == -1 and (x, y) not in explored_points:
                    # Calcular puntaje para este punto basado en la cantidad de -1 alrededor
                    score = calculate_score(map_data, x, y)

                    if score > best_score:
                        best_score = score
                        best_x = x
                        best_y = y

        if best_x is not None and best_y is not None:
            print('Destino elegido. Navegando hasta el punto...')
            goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = best_x * map_data.info.resolution + map_data.info.origin.position.x
            goal.target_pose.pose.position.y = best_y * map_data.info.resolution + map_data.info.origin.position.y
            goal.target_pose.pose.orientation.w = 1.0

            goal_client.send_goal(goal)
            wait = goal_client.wait_for_result()

            # Agregar el punto a los puntos explorados
            explored_points.add((best_x, best_y))

            print('Punto alcanzado!')
        else:
            print('No se encontraron destinos adecuados. Rotando hacia otra dirección...')
            # Enviar una orden de rotación hacia otra dirección
            goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"  # Rotación relativa al robot
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.orientation.z = 1.0  # Rotar 180 grados

            goal_client.send_goal(goal)
            wait = goal_client.wait_for_result()

def calculate_score(map_data, x, y):
    width = map_data.info.width
    height = map_data.info.height
    score = 0

    # Recorrer los vecinos del punto y contar cuántos son -1
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            nx = x + dx
            ny = y + dy

            if 0 <= nx < width and 0 <= ny < height:
                index = ny * width + nx
                if map_data.data[index] == -1:
                    score += 1

    return score

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)
        
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploración terminada.")
