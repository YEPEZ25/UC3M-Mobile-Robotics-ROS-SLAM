#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint

def map_callback(map_msg):
    print('Mapa recibido')
    map_data = map_msg
    select_and_publish_goal(map_data)

def select_and_publish_goal(map_data):
    if map_data is not None:
        print('Eligiendo destino...')
        width = map_data.info.width
        height = map_data.info.height
        goalFound = False
        while not goalFound:
            random_x = randint(0, width - 1)
            random_y = randint(0, height - 1)
            index = random_y * width + random_x
            if map_data.data[index] == 0:
                goalFound = True

        print('Destino elegido. Navegando hasta el punto...')
        
        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        goal_client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = random_x * map_data.info.resolution + map_data.info.origin.position.x
        print("X: ",goal.target_pose.pose.position.x)
        goal.target_pose.pose.position.y = random_y * map_data.info.resolution + map_data.info.origin.position.y
        print("Y: ",goal.target_pose.pose.position.y)
        goal.target_pose.pose.orientation.w = 1.0

        goal_client.send_goal(goal)
        wait = goal_client.wait_for_result()
        print('Punto alcanzado!')


if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)
        
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
