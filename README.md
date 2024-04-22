#EXECUTE


Terminal 1: 

            roslaunch robots_moviles turtlebot3_escenario1.launch

Terminal 2: 
            
            roslaunch turtlebot3_slam turtlebot3_gmapping.launch

Terminal 3: cd catkin_ws/src/turtlebot3/turtlebot3_navigation/rviz/

            rviz -d turtlebot3_navigation.rviz

Terminal 4: 

            roslaunch turtlebot3_navigation move_base.launch

Terminal 5: 

            rosrun robots_moviles exploration.py
