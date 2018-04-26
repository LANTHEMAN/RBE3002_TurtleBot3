For mapping in Gazebo simulation:
1. roslaunch rbe3002_d2018_final_gazebo practice_maze.launch
2. roslaunch rbe3002_d2018_final_gazebo practice_exploration.launch
3. python FrontierNode.py
4. rosrun rviz rab3/rviz/rviz2.rviz 
5. rosrun map_server map_saver -f practiceMaze1  "Do this once the map is complete"


For Navigation in simulation:
1. roslaunch rbe3002_d2018_final_gazebo practice_maze.launch
2. roslaunch rbe3002_d2018_final_gazebo final_run.launch map_file:=$(find lab3)/maps/practiveMaze1.yaml
3. python Drive.py
4. python requestPathServer.py 
5. rosrun rviz rab3/rviz/rviz2.rviz 


