 – Mobile Robot Navigation Coursework
README 

PART 1 – A* Online Planning (Python)

How to run Part 1:
1. Start online SLAM:
   ros2 launch slam_examples turtlebot_octomap_online_demo.launch.py rviz_ver:=nav octomap_res:=0.1

2. Start the A* planner:
   ros2 launch grid_planners_demo goto_astar_demo.launch.py

Tested locations in the TurtleBot3 world:
-  in the bin room area ( 1.286074, 3.082391)
- moving robot to another room ( 6.374808, -1.190011) 
- take it to the further room in bottom (-5.855394, 3.715197)
- lastly the room to the right (-6.32019 -0.132033)
if goal is in table or in obstacle, it wont move but if you then give it a goal in a valid place it will move.


PART 2 – RRT* Online Planning (Python)

How to run Part 2 (Python RRT*):
1. Start online SLAM:
   ros2 launch slam_examples turtlebot_octomap_online_demo.launch.py rviz_ver:=nav octomap_res:=0.1

2. Start the Python RRT* planner:
   ros2 launch sampling_planners_demo goto_rrt_star_demo.launch.py

Tested locations in the TurtleBot3 world:
- in the bin room area (1.025065 2.920674)
- moving robot to another room ( 6.374808, -1.190011) 
- take it to the further room in bottom (-5.855394, 3.715197)
- lastly the room to the right (-6.32019 -0.132033)

PART 2 – RRT* Online Planning (C++ OMPL)

Tested locations in the TurtleBot3 world:
- in the bin room area (1.025065 2.920674)
- moving robot to another room ( 6.374808, -1.190011) 
- take it to the further room in bottom (-5.855394, 3.715197)
- lastly the room to the right (-6.32019 -0.132033)

How to run Part 2 (C++ OMPL version):
1. Start online SLAM:
   ros2 launch slam_examples turtlebot_octomap_online_demo.launch.py rviz_ver:=nav octomap_res:=0.1

2. Start the C++ OMPL planner:
   ros2 launch sampling_planners_demo goto_ompl_demo.launch.py
   
   To run the new map please use the commands below 
   1. Start online SLAM:
   ros2 launch slam_examples RoboticsNewMap.launch.py

2. Start the C++ OMPL planner:
   ros2 launch sampling_planners_demo goto_ompl_demo.launch.py

My modified packages included in Code_[23025758].zip:
- grid_planners_demo (Part 1)
- sampling_planners_demo (Part 2 Python and C++)

