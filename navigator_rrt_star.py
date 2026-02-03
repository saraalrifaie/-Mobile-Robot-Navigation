#!/usr/bin/env python3


import time
import rclpy
import rclpy.node


from collision_checker import CollisionChecker
from node import Node
from planners.rrt_star import RRTstar


from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, Buffer





class Navigator(rclpy.node.Node):
   """Manages the incoming start-to-goal query, solves it by requesting a map
   and sends the path to the controller to follow. Extended to support
   ONLINE RRT* replanning with updated SLAM map.
   """


   def __init__(self):
       super().__init__("navigator_node")


       #! Start and goal nodes for the query
       self.start = None
       self.goal = None


       self.collision_checker = None
       self.map_ = None


       # extra state for online planning
       self.current_path_nodes = []
       self.replan_timer = None


       # ========================================
       #! Flags to check the status of the robot, the query and the map
       self.is_goal_cancelled = False
       self.is_goal_reached = False
       self.is_map_loaded = False
       self.is_robot_moving = False


       # ! Planner parameters
       self.step_size = (
           self.declare_parameter("step_size", 0.001)
           .get_parameter_value()
           .double_value
       )
       self.expansion_size = (
           self.declare_parameter("expansion_size", 0.5)
           .get_parameter_value()
           .double_value
       )
       self.solve_time = (
           self.declare_parameter("solve_time", 5.0)
           .get_parameter_value()
           .double_value
       )
       self.bias = (
           self.declare_parameter("bias", 0.5)
           .get_parameter_value()
           .double_value
       )


       self.goal_tolerance = (
           self.declare_parameter("goal_tolerance", 0.1)
           .get_parameter_value()
           .double_value
       )  # tolerance of the solution path to the goal


       # ? flag to decide use of path smoother
       self.use_path_smoother = (
           self.declare_parameter("use_path_smoother", False)
           .get_parameter_value()
           .bool_value
       )


       self.declare_parameter("max_planning_bounds", [20, 20])
       self.max_planning_bounds = self.get_parameter("max_planning_bounds").value


       # ========================================
       #! Information about the robot
       self.robot_position = None
       self.robot_orientation = None
       self.robot_radius = (
           self.declare_parameter("robot_radius", 0.15)
           .get_parameter_value()
           .double_value
       )


       # ========================================
       self.map_topic_ = (
           self.declare_parameter("map_topic", "/map")
           .get_parameter_value()
           .string_value
       )


       self.tf_buffer = Buffer()
       self.transform_listener = TransformListener(self.tf_buffer, self)


       time.sleep(1)


       #! SUBSCRIBERS
       self.start_subscriber = self.create_subscription(
           Odometry, "/odom", self.robot_pose_callback, 10
       )
       self.goal_subscriber = self.create_subscription(
           PoseStamped, "/goal_pose", self.goal_query_callback, 10
       )


       # ? subscribers to communicate with the controller
       self.robot_moving_subscriber = self.create_subscription(
           Bool, "/robot_is_moving", self.robot_moving_callback, 10
       )


       self.goal_reached_subscriber = self.create_subscription(
           Bool, "/goal_reached", self.goal_reached_callback, 10
       )


       self.map_subscriber = self.create_subscription(
           OccupancyGrid, self.map_topic_, self.map_callback, 10
       )


       # =======================================
       #! PUBLISHERS
       # ? publishers to communicate with the controller

       self.path_publisher = self.create_publisher(Path, "/path", 10)
       self.stop_motion_publisher = self.create_publisher(Bool, "/stop_motion", 10)
       self.goal_publisher = self.create_publisher(PoseStamped, "/goal_controller", 10)
       self.tree_publisher = self.create_publisher(Marker, "/rrt", 10)


       # ========================================


  
   def goal_reached_callback(self, goal_reached: Bool):
       self.is_goal_reached = goal_reached.data


   def robot_moving_callback(self, robot_moving: Bool):
       self.is_robot_moving = robot_moving.data


   def robot_pose_callback(self, data: TFMessage):
       try:
           transform = self.tf_buffer.lookup_transform(
               "map", "base_link", rclpy.time.Time()
           )
           position = transform.transform.translation
           quaternion = transform.transform.rotation
       except Exception:
           return


       self.robot_position = [position.x, position.y, position.z]
       self.robot_orientation = [
           quaternion.x,
           quaternion.y,
           quaternion.z,
           quaternion.w,
       ]


   def goal_query_callback(self, data: PoseStamped) -> bool:
       self.stop_motion_publisher.publish(Bool(data=True))
       time.sleep(0.15)


       if self.is_robot_moving:
           self.cancel_goal()


       self.goal = data.pose
       self.is_goal_cancelled = False
       self.is_goal_reached = False


       self.get_logger().info("Received new goal")
       self.goal_publisher.publish(data)
       return self.planning_callback()


   def map_callback(self, map: OccupancyGrid):
       self.map_ = map
       self.is_map_loaded = True


       self.collision_checker = CollisionChecker(
           self.map_, self.robot_radius, self.max_planning_bounds
       )


   def _project_goal_to_free_space(self, goal_node: Node,
                                   max_offset: float = 0.6,
                                   step: float = 0.05):
       if self.collision_checker is None:
           return None


       gx, gy = goal_node.x, goal_node.y
       if self.collision_checker.is_node_free(goal_node):
           return goal_node


       offsets = [i * step for i in range(int(max_offset / step) + 1)]


       for dx in offsets:
           for dy in offsets:
               candidates = [
                   (gx + dx, gy + dy),
                   (gx + dx, gy - dy),
                   (gx - dx, gy + dy),
                   (gx - dx, gy - dy),
               ]
               for (nx, ny) in candidates:
                   cand = Node(nx, ny)
                   if self.collision_checker.is_node_free(cand):
                       return cand
       return None


   
   def planning_callback(self):
       if self.robot_position is None or self.robot_orientation is None or self.goal is None:
           self.get_logger().warning("Start/goal not defined yet.")
           return False


       start = Node.from_tf(self.robot_position, self.robot_orientation)
       goal = Node.from_pose(self.goal)


       if not self.is_map_loaded or self.map_ is None:
           self.get_logger().warning("Map not loaded yet.")
           return False


       self.collision_checker = CollisionChecker(
           self.map_, self.robot_radius, self.max_planning_bounds
       )


       if not self.collision_checker.is_node_free(goal):
           adjusted_goal = self._project_goal_to_free_space(goal)
           if adjusted_goal is None:
               self.get_logger().warning(
                   "Goal can't be reached, goal is in collision and no free neighbour."
               )
               self.path_publisher.publish(self.nodes_to_path_msg([]))
               return False
           self.get_logger().info(
               f"Adjusted goal to free space ({adjusted_goal.x:.2f}, {adjusted_goal.y:.2f})"
           )
           goal = adjusted_goal


       if not self.collision_checker.is_node_free(start):
           self.get_logger().warning(
               "Goal can't be reached, start is in collision"
           )
           self.path_publisher.publish(self.nodes_to_path_msg([]))
           return False


       planner = RRTstar()
       planner.set_start(start)
       planner.set_goal(goal)
       planner.set_collision_checker(self.collision_checker)
       planner.set_goal_tolerance(self.goal_tolerance)
       planner.set_expansion_size(self.expansion_size)
       planner.set_step_size(self.step_size)
       planner.set_bias(self.bias)


       self.get_logger().info("Searching solution path with RRT*...")
       solved = planner.solve(self.solve_time)


       if solved: 
           path_length = planner.get_path_length()
           self.get_logger().info(f"Path length: {path_length} meters")


           solution_path = planner.get_solution_path()
           self.get_logger().info("Solution path found")
           if self.use_path_smoother:
               solution_path = self.moving_average(solution_path)
           path_msg = self.nodes_to_path_msg(solution_path)


           rrt = planner.get_tree()
           self.publish_tree(rrt)


           self.current_path_nodes = list(solution_path)


           if self.replan_timer is not None:
               try:
                   self.replan_timer.cancel()
               except Exception:
                   pass
           self.replan_timer = self.create_timer(
               1.5, self.monitor_path_callback
           )


           self.stop_motion_publisher.publish(Bool(data=False))
           self.path_publisher.publish(path_msg)
           return True


       rrt = planner.get_tree()
       self.publish_tree(rrt)


       self.get_logger().warning("No path found")
       self.path_publisher.publish(
           self.nodes_to_path_msg([])
       )
       self.stop_motion_publisher.publish(Bool(data=True))
       return False


  
   def monitor_path_callback(self):
       if self.is_goal_cancelled:
           self.stop_motion_publisher.publish(Bool(data=True))
           if self.replan_timer is not None:
               try:
                   self.replan_timer.cancel()
               except Exception:
                   pass
           return


       if self.is_goal_reached:
           if self.replan_timer is not None:
               try:
                   self.replan_timer.cancel()
               except Exception:
                   pass
           return


       if (
           self.map_ is None
           or self.collision_checker is None
           or not self.current_path_nodes
           or self.robot_position is None
       ):
           return


       self.collision_checker = CollisionChecker(
           self.map_, self.robot_radius, self.max_planning_bounds
       )


       rx, ry = self.robot_position[0], self.robot_position[1]
       closest_idx = 0
       closest_dist = float("inf")


       for idx, node in enumerate(self.current_path_nodes):
           d2 = (node.x - rx) ** 2 + (node.y - ry) ** 2
           if d2 < closest_dist:
               closest_dist = d2
               closest_idx = idx


       if closest_idx > 2:
           self.current_path_nodes = self.current_path_nodes[closest_idx - 2 :]


       end_idx = min(closest_idx + 5, len(self.current_path_nodes))
       segment = self.current_path_nodes[closest_idx:end_idx]
       if len(segment) < 1:
           return


       for node in segment:
           if not self.collision_checker.is_node_free(node):
               self.get_logger().warning(
                   "Node on path now in collision. Stopping and replanning..."
               )
               self.stop_motion_publisher.publish(Bool(data=True))
               self._replan_from_current_pose()
               return


       for i in range(len(segment) - 1):
           n1 = segment[i]
           n2 = segment[i + 1]
           if not self.collision_checker.is_connection_free(n1, n2, self.step_size):
               self.get_logger().warning(
                   "Connection on path now in collision. Stopping and replanning..."
               )
               self.stop_motion_publisher.publish(Bool(data=True))
               self._replan_from_current_pose()
               return


  
   def _replan_from_current_pose(self):
       if (
           self.robot_position is None
           or self.robot_orientation is None
           or self.goal is None
           or self.map_ is None
       ):
           return


       self.collision_checker = CollisionChecker(
           self.map_, self.robot_radius, self.max_planning_bounds
       )


       new_start = Node.from_tf(self.robot_position, self.robot_orientation)
       new_goal = Node.from_pose(self.goal)


       if new_start is None or new_goal is None:
           return


       if not self.collision_checker.is_node_free(new_goal):
           adjusted_goal = self._project_goal_to_free_space(new_goal)
           if adjusted_goal is None:
               self.get_logger().warning(
                   "Replan failed: goal still in obstacle."
               )
               return
           new_goal = adjusted_goal


       if not self.collision_checker.is_node_free(new_start):
           self.get_logger().warning(
               "Replan failed: current robot pose in obstacle."
           )
           return


       planner = RRTstar()
       planner.set_start(new_start)
       planner.set_goal(new_goal)
       planner.set_collision_checker(self.collision_checker)
       planner.set_goal_tolerance(self.goal_tolerance)
       planner.set_expansion_size(self.expansion_size)
       planner.set_step_size(self.step_size)
       planner.set_bias(self.bias)


       self.get_logger().info("Replanning with RRT* from current pose...")
       solved = planner.solve(self.solve_time)


       if not solved:
           self.get_logger().warning("Replan failed: no solution found.")
           return


       new_path = planner.get_solution_path()
       if self.use_path_smoother:
           new_path = self.moving_average(new_path)


       self.current_path_nodes = list(new_path)
       path_msg = self.nodes_to_path_msg(self.current_path_nodes)
       self.path_publisher.publish(path_msg)


       rrt = planner.get_tree()
       self.publish_tree(rrt)


       self.stop_motion_publisher.publish(Bool(data=False))
       self.get_logger().info("New RRT* path sent to controller.")


   
   def moving_average(self, path: list, window: int = 4) -> list:
       window_queue = []
       smoothed_path = [path[0]]


       for node in path:
           if len(window_queue) == window:
               smoothed_path.append(sum(window_queue) / window)  
               window_queue.pop(0)


           window_queue.append(node)
       goal = Node.from_pose(self.goal)
       return smoothed_path + [goal]


   def nodes_to_path_msg(self, path_nodes: list) -> Path:
       path = Path()
       path.header.frame_id = "map"
       path.header.stamp = self.get_clock().now().to_msg()


       for node in path_nodes:
           path.poses.append(node.to_pose_stamped())


       return path


   def publish_tree(self, tree):
       visual_rrt = Marker()
       visual_rrt.header.frame_id = "map"
       visual_rrt.header.stamp = self.get_clock().now().to_msg()
       visual_rrt.action = Marker.ADD


       visual_rrt.type = Marker.LINE_LIST


       visual_rrt.pose.orientation.w = 1.0


       visual_rrt.id = 0


       visual_rrt.scale.x = 0.03


       visual_rrt.color.b = 1.0
       visual_rrt.color.a = 1.0


       for branch in tree:
           node = tree[branch]


           while node.parent:
               p = Point()
               p.x = node.x
               p.y = node.y
               p.z = 0.1


               visual_rrt.points.append(p)


               p = Point()


               p.x = node.parent.x
               p.y = node.parent.y
               p.z = 0.1
               visual_rrt.points.append(p)


               node = node.parent
       self.tree_publisher.publish(visual_rrt)


   def send_goal(self, pose: Pose) -> bool:
       goal = PoseStamped()
       goal.header.frame_id = "map"
       goal.header.stamp = self.get_clock().now().to_msg()
       goal.pose = pose


       return self.goal_query_callback(goal)


   def cancel_goal(self):
       self.stop_motion_publisher.publish(Bool(data=True))
       self.is_goal_cancelled = True




################################################################
# MAIN DEPLOY
def main(args=None):
   rclpy.init(args=args)
   navigator_node = Navigator()
   try:
       rclpy.spin(navigator_node)
   except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
       navigator_node.cancel_goal()
   finally:
       rclpy.try_shutdown()


   navigator_node.destroy_node()
   rclpy.shutdown()




if __name__ == "__main__":
   main()
