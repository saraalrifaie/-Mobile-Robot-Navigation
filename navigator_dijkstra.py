"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Implementation of navigator class that uses the Dijkstra algorithm to solve
a path planning problem
"""

import time
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, Buffer, TransformException
from grid_planners_demo.collision_checker import CollisionChecker
from grid_planners_demo.node import Node
from grid_planners_demo.planners.dijkstra import Dijkstra


class Navigator(rclpy.node.Node):
    """Navigator class

    Manages the incoming start-to-goal query, solves it by requesting a map and sends the path to the controller to follow
    """

    def __init__(self):
        """Navigator constructor"""
        super().__init__("navigator_node")

        # ===============================================================================
        # ! Initial values
        # ===============================================================================

        # parameters

        self.goal_tolerance = (
            self.declare_parameter("goal_tolerance", 0.1)
            .get_parameter_value()
            .double_value
        )  # tolerance of the solution path to the goal

        self.use_path_smoother = (
            self.declare_parameter("use_path_smoother", False)
            .get_parameter_value()
            .bool_value
        )

        self.map_topic = (
            self.declare_parameter("map_topic", "/map")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("max_planning_bounds", [20, 20])
        self.max_planning_bounds = self.get_parameter("max_planning_bounds").value

        # stores map information
        self.map = None

        # object to perform collision checking
        self.collision_checker = None

        #! Information about the robot
        self.robot_position = None
        self.robot_orientation = None
        self.robot_radius = (
            self.declare_parameter("robot_radius", 0.15)
            .get_parameter_value()
            .double_value
        )

        #! Start and goal nodes for the query
        self.start = None
        self.goal = None

        #! Flags to check the status of the robot, the query and the map
        self.is_goal_cancelled = False
        self.is_goal_reached = False
        self.is_map_loaded = False
        self.is_robot_moving = False

        # ! transform objects to listen to tfs
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        time.sleep(1)

        # =======================================
        #! SUBSCRIBERS
        # =======================================
        # A TFMessage message, not really used, just defined to obtain
        #  the transforms in the callback
        self.start_sub = self.create_subscription(
            TFMessage, "/tf", self.robot_pose_callback, 10
        )

        # A PoseStamped message to get the goal pose
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_query_callback, 10
        )

        # ? subscribers to communicate with the controller
        # A Bool message to get know if the controller is sending velocities
        # to the robot
        self.robot_moving_sub = self.create_subscription(
            Bool, "/robot_is_moving", self.robot_moving_callback, 10
        )

        # A Bool message to know when the robot has reached the goal
        self.goal_reached_sub = self.create_subscription(
            Bool, "/goal_reached", self.goal_reached_callback, 10
        )

        # A OccupancyGrid message including the information of occupied areas
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )

        # =======================================
        #! PUBLISHERS
        # =======================================
        # ? publishers to communicate with the controller
        # publishes path for controller
        self.path_pub = self.create_publisher(Path, "/path", 10)
        # publishes a Bool to tell the controller if the motion should be stopped
        self.stop_motion_pub = self.create_publisher(Bool, "/stop_motion", 10)
        # publishes the goal for the controller
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_controller", 10)

    # =======================================
    #! CALLBACKS
    # =======================================
    def goal_reached_callback(self, goal_reached: Bool):
        """Goal reached callback

        Listens from the controller whether the robot has reached the goal

        Args:
            goal_reached (std_msgs.msg.Bool): will contain True if the robot reached the goal,
            otherwise False
        """
        self.is_goal_reached = goal_reached.data

    def robot_moving_callback(self, robot_moving: Bool):
        """Robot moving callback

        Listens from the controller wether the robot is moving or not

        Args:
            robot_moving (std_msgs.msg.Bool):  will contain True if the robot is being moved by
            the controller, otherwise False
        """
        self.is_robot_moving = robot_moving.data

    def robot_pose_callback(self, tf_msg: TFMessage):
        """Robot pose callback

        Listen to robot current position"""

        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            position = transform.transform.translation
            quaternion = transform.transform.rotation
        except TransformException as e:
            return

        # save position and orientation
        self.robot_position = [
            position.x,
            position.y,
            position.z,
        ]
        self.robot_orientation = [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]

    def goal_query_callback(self, data: PoseStamped) -> bool:
        """Goal query callback

        Obtain goal query and ask for the planning callback

        Args:
            pose_msg (geometry_msgs.msg.PoseStamped): message with the information of the
            requested planning query

        Returns:
            (bool): return True if a solution path has been found and sent to controller,
            otherwise False
        """

        # tell controller to stop the robot
        if self.is_robot_moving:
            self.cancel_goal()

        # set some flags and variables
        self.goal = data.pose
        self.is_goal_cancelled = False
        self.is_goal_reached = False

        self.get_logger().info("Received new goal")
        self.goal_pub.publish(data)  # communicate the goal to controller
        return self.planning_callback()  # start running path planning routine

    def map_callback(self, map_msg: OccupancyGrid):
        """Obtain map used for the planning callback

        Args:
            map_msg (nav_msgs.msg.OccupancyGrid): message with the information
            of the used map for planning
        """
        self.map = map_msg
        self.is_map_loaded = True

    def planning_callback(self):
        """Planning callback

        Main planning function where the whole process is carried,
        definition of the start and goal, map and solving of the query"""

        #! 1. Define start and goal states as nodes
        start = Node.from_tf(self.robot_position, self.robot_orientation)
        goal = Node.from_pose(self.goal)

        #! 2. Check that map is already obtained, otherwise, stop the process
        if not self.is_map_loaded:
            return False

        #! 3. Define Collision Checker
        collision_checker = CollisionChecker(
            self.map, self.robot_radius, self.max_planning_bounds
        )

        #! 4. Check if Start and Goal states are valid

        if (
            not start
            or not goal
            or not collision_checker
            or not collision_checker.is_node_free(goal)
            or not collision_checker.is_node_free(start)
        ):
            self.get_logger().warning(
                "Goal can't be reached, either start or goal are in collision"
            )
            # Clearing path from RViz for visualization purposes
            self.path_pub.publish(self.nodes_to_path_msg([]))
            return False

        #! 5. Define planner

        planner = Dijkstra()

        planner.set_start(start)  # define start robot position
        planner.set_goal(goal)  # define goal
        planner.set_collision_checker(collision_checker)  # set collision node checker
        # set tolerance to find the goal
        planner.set_goal_tolerance(self.goal_tolerance)

        #! 6. Solve the path planning query

        self.get_logger().info("Searching solution path...")
        solved = planner.solve()  # solve the query

        if solved:  # if the query is solved
            path_length = planner.get_path_length()  # show path length
            self.get_logger().info(f"Path length: {path_length} meters")

            solution_path = planner.get_solution_path()  # obtain solution path
            self.get_logger().info("Solution path found")
            if self.use_path_smoother:  # smooth the path if option enabled
                solution_path = self.smooth_path(solution_path)
            # turn the solution path into a ROS2 message
            path_msg = self.nodes_to_path_msg(solution_path)

            #! 7. Send path to controller
            self.path_pub.publish(path_msg)

            return True

        self.get_logger().warning("No path found")
        # Clearing path from RViz for visualization purposes
        self.path_pub.publish(self.nodes_to_path_msg([]))
        return False

    def smooth_path(self, path: list, window: int = 4) -> list:
        """Smooth path

        Smoothes the path obtained by finding an average

        Args:
            path (list(node.Node)): list with nodes that form a solution path
            window (int, optional): number of nodes that will be taken each step for the
            smoothing. Defaults to 4.

        Returns:
            list(node.Node): list of nodes with smoothed path
        """

        window_queue = []
        smoothed_path = [path[0]]

        for node in path:
            if len(window_queue) == window:
                smoothed_path.append(sum(window_queue) / window)  # Mean
                window_queue.pop(0)

            window_queue.append(node)
        goal = Node.from_pose(self.goal)
        return smoothed_path + [goal]

    def nodes_to_path_msg(self, path_nodes: list) -> Path:
        """Nodes to path message

        Transforms the list of path nodes into a Path type of object

        Args:
            path_nodes (list(node.Node)): list with nodes that form a path

        Returns:
            path (nav_msgs.msg.Path): returns a ROS2 message that can be published
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        return path

    def cancel_goal(self):
        """Cancel goal

        Send a cancel to the path follower
        """
        self.stop_motion_pub.publish(Bool(data=True))
        self.is_goal_cancelled = True


def main(args=None):
    rclpy.init(args=args)
    navigator_node = Navigator()
    try:
        rclpy.spin(navigator_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        navigator_node.cancel_goal()
    finally:
        rclpy.try_shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
