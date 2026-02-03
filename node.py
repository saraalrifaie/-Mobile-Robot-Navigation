"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Implementation of Node class to be used for path planning algorithms
"""

from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.time import Time


class Node:
    """Node class

    Used to handle the implementation of some of the algorithms in order to handle the visited nodes
    in a grid and different operations
    """

    def __init__(
        self, _x: float = 0.0, _y: float = 0.0, theta: float = 0.0, parent=None
    ):
        """Node constructor

        Args:
            _x (float, optional): X position of node. Defaults to 0.0.
            _y (float, optional): Y position of node. Defaults to 0.0.
            theta (float, optional): Yaw orientation of node. Defaults to 0.0.
            parent (node.Node, optional): parent node from this node. Defaults to None.
        """
        self.parent = parent

        self._x = _x
        self._y = _y
        self.theta = theta

        # attributes mainly used for search-based path planning algorithms
        self._g = 0  # From current node to start node
        self._h = 0  # From current node to end node
        self._f = 0  # Total cost

        self.pixel_tolerance = 1

    def __eq__(self, other) -> bool:
        return self.calculate_distance(other) < 0.1

    def __add__(self, other):
        return Node(self._x + other._x, self._y + other._y)

    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)

    def __truediv__(self, other):
        return Node(self._x / other, self._y / other)

    def calculate_distance(self, end) -> float:
        """Calculate distance

        Euclidean distance between two nodes

        d = sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2))


        Args:
            end (node.Node): node to which distance will be calculated

        Returns:
            float: distance from this node to end node
        """
        return ((self._x - end._x) ** 2 + (self._y - end._y) ** 2) ** 0.5

    def generate_neighbors(self, map_resolution: float) -> list:
        """Generate neighbors

        Obtain the neighbors as nodes

        Args:
            map_resolution (float): resolution of the grid map used

        Returns:
            neighbors (list(node.Nodes)): list of nodes that are neighbors from this node
        """
        neighbors = []
        step = map_resolution
        moves = [
            (0, -step),
            (0, step),
            (-step, 0),
            (step, 0),
            (-step, -step),
            (-step, step),
            (step, -step),
            (step, step),
        ]

        for move in moves:
            neighbors.append(Node(_x=self._x + move[0], _y=self._y + move[1]))

        return neighbors

    def backtrack_path(self) -> list:
        """Backtrack path

        Obtain the path backtracking from the current nodes to all the subsequent parents

        Returns:
            (list(node.Node)): list of nodes backtracked parent by parent from this node
        """
        path = []
        current_node = self

        while current_node.parent:
            path.append(current_node)
            current_node = current_node.parent

        return (path + [current_node])[::-1]

    @staticmethod
    def from_pose(pose: Pose):
        """From pose

        Define Node object from a Pose

        Args:
            pose (geometry_msgs.msg.Pose): pose of the node

        Returns:
            new_state (node.Node): node object created from pose
        """
        new_state = Node()
        new_state._x = pose.position.x
        new_state._y = pose.position.y

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        new_state.theta = yaw

        return new_state

    @staticmethod
    def from_tf(position: list, quaternion: list):
        """Define Node object from position and quaternion

        Args:
            position (list(double, double)): list with the positions XY of the node
            quaternion (list(double, double, double, double)): list with the orientation XYZW in
            quaternion notation

        Returns:
            new_state (node.Node): node created from position and quaternion
        """
        new_state = Node()
        new_state._x = position[0]
        new_state._y = position[1]

        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        new_state.theta = yaw

        return new_state

    def to_pose_stamped(self):
        """Get Pose object from current Node object position

        Returns:
            pose (geometry_msgs.msg.PoseStamped): PoseStamped message with the current node
            information.
            Orientation is not considered.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = Time().to_msg()

        pose.pose.position.x = self._x
        pose.pose.position.y = self._y
        pose.pose.orientation.w = 1.0

        return pose
