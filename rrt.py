import math
import time
from functools import wraps
import random
import rclpy
from collision_checker import CollisionChecker
from node import Node
from nav_msgs.msg import Path
from rclpy.time import Time
from tree import Tree


def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rclpy.logging.get_logger("RRT planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


class RRT:
    """RRT algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        self.start: Node = None
        self.goal: Node = None

        self.nodes = {}

        self.step_size = 0.001
        self.expansion_size = 0.5
        self.bias = 0.5

        self.collision_checker: CollisionChecker = None
        self.tolerance = None

        self.solution_node = None
        self.solution_path = None

        self.tree = Tree()

    def set_start(self, start):
        """Set start position of the robot"""
        self.start = start

    def set_goal(self, goal):
        """Set goal of the start to goal query"""
        self.goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker to detect collision free nodes"""
        self.collision_checker = collision_checker

    def set_goal_tolerance(self, tolerance):
        """Set goal tolerance to check when the query has been solved"""
        self.tolerance = tolerance

    def set_step_size(self, step_size):
        """Set the step size to determine if connection between two
        nodes is possible with collision checking"""
        self.step_size = step_size

    def set_expansion_size(self, expansion_size):
        """Set the maximum expansion for a new node in the tree"""
        self.expansion_size = expansion_size

    def set_bias(self, bias):
        """Set the bias of the planner"""
        self.bias = bias

    def get_solution_path(self):
        """Obtain stored solution path"""
        return self.solution_path

    def get_path_length(self):
        """Obtain the length of the solution path"""
        distance = 0

        for i in range(0, len(self.solution_path) - 2):
            distance += math.sqrt(
                math.pow(self.solution_path[i].x - self.solution_path[i + 1].x, 2)
                + math.pow(self.solution_path[i].y - self.solution_path[i + 1].y, 2)
            )
        return distance

    def get_tree(self):
        """Obtain the tree"""
        tree = self.tree.get_tree()
        return tree

    @timeit
    def solve(self, termination_time) -> list:
        """Function to find path from start to goal using Dijkstra algorithm"""

        if not self.collision_checker:
            rclpy.logging.get_logger("RRT planner").warning(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.start:
            rclpy.logging.get_logger("RRT planner").warning(
                "Start position has not been set on the planner"
            )
            return False
        if not self.goal:
            rclpy.logging.get_logger("RRT planner").warning(
                "Goal position has not been set on the planner"
            )
            return False

        self.tree.add_node(self.start)

        timeout = time.time() + termination_time

        while timeout > time.time():
            temp_val = random.uniform(0, 1)
            if 0 < temp_val < self.bias:
                nrand = self.goal
            else:
                nrand = self.sample()

            nnear = self.tree.nearest(nrand)
            new_node = self.tree.step(nnear, nrand, self.expansion_size)

            if self.collision_checker.is_node_free(new_node):
                if self.collision_checker.is_connection_free(
                    new_node, nnear, self.step_size
                ) and not self.tree.is_node_in_tree(new_node):
                    new_node.parent = nnear
                    self.tree.add_node(new_node)

                if (
                    new_node.calculate_distance(self.goal) < self.tolerance
                    and self.solution_node is None
                ):
                    self.solution_node = new_node
                    if self.solution_node:
                        self.solution_path = self.tree.get_branch(self.solution_node)
                        return True

        return False

    def sample(self):
        """Sample node from the workspace"""
        _i = int(random.uniform(0, self.collision_checker.map_width - 1))
        _j = int(random.uniform(0, self.collision_checker.map_height - 1))
        _x, _y = self.collision_checker.indices_to_coordinates(_i, _j)
        node = Node(_x, _y)
        return node
