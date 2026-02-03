"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: A* algorithm implementation used for path planning
"""

import heapq
import math
import time
from functools import wraps
import rclpy


def timeit(func):
    """timeit

    This method is used to obtain the time used by a specific method. Has to be used
    as a method decorator. Typically used in this example to obtain the time taken
    by the solve method of the algorithm implementation"""

    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rclpy.logging.get_logger("A* Planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


class AStar:
    """AStar class

    A* algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        """AStar constructor"""

        # initial states of the robot
        self.start = None
        self.goal = None

        # grid searching variables
        self.open_list = []
        self._g = {}
        self.nodes = {}

        # additional variables for path planning
        self.collision_checker = None
        self.tolerance = None
        self.solution_path = None

    # =============================
    # ! SETTERS
    # =============================

    def set_start(self, start):
        """Set start

        Set start position of the robot

        Args:
            start (node.Node): contains the start XY position and orientation of robot
        """

        self.start = start

    def set_goal(self, goal):
        """Set goal

        Set goal of the start to goal query

        Args:
            goal (node.Node): contains the goal XY position and orientation of robot
        """
        self.goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker

        Set collision checker to detect collision free nodes

        Args:
            collision_checker (collision_checker.CollisionChecker): the object used to perform
            collision checking by the algorithm
        """
        self.collision_checker = collision_checker

    def set_goal_tolerance(self, tolerance):
        """Set goal tolerance

        Goal tolerance to check when the query has been solved

        Args:
            tolerance (float): distance from the goal in which the query can be considered
            as solved
        """
        self.tolerance = tolerance

    # =============================
    # ! GETTERS
    # =============================

    def get_solution_path(self):
        """Get solution path

        Obtain stored solution path

        Returns:
            (list(node.Node)): list of nodes containing the solution path
        """
        return self.solution_path

    def get_path_length(self):
        """Get path length

        Obtain the length of the solution path

        Returns:
            (float): length of the solution path in meters
        """
        path_length = 0

        for i in range(0, len(self.solution_path) - 2):
            path_length += math.sqrt(
                math.pow(self.solution_path[i]._x - self.solution_path[i + 1]._x, 2)
                + math.pow(self.solution_path[i]._y - self.solution_path[i + 1]._y, 2)
            )

        return path_length

    # =============================

    @timeit
    def solve(self):
        """Solve

        Function to find path from start to goal using A* algorithm"""

        # ! CHECK IF ALL THE REQUIRED ATTRIBUTES HAVE BEEN SET
        if not self.collision_checker:
            rclpy.logging.get_logger("A* Planner").warning(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.start:
            rclpy.logging.get_logger("A* Planner").warning(
                "Start position has not been set on the planner"
            )
            return False
        if not self.goal:
            rclpy.logging.get_logger("A* Planner").warning(
                "Goal position has not been set on the planner"
            )
            return False

        # ! START SOLVING THE QUERY
        # initialise first nodes, start and goal
        self._g[(self.start._x, self.start._y)] = 0
        self._g[(self.goal._x, self.goal._y)] = math.inf

        # add nodes to priority queue
        heapq.heappush(self.open_list, (0.0, (self.start._x, self.start._y)))
        self.nodes[(self.start._x, self.start._y)] = self.start

        # iterate over the priority queue
        while len(self.open_list) > 0:
            temp_node = heapq.heappop(self.open_list)[1]
            current_node = self.nodes[temp_node]

            # search for neighbors of the node
            for neighbor in current_node.generate_neighbors(
                self.collision_checker.map_resolution
            ):
                # check if neighbor is in collision
                if self.collision_checker.is_node_free(neighbor):
                    if (
                        neighbor.calculate_distance(self.goal) < self.tolerance
                    ):  # Check if goal has been achieved
                        neighbor.parent = current_node
                        # obtain path by backtracking neighbor parents
                        self.solution_path = neighbor.backtrack_path()

                        # check first if solution path contains at least one node
                        if len(self.solution_path) > 0:
                            return True
                        else:
                            return False

                    # if neighbor is new, it is stored with an inf cost value
                    if (neighbor._x, neighbor._y) not in self._g:
                        self._g[(neighbor._x, neighbor._y)] = math.inf

                    # define new cost for node
                    new_cost = current_node._g + neighbor.calculate_distance(
                        current_node
                    )

                    # check if new cost is less than neighbor node cost
                    if new_cost < self._g[(neighbor._x, neighbor._y)]:
                        # if cost is lower assign new parent
                        self._g[(neighbor._x, neighbor._y)] = new_cost
                        neighbor._g = new_cost
                        neighbor._h = neighbor.calculate_distance(self.goal)
                        # different from Dijkstra algorithm, heuristic cost is added
                        neighbor._f = neighbor._g + neighbor._h
                        neighbor.parent = current_node

                        # add the neighbor node to the priority queue
                        heapq.heappush(
                            self.open_list,
                            (
                                neighbor._f,
                                (neighbor._x, neighbor._y),
                            ),
                        )
                        self.nodes[(neighbor._x, neighbor._y)] = neighbor

        # if the whole grid has been searched and no solution path was found
        return False
