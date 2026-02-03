"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: D* Lite algorithm implementation used for path planning
"""

import math
import time
from functools import wraps
import rclpy
from grid_planners_demo.node import Node


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
        rclpy.logging.get_logger("D* Lite").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


# =============================


class DStarLite:
    """DStarLite class

    D* Lite algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        """DStarLite constructor"""

        # initial states of the robot
        self.node_start = None
        self.node_goal = None

        self.s_start = None
        self.s_goal = None

        # grid search variables
        self._g, self.rhs, self._U = {}, {}, {}
        self._km = 0

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
        self.node_start = start

    def set_goal(self, goal):
        """Set goal

        Set goal of the start to goal query

        Args:
            goal (node.Node): contains the goal XY position and orientation of robot
        """
        self.node_goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker

        Set collision checker to detect collision free nodes

        Args:
            collision_checker (collision_checker.CollisionChecker): the object used to perform
            collision checking by the algorithm
        """
        self.collision_checker = collision_checker
        self.s_start = self.collision_checker.coordinates_to_indices(
            self.node_start._x, self.node_start._y
        )
        self.s_goal = self.collision_checker.coordinates_to_indices(
            self.node_goal._x, self.node_goal._y
        )

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

    def get_neighbor(self, s_):
        """Get neighbor

        Generate neighbors from node s_, neighbors generated manually
        instead of using Node function since indices were desired

        Args:
            s_ (tuple(double, double)): tuple containing the XY position of a node

        Returns:
            (list(tuple(double, double))): list containing the nodes that are neighbor
            to s_
        """
        nei_list = set()
        if not (s_[0] + 1) > (self.collision_checker.map_width - 1):
            nei_list.add((s_[0] + 1, s_[1]))
        if not (s_[0] - 1) < 0:
            nei_list.add((s_[0] - 1, s_[1]))

        if not (s_[1] + 1) > (self.collision_checker.map_height - 1):
            nei_list.add((s_[0], s_[1] + 1))
        if not (s_[1] - 1) < 0:
            nei_list.add((s_[0], s_[1] - 1))

        if not (s_[0] + 1) > (self.collision_checker.map_width - 1) and not (
            s_[1] + 1
        ) > (self.collision_checker.map_height - 1):
            nei_list.add((s_[0] + 1, s_[1] + 1))

        if not (s_[0] - 1) < 0 and not (s_[1] + 1) > (
            self.collision_checker.map_height - 1
        ):
            nei_list.add((s_[0] - 1, s_[1] + 1))

        if (
            not (s_[0] + 1) > (self.collision_checker.map_width - 1)
            and not (s_[1] - 1) < 0
        ):
            nei_list.add((s_[0] + 1, s_[1] - 1))

        if not (s_[0] - 1) < 0 and not (s_[1] - 1) < 0:
            nei_list.add((s_[0] - 1, s_[1] - 1))

        return nei_list

    def extract_path(self):
        """Extract path

        Extract the path based on the parent set and store it in self.solution_path
        """

        path = [self.s_start]
        _s = self.s_start

        for k in range(1000):
            g_list = {}
            for nei in self.get_neighbor(_s):
                _x, _y = self.collision_checker.indices_to_coordinates(nei[0], nei[1])
                node = Node(_x, _y)
                if self.collision_checker.is_node_free(node):
                    g_list[nei] = self._g[nei]
            _s = min(g_list, key=g_list.get)
            path.append(_s)
            if _s == self.s_goal:
                break

        self.solution_path = []

        for i in path:
            _x, _y = self.collision_checker.indices_to_coordinates(i[0], i[1])
            node = Node(_x, _y)
            self.solution_path.append(node)

        if len(self.solution_path) > 2:
            self.solution_path = self.solution_path[2:]

    # ==================================================
    # ! D* LITE COMPLEMENTARY FUNCTIONS FOR SOLVING QUERY
    # ==================================================
    def compute_path(self) -> list:
        """Compute path

        By using top_key, calculate_key, get_neighbor and update_vertex, searched through the
        whole grid for the solution.
        """
        while True:
            try:
                _s, _v = self.top_key()
            except:
                return False

            if (
                _v >= self.calculate_key(self.s_start)
                and self.rhs[self.s_start] == self._g[self.s_start]
            ):
                return True

            k_old = _v
            self._U.pop(_s)

            if k_old < self.calculate_key(_s):
                self._U[_s] = self.calculate_key(_s)
            elif self._g[_s] > self.rhs[_s]:
                self._g[_s] = self.rhs[_s]
                for nei in self.get_neighbor(_s):
                    self.update_vertex(nei)
            else:
                self._g[_s] = float("inf")
                self.update_vertex(_s)
                for nei in self.get_neighbor(_s):
                    self.update_vertex(nei)

    def update_vertex(self, _s):
        """Update vertex

        Updates a specific node and its neighbors

        Args:
            _s (tuple(float, float)): contains the XY position of a node
        """
        if _s != self.s_goal:
            self.rhs[_s] = float("inf")

            for nei in self.get_neighbor(_s):
                _x, _y = self.collision_checker.indices_to_coordinates(nei[0], nei[1])
                node = Node(_x, _y)
                if self.collision_checker.is_node_free(node):
                    self.rhs[_s] = min(
                        self.rhs[_s],
                        self._g[nei] + self.cost(_s, nei),
                    )
                else:
                    self.rhs[_s] = min(
                        self.rhs[_s],
                        self._g[nei] + float("inf"),
                    )

        if _s in self._U:
            self._U.pop(_s)

        if self._g[_s] != self.rhs[_s]:
            self._U[_s] = self.calculate_key(_s)

    def calculate_key(self, _s):
        """Calculate key

        Calculates key for specific node"""
        return [
            min(self._g[_s], self.rhs[_s]) + self.cost(self.s_start, _s) + self._km,
            min(self._g[_s], self.rhs[_s]),
        ]

    def top_key(self):
        """Top key

        returns the min key and its value from self._U
        """
        s = min(self._U, key=self._U.get)
        return s, self._U[s]

    def cost(self, s_start, s_goal):
        """Cost

        Calculate Cost for motion from s_start to s_goal

        Args:
            s_start (tuple(float, float)): starting node
            s_goal ((float, float)): end node

        Returns:
            (float): cost of the motion
        """

        _x0, _y0 = self.collision_checker.indices_to_coordinates(s_start[0], s_start[1])
        _x1, _y1 = self.collision_checker.indices_to_coordinates(s_goal[0], s_goal[1])
        return math.hypot(_x1 - _x0, _y1 - _y0)

    # ==========================================================

    @timeit
    def solve(self):
        """Solve

        Function to find path from start to goal using D* lite algorithm"""

        # ! CHECK IF ALL THE REQUIRED ATTRIBUTES HAVE BEEN SET
        if not self.collision_checker:
            rclpy.logging.get_logger("D* lite planner").warning(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.s_start:
            rclpy.logging.get_logger("D* lite planner").warning(
                "Start position has not been set on the planner"
            )
            return False
        if not self.s_goal:
            rclpy.logging.get_logger("D* lite planner").warning(
                "Goal position has not been set on the planner"
            )
            return False

        # ! START SOLVING THE QUERY

        # initialize the values in rhs and _g
        for i in range(0, self.collision_checker.map_width - 1):
            for j in range(0, self.collision_checker.map_height - 1):
                self.rhs[(i, j)] = float("inf")
                self._g[(i, j)] = float("inf")

        # define initial costs for goal
        self.rhs[self.s_goal] = 0.0
        self._U[self.s_goal] = self.calculate_key(self.s_goal)

        # flag to define if path has been found
        path_found = False

        # start seaching the grid
        try:
            path_found = self.compute_path()
        except:
            return False

        # path has been found and extract it
        if path_found:
            self.extract_path()
            return True
        return False

    @timeit
    def update_path(self, collision_checker, start, nodes):
        """Update path

        Function to update path from start to goal using D* lite algorithm when nodes
        corresponds to states that are in collision

        Args:
            collision_checker (collision_checker.CollisionChecker): updated object to to perform
            collision checking
            start (node.Node): node with the start position of the robot
            nodes (node.Node): list of nodes that are currently under collision

        Returns:
            (bool): True if a new path has been found, otherwise False
        """

        # redefine start and collision checker
        self.node_start = start
        self.collision_checker = collision_checker

        # check collision with start
        self.s_start = self.collision_checker.coordinates_to_indices(
            self.node_start._x, self.node_start._y
        )

        s_last = self.s_start
        s_curr = self.s_start
        path = [self.s_start]

        # asign nodes in collision
        occ_nodes = nodes

        while s_curr != self.s_goal:

            s_list = {}

            # update nodes from the start
            for s in self.get_neighbor(s_curr):
                x, y = self.collision_checker.indices_to_coordinates(s[0], s[1])
                node = Node(x, y)

                # check if node is in collision
                if self.collision_checker.is_node_free(node):
                    s_list[s] = self._g[s] + self.cost(s_curr, s)
                else:
                    s_list[s] = self._g[s] + float("inf")
            s_curr = min(s_list, key=s_list.get)
            path.append(s_curr)

            # iterate and update nodes in collision
            self._km += self.cost(s_last, s_curr)
            s_last = s_curr

            for s in occ_nodes:
                self.update_vertex(s)
                for nei in self.get_neighbor(s):
                    self.update_vertex(nei)

            # attempt to find new path after updating nodes
            if not self.compute_path():
                return False

        # attempt to extract path
        self.extract_path()
        if len(self.solution_path) > 0:
            return True
        return False
