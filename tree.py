from node import Node
import math


class Tree:
    def __init__(self):
        """self.nodes contains the tree in following manner:
        {(x, y): Node, (x, y): Node, (x, y): Node}"""
        self.nodes = {}

    def set_start(self, _n: Node):
        "Set the first node, the root of the tree"
        coords = (_n.x, _n.y)
        self.nodes[coords] = _n

    def get_tree(self):
        "Get the whole dictionary with the tree information"
        return self.nodes

    def get_branch(self, _n):
        """Obtain nodes of branch in tree"""
        branch = [_n]
        cur_node = _n
        while cur_node.parent:
            cur_node = self.nodes[(cur_node.parent.x, cur_node.parent.y)]
            branch.append(cur_node)
        branch.reverse()
        return branch

    def is_node_in_tree(self, _n):
        """Check if an specific node is in the tree."""
        coords = (_n.x, _n.y)
        if coords in self.nodes.keys():
            return True
        return False

    def add_node(self, _n: Node):
        """Add a new node to the tree."""
        coords = (_n.x, _n.y)
        self.nodes[coords] = _n

    def neighbor_nodes(self, _n: Node, expansion_size):
        """Returns nodes inside the range of expansion size"""
        nei_nodes = []
        for node in self.nodes:
            if _n.calculate_distance(self.nodes[node]) <= expansion_size + 0.000000001:
                nei_nodes.append(self.nodes[node])
        return nei_nodes

    def nearest(self, _n: Node):
        """Return the nearest node from _n"""
        nearest_node = None
        dis = float("inf")

        for node in self.nodes:
            cur_dis = _n.calculate_distance(self.nodes[node])
            if cur_dis < dis:
                dis = cur_dis
                nearest_node = self.nodes[node]
        return nearest_node

    def step(self, nnear: Node, nrand: Node, expansion_size):
        """Defines the node step from nnear to nrand"""
        if nnear.calculate_distance(nrand) > expansion_size:
            theta = math.atan2(nrand.y - nnear.y, nrand.x - nnear.x)
            (_x, _y) = (
                nnear.x + expansion_size * math.cos(theta),
                nnear.y + expansion_size * math.sin(theta),
            )
            step_node = Node(_x, _y)
            return step_node
        return nrand

    def update_parent(self, node, _n):
        """Update the parent of an specific node in the tree."""
        node_coord = (node.x, node.y)
        self.nodes[node_coord].parent = _n

    def cost(self, _n: Node):
        """Returns the cost of a node to the start node"""
        if not _n.parent:
            return 0
        cost = 0
        cur_node = _n
        next_node = self.nodes[(cur_node.parent.x, cur_node.parent.y)]
        cost += cur_node.calculate_distance(next_node)
        while next_node.parent:
            cur_node = next_node
            next_node = self.nodes[(cur_node.parent.x, cur_node.parent.y)]
            cost += cur_node.calculate_distance(next_node)
        return cost
