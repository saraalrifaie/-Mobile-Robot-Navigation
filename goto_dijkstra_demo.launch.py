"""
Created on August 26, 2025

@author: Juan D. Hernandez Vega (HernandezVegaJ@cardiff.ac.uk)
@author: Steven Silva (SilvaS1@cardiff.ac.uk)

Purpose: Launches all of the needed nodes to make the robot navigate using the Dijkstra algorithm
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    """Constains all of the parameters and nodes to be launched"""

    # ===========================================
    # ! PACKAGES DIRECTORIES
    # ===========================================
    grid_planners_demo_dir = FindPackageShare(package="grid_planners_demo").find(
        "grid_planners_demo"
    )

    # ===========================================
    # ! NODES
    # ===========================================
    navigator_node = Node(
        package="grid_planners_demo",
        executable="navigator_dijkstra",
        name="navigator_node",
        output="screen",
        parameters=[grid_planners_demo_dir + "/config/dijkstra_params.yaml"],
    )

    path_follower_node = Node(
        package="path_follower",
        executable="path_follower",
        name="path_follower_node",
        output="screen",
        # parameters=[{"linear_velocity": 0.1, "angular_velocity": 0.6}],
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(navigator_node)
    ld.add_action(path_follower_node)

    return ld
