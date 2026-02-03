from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # ! PACKAGES DIR
    sampling_planners_demo_dir = FindPackageShare(
        package="sampling_planners_demo"
    ).find("sampling_planners_demo")

    # ! NODES
    navigator_node = Node(
        package="sampling_planners_demo",
        executable="navigator_rrt.py",
        name="navigator_node",
        output="screen",
        parameters=[sampling_planners_demo_dir + "/config/rrt_params.yaml"],
    )

    path_follower_node = Node(
        package="path_follower",
        executable="path_follower",
        name="path_follower_node",
        output="screen",
        # parameters=[{"linear_velocity": 0.1, "angular_velocity": 0.6}],
        condition=IfCondition(LaunchConfiguration("enable_move", default=True)),
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(navigator_node)
    ld.add_action(path_follower_node)

    return ld
