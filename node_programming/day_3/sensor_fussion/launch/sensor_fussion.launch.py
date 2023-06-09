from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container_name = LaunchConfiguration('container_name')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='rossdl_test_container',
        description='Name of the container')

    container_cmd = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='sensor_fussion',
                plugin='sensor_fussion::CameraDriver'),
            ComposableNode(
                package='sensor_fussion',
                plugin='sensor_fussion::LaserDriver',),
            ComposableNode(
                package='sensor_fussion',
                plugin='sensor_fussion::FussionNode'),
        ])

    startup_cmd = Node(
        name='startup_script',
        package='sensor_fussion',
        executable='startup_script',
        output='both',
    )

    ld = LaunchDescription()

    ld.add_action(declare_container_name_cmd)
    ld.add_action(container_cmd)
    ld.add_action(load_composable_nodes)
    ld.add_action(startup_cmd)

    return ld
