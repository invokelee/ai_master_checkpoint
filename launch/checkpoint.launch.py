"""Launch the mission controller and the robot monitor."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the checkpoint launch description."""
    mission_controller = Node(
        package='ai_master_checkpoint',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
    )
    robot_monitor = Node(
        package='ai_master_checkpoint',
        executable='robot_monitor',
        name='robot_monitor',
        output='screen',
    )
    # The mission is finished once the controller exits: stop everything.
    shutdown_on_finish = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_controller,
            on_exit=[EmitEvent(event=Shutdown(reason='Mission finished'))],
        )
    )
    return LaunchDescription([mission_controller, robot_monitor, shutdown_on_finish])
