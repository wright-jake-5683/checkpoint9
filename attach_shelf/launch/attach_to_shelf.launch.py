from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    package_description = "attach_shelf"

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value=TextSubstitution(text="0.3")
    )

    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value=TextSubstitution(text="-90.0")
    )

    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value=TextSubstitution(text="false")
    )

    rviz_config_file_name_arg = DeclareLaunchArgument(
        'rviz_config_file_name', default_value='launch_part.rviz')

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')
    rviz_config_file_name = LaunchConfiguration('rviz_config_file_name')

    global_path_to_rviz_file = PathJoinSubstitution([
        FindPackageShare(package_description),
        'rviz',
        rviz_config_file_name
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', global_path_to_rviz_file])

    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        output='screen',
        name='approach_shelf_service_node',
        parameters=[{'use_sim_time': True}]
    )

    lifecycle_node = LifecycleNode(
        package='attach_shelf',
        executable='pre_approach_v2',
        name='pre_approach_node_v2',
        namespace='',
        output='screen',
        arguments=["-obstacle", obstacle_f, 
                   "-degrees", degrees_f,
                   "-final_approach", final_approach_f
        ]
    )

    # Transition: unconfigured -> configuring
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == lifecycle_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    configured_message = LogInfo(
        msg="pre_approach_node_v2 set to state: configured")


    # When node reaches "inactive", activate it
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == lifecycle_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    active_message = LogInfo(
        msg="pre_approach_node_v2 set to state: active")


    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        rviz_config_file_name_arg,
        #rviz_node,
        approach_service_server_node,
        lifecycle_node,
        configure_event,
        configured_message,
        activate_handler,
        active_message
    ])