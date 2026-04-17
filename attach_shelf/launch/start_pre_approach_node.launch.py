from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value=TextSubstitution(text="0.3")
    )

    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value=TextSubstitution(text="-90.0")
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')


    lifecycle_node = LifecycleNode(
        package='attach_shelf',
        executable='pre_approach',
        name='pre_approach_node',
        namespace='',
        output='screen',
        arguments=["-obstacle", obstacle_f, 
                   "-degrees", degrees_f
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
        msg="pre_approach_node set to state: configured")


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
        msg="pre_approach_node set to state: active")


    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        lifecycle_node,
        configure_event,
        configured_message,
        activate_handler,
        active_message
    ])