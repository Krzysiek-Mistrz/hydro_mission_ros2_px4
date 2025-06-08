import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.actions import Node

#after what time after starting 
time_start = 10.0

def generate_launch_description():
    #uruchomienie wezlow z pakietu hydrolab
    drone_controller = Node(
        package='hydrolab',
        executable='drone_controller',
        name='drone_controller',
        output='screen'
    )

    # pool_tracker = Node(
    #     package='hydrolab',
    #     executable='pool_tracker',
    #     name='pool_tracker',
    #     output='screen'
    # )

    aruco_tracker = Node(
        package='hydrolab',
        executable='aruco_tracker',
        name='aruco_tracker',
        output='screen'
    )

    #uruchomienie MicroXRCEAgent (ros agent) z wymaganymi argumentami
    # micro_xrce_agent = ExecuteProcess(
    #     cmd=[os.path.expanduser('~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'), 'serial', '--dev', '/dev/ttyUSB0', '-b', '57600'],
    #     output='screen'
    # )

    ld = LaunchDescription([
        # micro_xrce_agent,
        drone_controller, aruco_tracker
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=micro_xrce_agent,
        #         on_start=[TimerAction(
        #             period=time_start,
        #             actions=[drone_controller, aruco_tracker]
        #         )]
        #     )
        # ),
    ])

    return ld