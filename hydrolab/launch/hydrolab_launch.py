import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.actions import Node

#after what time after starting 
time_start = 20.0

def generate_launch_description():
    #uruchomienie wezlow z pakietu hydrolab
    drone_controller = Node(
        package='hydrolab',
        executable='drone_controller',
        name='drone_controller',
        output='screen'
    )

    pool_tracker = Node(
        package='hydrolab',
        executable='pool_tracker',
        name='pool_tracker',
        output='screen'
    )

    #uruchomienie Gazebo z customowym swiatem "betterpool"
    env = {
        'GZ_SIM_RESOURCE_PATH': '/home/krzychu/PX4-Autopilot/Tools/simulation/gz/worlds',
        'PX4_SYS_AUTOSTART': '4014_gz_x500_mono_cam_down',
        'PX4_GZ_WORLD': 'betterpool',
        'PX4_GZ_MODEL_POSE': '0,0,1',
        'PX4_GZ_MODEL': 'gz_x500_mono_cam_down'
    }

    px4_sitl = ExecuteProcess(
        cmd=['/home/krzychu/PX4-Autopilot/build/px4_sitl_default/bin/px4'],
        output='screen',
        additional_env=env
    )

    #uruchomienie MicroXRCEAgent (ros agent) z wymaganymi argumentami
    micro_xrce_agent = ExecuteProcess(
        cmd=[os.path.expanduser('~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'), 'udp4', '-p', '8888'],
        output='screen'
    )

    #bridge ROS2 <-> Gazebo dla obrazu i informacji z kamery
    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_image_bridge',
        arguments=[
            '/world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            '/world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    #bridge ROS2 <-> Gazebo dla lidara
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/world/betterpool/model/x500_mono_cam_down_0/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )

    ld = LaunchDescription([
        #uruchamiamy sekwencyjnie Gazebo (startuje jako 1)
        px4_sitl,
        #po starcie Gazebo, uruchamiamy MicroXRCEAgent
        RegisterEventHandler(
            OnProcessStart(
                target_action=px4_sitl,
                on_start=[micro_xrce_agent]
            )
        ),
        #micro_xrce_agent, 
        #po uruchomieniu MicroXRCEAgenta, uruchamiamy mostki komunikacyjne (bridges)
        RegisterEventHandler(
            OnProcessStart(
                target_action=micro_xrce_agent,
                on_start=[camera_image_bridge, camera_info_bridge, lidar_bridge]
            )
        ),
        #po uruchomieniu bridge'ow uruchamiamy node'y
        RegisterEventHandler(
            OnProcessStart(
                target_action=lidar_bridge,
                on_start=[TimerAction(
                    period=time_start,
                    actions=[drone_controller, pool_tracker]
                )]
            )
        ),
        #jeszcze raz mmusimy mostki uruchomic bo nie zalapie po 1 razie
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=drone_controller,
        #         on_start=[TimerAction(
        #             period=5.0,
        #             actions=[camera_image_bridge, camera_info_bridge]
        #         )]
        #     )
        # )
        #drone_controller, pool_tracker
    ])

    return ld