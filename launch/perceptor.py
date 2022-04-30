from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Node to start
    perceptor_node = Node(
        package="perceptor",
        executable="perceptor",
        parameters=[
          {'perception_radius': 1.0},
          {'camera_pitch': 0.0},
          {'point_cloud_period': 1000},
          {'rgb_frame_period': 300}
        ],
        output='both',
        emulate_tty=True,
        arguments=[
                "/usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow",
                "/usr/local/share/ORB_SLAM2/Config/RealSense-D435i-IRD.yaml"
        ]
    )

    return LaunchDescription([perceptor_node])
