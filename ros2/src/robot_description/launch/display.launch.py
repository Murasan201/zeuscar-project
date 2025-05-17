from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    xacro_file = PathJoinSubstitution([
        robot_description_pkg,
        'urdf',
        'robot.xacro'
    ])

    # xacroコマンドでURDFを生成（文字列として取得）
    robot_description_content = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

