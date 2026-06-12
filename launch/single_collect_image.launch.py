from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/perception/sensors/hik_camera_29/image',
            description='Input image topic.',
        ),
        DeclareLaunchArgument(
            'image_path',
            default_value='/home/se/test_workspace/calib_image29',
            description='Directory where images are saved.',
        ),
        DeclareLaunchArgument('image_name', default_value='IMG_'),
        DeclareLaunchArgument('rate', default_value='9'),
        DeclareLaunchArgument('display_max_width', default_value='1280'),
        DeclareLaunchArgument('display_max_height', default_value='720'),
        DeclareLaunchArgument('min_save_interval', default_value='1.0'),
        DeclareLaunchArgument('board_width', default_value='11'),
        DeclareLaunchArgument('board_height', default_value='8'),
        DeclareLaunchArgument('is_use_OpenCV', default_value='true'),
        DeclareLaunchArgument('is_show', default_value='true'),
        Node(
            package='calib_image_saver',
            executable='singleImageSaver',
            name='saver',
            output='screen',
            remappings=[
                ('/image_input', LaunchConfiguration('image_topic')),
            ],
            parameters=[{
                'image_path': LaunchConfiguration('image_path'),
                'image_name': LaunchConfiguration('image_name'),
                'rate': LaunchConfiguration('rate'),
                'display_max_width': LaunchConfiguration('display_max_width'),
                'display_max_height': LaunchConfiguration('display_max_height'),
                'min_save_interval': LaunchConfiguration('min_save_interval'),
                'board_width': LaunchConfiguration('board_width'),
                'board_height': LaunchConfiguration('board_height'),
                'is_use_OpenCV': LaunchConfiguration('is_use_OpenCV'),
                'is_show': LaunchConfiguration('is_show'),
            }],
        ),
    ])
