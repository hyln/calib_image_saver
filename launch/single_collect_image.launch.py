from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calib_image_saver',
            executable='singleImageSaver',
            name='saver',
            output='screen',
            remappings=[
                ('/image_input', '/perception/sensors/hik_camera_30/image'),
            ],
            parameters=[{
                'image_path': '/home/se/test_workspace/calib_image30',
                'image_name': 'IMG_',
                'rate': 9,
                'display_max_width': 1280,
                'display_max_height': 720,
                'min_save_interval': 1.0,
                'board_width': 11,
                'board_height': 8,
                'is_use_OpenCV': True,
                'is_show': True,
            }],
        ),
    ])
