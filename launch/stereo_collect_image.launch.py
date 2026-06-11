from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calib_image_saver',
            executable='stereoImageSaver',
            name='saver',
            output='screen',
            remappings=[
                ('/left_image', '/mv_25001326/image_raw'),
                ('/right_image', '/mv_25001326/image_raw'),
            ],
            parameters=[{
                'image_path': '/home/gao/ws2/devel/lib/camera_model/tmp',
                'image_name_left': 'left_',
                'image_name_right': 'right_',
                'rate': 9,
                'display_max_width': 1280,
                'display_max_height': 720,
                'board_width': 9,
                'board_height': 8,
                'is_use_OpenCV': False,
                'is_show': True,
            }],
        ),
    ])
