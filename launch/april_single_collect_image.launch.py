from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calib_image_saver',
            executable='singleImageAprilSaver',
            name='saver',
            output='screen',
            remappings=[
                ('/image_input', '/snappy_cam/stereo_l'),
            ],
            parameters=[{
                'image_path': '/home/gao/bag/april/outdoor_forward_calib_snapdragon_cam/l',
                'image_name': 'IMG_',
                'rate': 30,
                'display_max_width': 1280,
                'display_max_height': 720,
                'board_width': 5,
                'board_height': 4,
                'is_use_OpenCV': True,
                'is_show': True,
            }],
        ),
    ])
