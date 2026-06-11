# calib_image_saver

A small tool to collect images for calibration. The program will detect the chessboard in the image and save the image with chessboard detected.

## Download code  

Enter your ROS 2 workspace:

```
cd YOUR_PATH/ros2_ws/src
  
git clone https://github.com/gaowenliang/calib_image_saver.git
```
## Install

```
cd YOUR_PATH/ros2_ws/

colcon build --packages-select calib_image_saver
```

## Run

Modify the launch file:

* /image_input: The image topic name, such as "/pg_17221069/image_raw"
* image_path: The path to save images, such as "/home/ubuntu/images". You need to make sure the path exist.
* board_width: Chessboard point size.
* board_height: Chessboard point size.

```
ros2 launch calib_image_saver single_collect_image.launch.py
```

Make sure the detected chessboard points (yellow) fully fill the image as dense as possible.

<img src="doc/Distributed.jpg">
