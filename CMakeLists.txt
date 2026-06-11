cmake_minimum_required(VERSION 3.8)
project(calib_image_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB APRILTAG_SRC CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/apriltag/*.cc
)

file(GLOB APRILTAG_FRONTEND_SRC CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/apriltag_frontend/*.cpp
)

function(add_calib_image_saver_node target)
  add_executable(${target} ${ARGN})
  target_include_directories(${target}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${Boost_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
  )
  ament_target_dependencies(${target}
    cv_bridge
    message_filters
    rclcpp
    sensor_msgs
  )
  target_link_libraries(${target}
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${target} OpenMP::OpenMP_CXX)
  endif()
endfunction()

add_calib_image_saver_node(stereoImageSaver
  src/stereoImageSaver.cc
  src/chessboard/Chessboard.cc
)

add_calib_image_saver_node(singleImageSaver
  src/singleImageSaver.cc
  src/chessboard/Chessboard.cc
)

add_calib_image_saver_node(singleImageAprilSaver
  src/singleImageAprilSaver.cc
  ${APRILTAG_SRC}
  ${APRILTAG_FRONTEND_SRC}
)

install(TARGETS
  stereoImageSaver
  singleImageSaver
  singleImageAprilSaver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
