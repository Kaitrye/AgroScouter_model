cmake_minimum_required(VERSION 3.16)
project(robot_track_animation LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)

add_library(robot_track_animation SHARED TrackAnimation.cc)
target_compile_features(robot_track_animation PRIVATE cxx_std_17)
target_link_libraries(robot_track_animation PRIVATE gz-sim8::gz-sim8)
set_target_properties(robot_track_animation PROPERTIES PREFIX "" OUTPUT_NAME "robot_track_animation")
