# 在add_executable之前添加以下行
add_library(run_imu src/yesense_main.c src/analysis_data.c)
# target_include_directories(math_lib PUBLIC include)
target_include_directories(run_imu PUBLIC include)
target_link_libraries(run_imu ${catkin_LIBRARIES})
