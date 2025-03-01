# 在add_executable之前添加以下行
add_library(run_soem src/soem_dog.c)
# target_include_directories(math_lib PUBLIC include)
target_include_directories(run_soem PUBLIC include)

target_link_libraries(run_soem 
${catkin_LIBRARIES}
soem
)
