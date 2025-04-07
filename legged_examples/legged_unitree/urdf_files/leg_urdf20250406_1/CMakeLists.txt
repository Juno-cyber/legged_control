cmake_minimum_required(VERSION 3.0.2)

project(leg_urdf20250406_1)

find_package(catkin REQUIRED)

find_package(roslaunch)

catkin_package()

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
