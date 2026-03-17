cmake_minimum_required(VERSION 3.8)
project(at_dog_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY model xacro launch config
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()