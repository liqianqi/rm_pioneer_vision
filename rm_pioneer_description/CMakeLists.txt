cmake_minimum_required(VERSION 3.8)
project(pioneer_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY 
        launch 
        meshes 
        urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
