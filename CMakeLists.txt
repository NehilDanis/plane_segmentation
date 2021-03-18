# specify the minimum cmake version
cmake_minimum_required(VERSION 3.0.2)

# give your project a name
project(plane_segmentation)

# PCL will be excessively used in this project
find_package(PCL REQUIRED)

# generate an executable
add_executable (${PROJECT_NAME} segmentation.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_directories(${PROJECT_NAME} PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
