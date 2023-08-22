cmake_minimum_required(VERSION 3.22)
project(v4l2_driver)

set(CMAKE_CXX_STANDARD 14)
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/" CACHE PATH "Project Include Directory")

find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)

include_directories(
        include
        ${PROJECT_INCLUDE_DIR}
)
add_library(${PROJECT_NAME} SHARED src/v4l2_driver.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} glog::glog)


add_executable (camera_example test/camera_example.cpp)
add_dependencies(camera_example ${PROJECT_NAME})
target_link_libraries (camera_example  ${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} camera_example)
