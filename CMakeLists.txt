cmake_minimum_required(VERSION 3.20)
project(lightbar_detection)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})