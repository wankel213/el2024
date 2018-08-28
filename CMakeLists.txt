cmake_minimum_required(VERSION 2.8.4)
project(el2024)

include_directories(
    ${PROJECT_SOURCE_DIR}
    /home/robin/PR2/SOEM-master/soem
    /home/robin/PR2/SOEM-master/osal
    /home/robin/PR2/SOEM-master/osal/linux
    /home/robin/PR2/SOEM-master/oshw/linux
)

link_directories(
    /home/robin/PR2/SOEM-master/build
)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_definitions(
    #-Wall
    -pthread
)

add_executable(
    myCode
    el2024.cpp
)

target_link_libraries(
    myCode
    pthread
    soem # libsoem.a
)

