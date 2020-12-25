project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

file(GLOB HEADERS src/*.h)
file(GLOB HEADERS_HPP src/*.hpp)

set(sources src/pid.cpp src/main.cpp src/robot.cc src/twiddle.cc ${HEADERS} ${HEADERS_HPP})

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)
