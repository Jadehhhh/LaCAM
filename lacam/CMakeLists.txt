cmake_minimum_required(VERSION 3.16)
file(GLOB SRCS "./src/*.cpp")
project(lacam)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall -mtune=native -march=native)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE ./include)

# 新增的桥接可执行（不用新的 project）
add_executable(lacam_bridge bridge/bridge.cpp)
target_link_libraries(lacam_bridge PRIVATE lacam)
target_include_directories(lacam_bridge PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(lacam_bridge PRIVATE cxx_std_17)
target_compile_options(lacam_bridge PRIVATE -O3 -Wall -mtune=native -march=native)


