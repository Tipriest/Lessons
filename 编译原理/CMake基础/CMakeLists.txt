cmake_minimum_required(VERSION 3.10)
project(main)
aux_source_directory(./src SRC_LIST)        

# 生成动态库
# 将 Solution.cpp 编译为动态库 libSolution.so
add_library(Solution SHARED src/Solution.cpp)

# 添加可执行文件 main
add_executable(main src/main.cpp)

# 链接静态库到主程序
target_link_libraries(main PRIVATE Solution)

# 包含头文件目录
# 添加 include 文件夹到头文件搜索路径
target_include_directories(main PRIVATE include)