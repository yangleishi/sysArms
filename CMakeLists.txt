cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)
# 设置PROJECT_NAME变量
set(PROJECT_NAME sysArms)
# 设置工程名
project (${PROJECT_NAME})

include_directories(include)

# 查找当前目录下的所有源文件并存入DIR_SRCS变量
aux_source_directory(src DIR_SRCS)
# 添加一个可编译的目标到工程
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# use pthread
target_link_libraries(${PROJECT_NAME} pthread)

