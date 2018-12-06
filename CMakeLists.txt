CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
# 指定工程名称
PROJECT(object_lane)
#set(CMAKE_CXX_FLAGS "-std=c++1y")
#add_definitions( "-Wall -ansi -pedantic -g")
# 导入OpenCV库
FIND_PACKAGE( OpenCV REQUIRED )

# 设置源文件
SET( SRC_DIR_PATH src )
FILE(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

# 设置头文件搜索目录
SET( CAFFE_ROOT /home/jiaotong404/caffe-ssd)
SET( CUDA_HOME /usr/local/cuda-8.0 )
INCLUDE_DIRECTORIES( ${CAFFE_ROOT}/include/ ./include/ ${CAFFE_ROOT}/.build_release/src/ ${CUDA_HOME}/include/ )

# 设置动态库链接目录
LINK_DIRECTORIES( ${CAFFE_ROOT}/build/lib/ ${CUDA_HOME}/lib64/ )

# 设置要链接的库文件
SET( LIB_PROJECT caffe glog boost_system protobuf boost_thread gflags )

# 指定执行文件输出目录
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 生成可执行文件，即程序
ADD_EXECUTABLE( object_lane  ${PUBLIC_SRC_FILE} ${SRC_DIR_PATH}/lane.cpp )
#ADD_EXECUTABLE( TAGE ${PUBLIC_SRC_FILE} ${SRC_DIR_PATH}/detect_read_video_mixture.cpp )

# 要链接的动态库, 参数指明库的名字即可，比如libxxxx1.so, 指明是xxxx1即可！！！
TARGET_LINK_LIBRARIES( object_lane ${LIB_PROJECT} ${OpenCV_LIBS} )
#TARGET_LINK_LIBRARIES( TAGE ${LIB_PROJECT} ${OpenCV_LIBS} )



