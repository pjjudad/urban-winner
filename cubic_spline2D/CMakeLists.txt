cmake_minimum_required(VERSION 3.0.2)  
project(cubic_spline2D)  

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin components  
find_package(catkin REQUIRED COMPONENTS  
  roscpp  
  std_msgs  
  geometry_msgs
  fsd_common_msgs
  fsd_tools
  pcl_conversions
  pcl_ros
  sensor_msgs
)  

# Find external libraries  
find_package(yaml-cpp REQUIRED)  
find_package(Eigen3 REQUIRED)  
include_directories(${EIGEN3_INCLUDE_DIR})  

# 查找PCL库
find_package(PCL 1.8 REQUIRED COMPONENTS common io)

# For Python3 (not directly used in C++ but for completeness)  
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)  
#find_package(Python3 COMPONENTS NumPy)  

# Interface library for matplotlib_cpp (假设这是一个虚拟库，用于管理依赖)  
add_library(matplotlib_cpp INTERFACE)  
target_include_directories(matplotlib_cpp  
  INTERFACE  
    # 这里通常不需要包含examples，除非它们包含头文件  
    # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> 可能是更合适的  
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>  
)  
target_compile_features(matplotlib_cpp INTERFACE cxx_std_14)  

# Link Python libraries (通常C++代码不直接链接Python库)  
# 但如果你确实需要（例如，通过pybind11等），可以这样设置  
if(Python3_FOUND AND Python3_Development_FOUND)  
  target_link_libraries(matplotlib_cpp INTERFACE Python3::Python)  
  if(Python3_NumPy_FOUND)  
    target_link_libraries(matplotlib_cpp INTERFACE Python3::NumPy)  
  else()  
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)  
  endif()  
endif()  

# Catkin libraries  
catkin_package()  

# Include directories  
include_directories(  
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)  

# 添加PCL库目录
link_directories(${PCL_LIBRARY_DIRS})

# Add executable  
add_executable(cubicSpline src/main.cpp src/CatmullRom.cpp)  
target_link_libraries(cubicSpline PRIVATE  
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp  
)  

# Install rules  
install(TARGETS  
  cubicSpline  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)  

# Install headers if needed  
# install(DIRECTORY include/  
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}  
#   FILES_MATCHING PATTERN "*.h"  
# )