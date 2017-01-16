cmake_minimum_required(VERSION 2.8.3)
project(benchmark_nlpmp)

## show all files (http://wiki.ros.org/IDEs#QtCreator)
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR})
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  resource_retriever
  cmake_modules
  eigen_conversions
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# cmake_modules
find_package(cmake_modules REQUIRED)

# eigen
find_package(Eigen REQUIRED)

# openmp
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES benchmark_nlpmp
#  CATKIN_DEPENDS roscpp resource_retriever cmake_modules eigen_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(benchmark_laptop src/benchmark_laptop.cpp)
target_link_libraries(benchmark_laptop ${catkin_LIBRARIES})


