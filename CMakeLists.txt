cmake_minimum_required(VERSION 3.10)
project(ganisakta)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz
)
FetchContent_MakeAvailable(googletest)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ganisakta
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

# Library Dependencies
set(SOURCES src/type/triangle.cpp ${INCLUDE_DIR}/ganisakta/type/triangle.hpp)

# Add your test files here
add_executable(TriangleTest test/type/triangle-test.cpp ${SOURCES})

# Link your test files with gtest library
target_link_libraries(TriangleTest gtest gtest_main pthread)

# Add your test files here
add_test(
  NAME TriangleTest
  COMMAND TriangleTest
)