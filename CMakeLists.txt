cmake_minimum_required(VERSION 3.10)
project(ganisakta)

set(CMAKE_CXX_STANDARD 17)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz
)
FetchContent_MakeAvailable(googletest)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ganisakta
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

# ------- Make Library -------
set(SOURCES src/triangle/triangle.cpp src/quaternion/quaternion.cpp src/angle/angle.cpp src/angle/angle.utils.cpp src/matrix/transformation.cpp src/matrix/rotation.cpp src/point/point.cpp src/vector/vector.cpp)
# set(HEADERS ${INCLUDE_DIR}/type/triangle.hpp ${INCLUDE_DIR}/type/quaternion.hpp ${INCLUDE_DIR}/type/eulerangles.hpp)

add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# ------- Make Library -------

# ------- Test Files -------
add_executable(TriangleTest test/triangle.test.cpp)
add_executable(QuaternionTest test/quaternion.test.cpp)
add_executable(AnglesTest test/angle.test.cpp)
add_executable(AnglesUtilsTest test/angle.utils.test.cpp)

# Link your test files with gtest library
target_link_libraries(TriangleTest gtest gtest_main pthread ${PROJECT_NAME})
target_link_libraries(QuaternionTest gtest gtest_main pthread ${PROJECT_NAME})
target_link_libraries(AnglesTest gtest gtest_main pthread ${PROJECT_NAME})
target_link_libraries(AnglesUtilsTest gtest gtest_main pthread ${PROJECT_NAME})

# Add your test files here
add_test(
  NAME TriangleTest
  COMMAND TriangleTest
)
add_test(
  NAME QuaternionTest
  COMMAND QuaternionTest
)
add_test(
  NAME AnglesTest
  COMMAND AnglesTest
)
add_test(
  NAME AnglesUtilsTest
  COMMAND AnglesUtilsTest
)
# ------- Test Files -------