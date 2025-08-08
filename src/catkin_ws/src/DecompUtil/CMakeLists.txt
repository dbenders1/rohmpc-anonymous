cmake_minimum_required(VERSION 2.8.3)
project(decomp_util)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations") # without optimization
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -fno-math-errno -fno-trapping-math -DNDEBUG -Wno-deprecated-declarations") # previously used

# To use a pre-processed map (DECOMP_OLD=FALSE) or not (DECOMP_OLD=TRUE)
set(DECOMP_OLD FALSE)
if(DECOMP_OLD)
  add_definitions(-DDECOMP_OLD)
endif()

set(BUILD_SHARED_LIBS ON)

include(CTest)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  mpc_tools
)

# Define the current package
catkin_package(
  CATKIN_DEPENDS mpc_tools
  INCLUDE_DIRS include
               ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

# Directories to include
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
)

# Library sources 
list(APPEND LIBRARY_SOURCES
  src/decomp_basis/data_utils.cpp
  src/decomp_geometry/ellipsoid.cpp
  src/decomp_geometry/geometric_utils.cpp
  src/decomp_geometry/polyhedron.cpp
  src/decomp_base.cpp
  src/ellipsoid_decomp.cpp 
  src/iterative_decomp.cpp 
  src/line_segment.cpp 
  src/seed_decomp.cpp 
)

# Library headers
list(APPEND LIBRARY_HEADERS
  include/decomp_util/decomp_basis/data_type.h
  include/decomp_util/decomp_basis/data_utils.h
  include/decomp_util/decomp_geometry/ellipsoid.h
  include/decomp_util/decomp_geometry/geometric_utils.h
  include/decomp_util/decomp_geometry/polyhedron.h
  include/decomp_util/decomp_base.h
  include/decomp_util/ellipsoid_decomp.h
  include/decomp_util/iterative_decomp.h
  include/decomp_util/line_segment.h
  include/decomp_util/seed_decomp.h
) 

# Add the sources and headers to the library
add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
# Link the library to other libraries needed
target_link_libraries(${PROJECT_NAME} ${thirdparty_libraries})
# Specify include directories to use when compiling given target
target_include_directories(${PROJECT_NAME} PUBLIC include/)

# Installation rules for the created library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add_executable(test_seed_decomp test/test_seed_decomp.cpp)
# target_link_libraries(test_seed_decomp ${Boost_LIBRARIES})
# add_test(test_seed_decomp test_seed_decomp)

# add_executable(test_line_segment test/test_line_segment.cpp)
# target_link_libraries(test_line_segment ${Boost_LIBRARIES})
# add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
# target_link_libraries(test_ellipsoid_decomp ${Boost_LIBRARIES})
# add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
# target_link_libraries(test_iterative_decomp ${Boost_LIBRARIES})
# add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)