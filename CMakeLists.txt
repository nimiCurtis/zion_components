cmake_minimum_required(VERSION 3.5)
project(zion_components)

################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# DEPEND
# find dependencies - I don't think all of them are requierd.  
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcutils REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(zion_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common features filters io segmentation surface sample_consensus) 

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  builtin_interfaces
  rcutils
  zion_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  EIGEN3
  PCL
  ${TM_DEP}
)
###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
###############################################################################
# INCLUDES and LIBS

include_directories(
  # include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/include
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###############################################################################
# SOURCES

### need to check carefully about unrelevant sources


set(STAIR_MODELING_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities/include/utilities.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/include/plane.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/include/stair.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/include/stair_modeling_component.hpp
)

set(STAIR_MODELING_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities/src/utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/src/plane.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/src/stair.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/src/stair_modeling_component.cpp
)

###############################################################################
# Bin and Install

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(stair_modeling_component SHARED
    ${STAIR_MODELING_INC}
    ${STAIR_MODELING_SRC})
target_compile_definitions(stair_modeling_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(stair_modeling_component
    ${DEPENDENCIES})
rclcpp_components_register_nodes(stair_modeling_component "zion::StairModeling")
set(node_plugins "${node_plugins}zion::StairModeling;$<TARGET_FILE:stair_modeling_component>\n")

###############################################################################
# Install 

# Mark executables and/or libraries for installation

install(TARGETS
  stair_modeling_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling/include/
  DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(
                      stair_modeling_component
                      )
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
