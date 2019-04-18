cmake_minimum_required(VERSION 3.5)
project(canopen_motor_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(rclcpp REQUIRED)
find_package(canopen_402 REQUIRED)
find_package(canopen_chain_node REQUIRED)
find_package(canopen_master REQUIRED)

# find_package(controller_manager REQUIRED)
# find_package(controller_manager_msgs REQUIRED)
# find_package(filters REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(joint_limits_interface REQUIRED)
# find_package(urdf REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
    thread
)

find_package(PkgConfig)
pkg_check_modules(PC_MUPARSER QUIET muparser)
set(MUPARSER_DEFINITIONS ${PC_MUPARSER_CFLAGS_OTHER})

find_path(MUPARSER_INCLUDE_DIR muParser.h
  HINTS ${PC_MUPARSER_INCLUDEDIR} ${PC_MUPARSER_INCLUDE_DIRS}
)

find_library(MUPARSER_LIBRARY NAMES muparser libmurser
   HINTS ${PC_MUPARSER_LIBDIR} ${PC_MUPARSER_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)

# handle the QUIETLY and REQUIRED arguments and set MUPARSER_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MUPARSER  DEFAULT_MSG
                                  MUPARSER_LIBRARY MUPARSER_INCLUDE_DIR)
mark_as_advanced(MUPARSER_INCLUDE_DIR MUPARSER_LIBRARY )

if(NOT ${MUPARSER_FOUND})
  message(FATAL_ERROR "muparser library not found")
endif()

set(MUPARSER_LIBRARIES ${MUPARSER_LIBRARY} )
set(MUPARSER_INCLUDE_DIRS ${MUPARSER_INCLUDE_DIR} )

include_directories(
  include
  ${MUPARSER_INCLUDE_DIRS}
)

# canopen_motor
add_library(canopen_motor
  src/controller_manager_layer.cpp
  src/handle_layer.cpp
  src/motor_chain.cpp
  src/robot_layer.cpp
)

target_link_libraries(canopen_motor
  ${MUPARSER_LIBRARIES}
  ${Boost_LIBRARIES}
)

# add_dependencies(canopen_motor
# )

ament_target_dependencies(canopen_motor
  "rclcpp"
  "canopen_chain_node"
  "canopen_402"
)

# # canopen_motor_node
add_executable(${PROJECT_NAME}
  src/canopen_motor_chain_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  canopen_motor
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
)

# add_dependencies(${PROJECT_NAME}
#   # ${catkin_EXPORTED_TARGETS}
# )

# ament_export_dependencies(
#   ament_cmake
#   canopen_402
#   canopen_chain_node
#   canopen_master
#   # controller_manager
#   # hardware_interface
#   # joint_limits_interface
#   rclcpp
#   # urdf
#   Boost
#   # MUPARSER
# )

ament_export_include_directories(include)
ament_export_libraries(canopen_motor)

install(
  TARGETS
    canopen_motor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#
#   find_package(ament_cmake_pytest REQUIRED)
#   find_package(rmw_implementation_cmake REQUIRED)
#
#   # catkin_add_gtest(${PROJECT_NAME}-test_muparser
#   #   test/test_muparser.cpp
#   # )
#   # target_link_libraries(${PROJECT_NAME}-test_muparser
#   #   canopen_motor
#   #   ${catkin_LIBRARIES}
#   #   ${MUPARSER_LIBRARIES}
#   # )
# endif()

ament_package()
