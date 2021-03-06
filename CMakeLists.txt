cmake_minimum_required(VERSION 3.5)
project(chronoros)

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

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Vehicle Sensor Irrlicht
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(./include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

get_filename_component(ABSOLUTE_CHRONOROSDATA_PATH "./data/" ABSOLUTE)
add_definitions( -DCHRONOROS_DATA_DIR="${ABSOLUTE_CHRONOROSDATA_PATH}")
add_definitions(-DASSETS_FOLDER_PATH=\"${ASSETS_PATH}\")



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_add_executable(chrono_test_node src/chrono_test.cpp)

ament_auto_add_library(ChRos SHARED
        src/ChRosVehicle.cpp
        src/ChRosNode.cpp)

target_link_libraries(ChRos ${CHRONO_LIBRARIES})
target_link_libraries(chrono_test_node ChRos)

ament_target_dependencies(ChRos rclcpp geometry_msgs sensor_msgs autoware_auto_msgs autoware_auto_msgs Chrono)
add_dependencies(chrono_test_node ChRos)

set_target_properties(ChRos PROPERTIES
        COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
        COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
        LINK_FLAGS "${CHRONO_LINKER_FLAGS}")


install(TARGETS
  ChRos
  DESTINATION lib
)

install(TARGETS
    chrono_test_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
