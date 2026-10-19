cmake_minimum_required(VERSION 3.16)
project(perimeter_defense_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

add_library(ActorDriver SHARED src/ActorDriver.cc)
target_link_libraries(ActorDriver
  gz-sim::core
  gz-plugin::register
  gz-transport::core
  gz-msgs::core
  gz-math::core
)
install(TARGETS ActorDriver LIBRARY DESTINATION lib)

add_library(ClockThrottle SHARED src/ClockThrottle.cc)
target_link_libraries(ClockThrottle
  gz-sim::core
  gz-plugin::register
  gz-transport::core
  gz-msgs::core
)
install(TARGETS ClockThrottle LIBRARY DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
