cmake_minimum_required(VERSION 3.5)
project(sh_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/include/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        )
add_executable(talker ${all_SRCS})
# add_executable(listener src/subscriber_member_function.cpp)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED system filesystem serialization)
find_package(ompl REQUIRED)


include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(talker
          ${OMPL_LIBRARIES}
          ${Boost_FILESYSTEM_LIBRARY}
          ${Boost_SYSTEM_LIBRARY}
          ${Boost_SERIALIZATION_LIBRARY})
# target_include_directories(listener PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  talker
  "rclcpp"
  "std_msgs"
  "visualization_msgs"
  "ompl"
)
# ament_target_dependencies(
#   listener
#   "rclcpp"
#   "std_msgs"
#   "visualization_msgs"
# )

install(TARGETS talker
  # listener
  DESTINATION lib/${PROJECT_NAME})
ament_package()
