cmake_minimum_required(VERSION 3.16)
project(rosful)

# TODO: Upgrade to cmake 3.19 or above and use presets
# https://alexreinking.com/blog/how-to-use-cmake-without-the-agonizing-pain-part-2.html
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(say-hello REQUIRED)

add_executable(node src/node.cpp)
ament_target_dependencies(node PUBLIC rclcpp)
target_link_libraries(node PRIVATE say-hello::say-hello)

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
