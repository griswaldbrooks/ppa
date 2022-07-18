cmake_minimum_required(VERSION 3.16)

project(say-hello)

add_compile_options(-std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-missing-field-initializers)

find_package(GTest REQUIRED)
include_directories(include ${GTEST_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/speaker.cpp)

add_executable(${PROJECT_NAME}_tests
  test/say_hello_tests.cpp
	)

target_include_directories(${PROJECT_NAME}_tests PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:include>
	)

target_link_libraries(${PROJECT_NAME}_tests
  ${GTEST_LIBRARIES}
  pthread
	${PROJECT_NAME}
	)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME})

include(CMakePackageConfigHelpers)

install(FILES "say-helloConfig.cmake"
    DESTINATION lib/cmake/say-hello)
