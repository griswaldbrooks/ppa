cmake_minimum_required(VERSION 3.16)
project(say-hello VERSION 1.0.0 LANGUAGES CXX)

add_library(say-hello src/speaker.cpp)
target_include_directories(say-hello PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS say-hello
    EXPORT say-helloTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

install(EXPORT say-helloTargets
    FILE say-helloTargets.cmake
    NAMESPACE say-hello::
    DESTINATION lib/cmake/say-hello)

include(CMakePackageConfigHelpers)

install(FILES "say-helloConfig.cmake"
    DESTINATION lib/cmake/say-hello)
