cmake_minimum_required(VERSION 3.14.0)
project(rondo-graph
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "Rondo Graph Library")

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR})

# Use FetchContent
include(FetchContent)

# Fetch nlohmann/json
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.0
)
FetchContent_MakeAvailable(json)
target_link_libraries(${PROJECT_NAME} INTERFACE nlohmann_json::nlohmann_json)

# Fetch googletest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

# Setup googletest
enable_testing()
set(TEST_PROJECT_NAME test-${PROJECT_NAME})
set(TEST_FILES ${CMAKE_SOURCE_DIR}/test/test.cpp)
add_executable(${TEST_PROJECT_NAME} ${TEST_FILES})

target_link_libraries(${TEST_PROJECT_NAME} gtest gtest_main ${PROJECT_NAME})

# Add tests
add_test(NAME all COMMAND ${TEST_PROJECT_NAME})
