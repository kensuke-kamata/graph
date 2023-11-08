cmake_minimum_required(VERSION 3.14.0)
project(rondo-graph
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "Rondo Graph Library")

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

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

# Add subdirectories
if(BUILD_RONDO_GRAPH_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
