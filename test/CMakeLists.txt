# Fetch googletest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

set(TEST_PROJECT_NAME test-${PROJECT_NAME})
set(TEST_FILES "test.cpp")

add_executable(${TEST_PROJECT_NAME} ${TEST_FILES})
target_link_libraries(${TEST_PROJECT_NAME} gtest gtest_main ${PROJECT_NAME})

# Use gtest_discover_tests to add tests
include(GoogleTest)
gtest_discover_tests(${TEST_PROJECT_NAME})
