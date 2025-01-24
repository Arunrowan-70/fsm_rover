cmake_minimum_required(VERSION 3.23)

# Enable testing
enable_testing()

# Define the project name
project(RoverFSM)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Add source and header files for the unit tests and the RoverFSM logic
add_executable(unitTests
    src/Rover.cpp
    src/roverfsmwithEnums.h
    test/testRover_withenums.cpp
)

# Include directories for headers
# Include the main project `include/` folder to ensure FSM.h is found
target_include_directories(unitTests PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Fetch GoogleTest from GitHub repository
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG d83fee138a9ae6cb7c03688a2d08d4043a39815d  # release-1.10.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Link GoogleTest libraries with the unit test executable
target_link_libraries(unitTests
    gtest_main
)

# Automatically discover and run GoogleTest tests
include(GoogleTest)
gtest_discover_tests(unitTests)
