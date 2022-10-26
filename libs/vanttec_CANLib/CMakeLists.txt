cmake_minimum_required(VERSION 3.11)
project(Vanttec_CANLib)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

file(GLOB_RECURSE Vanttec_CANLib_SRC src/*.cpp)

include_directories(src src/Vanttec_CANLib)
add_executable(Vanttec_CANLib main.cpp ${Vanttec_CANLib_SRC})
target_link_libraries(Vanttec_CANLib pthread)

file(GLOB_RECURSE Vanttec_CANLib_Tests tests/*.cpp)

enable_testing()
add_executable(Vanttec_CANLib_test ${Vanttec_CANLib_SRC} ${Vanttec_CANLib_Tests})
target_link_libraries(Vanttec_CANLib_test gtest_main)

include(GoogleTest)
gtest_discover_tests(Vanttec_CANLib_test)
