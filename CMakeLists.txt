cmake_minimum_required(VERSION 3.20)
project(sat-eqmod VERSION 0.9.0 LANGUAGES CXX)

# Build identity (src/sateq/version.h): git HEAD at configure time.
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE SATEQ_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT SATEQ_GIT_SHA)
    set(SATEQ_GIT_SHA "nogit")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# clangd / mcp-cpp-server semantic navigation: build/compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra)

find_package(Threads REQUIRED)

add_library(sateq
    src/sateq/framing.cpp
    src/sateq/board.cpp
    src/sateq/simulator.cpp
    src/sateq/astro.cpp
    src/sateq/config.cpp
    src/sateq/session.cpp
    src/sateq/safety.cpp
    src/sateq/rotctld.cpp
    src/sateq/tracker.cpp
)
target_include_directories(sateq PUBLIC src)
target_compile_definitions(sateq PUBLIC
    SATEQ_VERSION="${PROJECT_VERSION}" SATEQ_GIT="${SATEQ_GIT_SHA}")
target_link_libraries(sateq PUBLIC Threads::Threads)

add_executable(eqtool tools/eqtool.cpp)
target_link_libraries(eqtool PRIVATE sateq)
add_executable(replay_gt2 tools/replay_gt2.cpp)
target_link_libraries(replay_gt2 PRIVATE sateq)

add_executable(sateqd src/main.cpp)
target_link_libraries(sateqd PRIVATE sateq)

enable_testing()
file(GLOB TEST_SOURCES tests/test_*.cpp)
foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE sateq)
    target_include_directories(${test_name} PRIVATE tests)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
