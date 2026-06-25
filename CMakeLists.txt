cmake_minimum_required(VERSION 3.14)
project(sensor_fusion_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Strict warnings. The GCC/Clang set matches the Makefile; MSVC gets its
# closest equivalent (plus _CRT_SECURE_NO_WARNINGS to silence the C4996
# nag about standard fopen) so the Windows CI build is clean too.
if(MSVC)
    # /wd4324: the SPSC ring buffer is deliberately padded with alignas to
    # avoid false sharing — that "structure was padded" warning is the
    # design working as intended, not a problem.
    add_compile_options(/W4 /permissive- /wd4324 /D_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(
        -Wall -Wextra -Wpedantic -Wshadow
        -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
        -Wconversion -Wsign-conversion
    )
endif()

find_package(Threads REQUIRED)

# Header-only core library. All pipeline modules are headers under
# include/.
add_library(sfp_core INTERFACE)
target_include_directories(sfp_core
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sfp_core INTERFACE Threads::Threads)
target_compile_features(sfp_core INTERFACE cxx_std_17)

# Main benchmark binary.
add_executable(sfp src/main.cpp)
target_link_libraries(sfp PRIVATE sfp_core)

# Tests.
enable_testing()

add_executable(test_ring_buffer tests/test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE sfp_core)
add_test(NAME ring_buffer COMMAND test_ring_buffer)

add_executable(test_sensor tests/test_sensor.cpp)
target_link_libraries(test_sensor PRIVATE sfp_core)
add_test(NAME sensor COMMAND test_sensor)

add_executable(test_estimator tests/test_estimator.cpp)
target_link_libraries(test_estimator PRIVATE sfp_core)
add_test(NAME estimator COMMAND test_estimator)

add_executable(test_logger tests/test_logger.cpp)
target_link_libraries(test_logger PRIVATE sfp_core)
add_test(NAME logger COMMAND test_logger)

add_executable(test_kalman tests/test_kalman.cpp)
target_link_libraries(test_kalman PRIVATE sfp_core)
add_test(NAME kalman COMMAND test_kalman)

# Optional TSAN build: `cmake -DSFP_TSAN=ON ..`
option(SFP_TSAN "Build tests with ThreadSanitizer" OFF)
if(SFP_TSAN)
    foreach(tgt test_ring_buffer test_sensor test_estimator test_logger test_kalman)
        target_compile_options(${tgt} PRIVATE -fsanitize=thread -g -O1)
        target_link_options(${tgt} PRIVATE -fsanitize=thread)
    endforeach()
endif()
