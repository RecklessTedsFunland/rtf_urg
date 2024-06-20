cmake_minimum_required(VERSION 3.20)
PROJECT(lidar
    VERSION "2024.06.19"
    LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic)
string (APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 ")
string (APPEND CMAKE_C_FLAGS_RELEASE " -O3 ")

if(PROJECT_IS_TOP_LEVEL)
    cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
    cmake_host_system_information(RESULT OSN QUERY OS_NAME)
    cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
    cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

    message(STATUS "-------------------------------------")
    message(STATUS "  Project: ${PROJECT_NAME}")
    message(STATUS "  Version: ${CMAKE_PROJECT_VERSION}")
    message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
    message(STATUS "  C ${CMAKE_C_STANDARD}")
    message(STATUS "-------------------------------------")
    message(STATUS " Host: ${HOST}")
    message(STATUS " ${OSN}: ${OS_VERSION}")
    message(STATUS " ${PROC}")
    message(STATUS "-------------------------------------")

    set(BUILD_EXAMPLES ON)
    set(BUILD_GTESTS ON)

    # GTest -----------------
    # include(FetchContent)

    # FetchContent_Declare(gtest
    #     GIT_REPOSITORY "https://github.com/google/googletest"
    #     GIT_TAG "origin/main"
    # )

    # FetchContent_MakeAvailable(gtest)

else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
    set(BUILD_EXAMPLES OFF)
    set(BUILD_GTESTS OFF)
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/../src)

add_executable(urg_linux
    ${CMAKE_SOURCE_DIR}/../src/urg_linux.cpp)
# target_include_directories(node PUBLIC
#     ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(urg_linux PRIVATE ${PROJECT_NAME})

# # ----------------------------------------------
# message(STATUS "Building ${PROJECT_NAME} examples is ${BUILD_EXAMPLES}")
# if (BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()

# # ----------------------------------------------
# message(STATUS "Building ${PROJECT_NAME} gtests is ${BUILD_GTESTS}")
# if (BUILD_GTESTS)
#     add_subdirectory(gtests)
# endif()
