
# GTest -----------------
include(FetchContent)

find_package(GTest REQUIRED)

# FetchContent_Declare(gtest
#   GIT_REPOSITORY "https://github.com/google/googletest"
#   GIT_TAG "origin/main"
# )
# FetchContent_MakeAvailable(gtest)
if(GTest_FOUND)
    message(STATUS "=> Found gtest")
else()
    message(STATUS "*** Didn't find gtest")
endif()

#----------------------------
enable_testing()

list(APPEND gci-gtests
    main
)

message(STATUS "gci::gTests ----------------------")
foreach(name ${gci-gtests})
    # message(STATUS " -> ${name}")
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name}
        GTest::gtest_main
        # GTest::gtest
        ${PROJECT_NAME})
    add_test(NAME ${name} COMMAND ${name})
endforeach()

include(GoogleTest)
gtest_discover_tests(${gci-gtests})