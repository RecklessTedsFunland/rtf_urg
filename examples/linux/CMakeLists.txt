list(APPEND apps
  urg_node
)

foreach(app ${apps})
  add_executable(${app} ${app}.cpp)
  # target_include_directories(node PUBLIC
  #     ${CMAKE_SOURCE_DIR}/src)
  target_link_libraries(${app} PRIVATE ${PROJECT_NAME})
endforeach()
