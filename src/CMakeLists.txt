set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

add_library(graph_lib INTERFACE)
target_include_directories(graph_lib INTERFACE ${INCLUDE_DIR})

add_executable(main main.cpp)
target_link_libraries(main graph_lib)