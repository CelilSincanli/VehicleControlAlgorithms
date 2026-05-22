cmake_minimum_required(VERSION 3.10)
project(vehicle_control_algorithms)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_definitions(-DASSET_PATH="${CMAKE_SOURCE_DIR}/include/assets/fonts")
add_definitions(-DDATA_PATH="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DCONFIG_PATH="${CMAKE_SOURCE_DIR}/config")

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/vehicle_control_ui.cpp
    src/map/map_loader.cpp
    src/vehicle/vehicle_loader.cpp
    src/vehicle/vehicle_bicycle_model.cpp
    src/path_tracking/pure_pursuit/pure_pursuit.cpp
    src/path_tracking/pure_pursuit/pure_pursuit_loader.cpp
    src/path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit.cpp
    src/path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.cpp
    src/path_tracking/lqr/lqr.cpp
    src/path_tracking/lqr/lqr_loader.cpp
    include/third_parties/imgui/imgui.cpp
    include/third_parties/imgui/imgui_draw.cpp
    include/third_parties/imgui/imgui_tables.cpp
    include/third_parties/imgui/imgui_widgets.cpp
    include/third_parties/imgui/backends/imgui_impl_glfw.cpp
    include/third_parties/imgui/backends/imgui_impl_opengl3.cpp
    include/third_parties/implot/implot.cpp
    include/third_parties/implot/implot_items.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/map
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vehicle
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path_tracking/pure_pursuit
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CMAKE_CURRENT_SOURCE_DIR}/include/third_parties/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/include/third_parties/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/include/third_parties/implot
    ${CMAKE_CURRENT_SOURCE_DIR}/include/assets/fonts
    ${CMAKE_CURRENT_SOURCE_DIR}/include/assets/images
    ${CMAKE_CURRENT_SOURCE_DIR}/include/assets/icons
    ${CMAKE_CURRENT_SOURCE_DIR}/include/third_parties/stb
)

target_link_libraries(${PROJECT_NAME} glfw OpenGL::GL dl z stdc++fs)
