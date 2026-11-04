cmake_minimum_required(VERSION 3.10)
project(vehicle_control_algorithms)

set(CMAKE_CXX_STANDARD 17)

# Prefer the GLVND OpenGL libraries (silences CMake policy CMP0072).
set(OpenGL_GL_PREFERENCE GLVND)

# The GUI needs GLFW + OpenGL; the core library and the tests do not, so a
# headless CI machine can build and run every test with -DVCA_BUILD_GUI=OFF.
option(VCA_BUILD_GUI "Build the ImGui/GLFW application" ON)

add_definitions(-DASSET_PATH="${CMAKE_SOURCE_DIR}/include/assets/fonts")
add_definitions(-DIMAGE_PATH="${CMAKE_SOURCE_DIR}/include/assets/images")
add_definitions(-DDATA_PATH="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DCONFIG_PATH="${CMAKE_SOURCE_DIR}/config")
add_definitions(-DDEBUG_LOG_PATH="${CMAKE_SOURCE_DIR}/debug_logs")
add_definitions(-DREPO_ROOT="${CMAKE_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# Core: maps, vehicles, vehicle models, path tracking, simulation rules.
# No ImGui/ImPlot/GLFW/OpenGL dependency.
# ---------------------------------------------------------------------------
add_library(vca_core STATIC
    src/map/map_loader.cpp
    src/vehicle/vehicle_loader.cpp
    src/vehicle/vehicle_bicycle_model.cpp
    src/vehicle/composed_vehicle_model.cpp
    src/vehicle/vehicle_model_factory.cpp
    src/vehicle/tire/linear_tire_model.cpp
    src/vehicle/suspension/rigid_suspension_model.cpp
    src/vehicle/drivetrain/direct_accel_drivetrain_model.cpp
    src/vehicle/aero/simple_aero_model.cpp
    src/vehicle/steering/instantaneous_steering_model.cpp
    src/vehicle/steering/rate_limited_steering_model.cpp
    src/path_tracking/pure_pursuit/pure_pursuit.cpp
    src/path_tracking/pure_pursuit/pure_pursuit_loader.cpp
    src/path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit.cpp
    src/path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.cpp
    src/path_tracking/lqr/lqr.cpp
    src/path_tracking/lqr/lqr_loader.cpp
    src/path_tracking/stanley/stanley.cpp
    src/path_tracking/stanley/stanley_loader.cpp
    src/path_tracking/mppi/mppi.cpp
    src/path_tracking/mppi/mppi_loader.cpp
    src/path_tracking/mpc/mpc.cpp
    src/path_tracking/mpc/mpc_loader.cpp
    src/path_tracking/path_feasibility.cpp
    src/sim/sim_rules.cpp
)
target_include_directories(vca_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/map
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vehicle
)

# ---------------------------------------------------------------------------
# GUI application
# ---------------------------------------------------------------------------
if(VCA_BUILD_GUI)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)

    add_executable(${PROJECT_NAME}
        src/main.cpp
        src/vehicle_control_ui.cpp
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

    target_link_libraries(${PROJECT_NAME} vca_core glfw OpenGL::GL dl z stdc++fs)
endif()

# ---------------------------------------------------------------------------
# Tests — all headless (no display or GPU needed)
#   vehicle_model_tests : vehicle subsystem unit tests
#   edge_tests          : edge cases and regression checks for every audit fix
#   sim_harness         : closed-loop scenario runner (also used by tests/run_matrix.py)
# ---------------------------------------------------------------------------
add_executable(vehicle_model_tests tests/vehicle_model_tests.cpp)
target_link_libraries(vehicle_model_tests vca_core)

add_executable(edge_tests tests/edge_tests.cpp)
target_link_libraries(edge_tests vca_core)

add_executable(sim_harness tests/sim_harness.cpp)
target_link_libraries(sim_harness vca_core)

enable_testing()
add_test(NAME vehicle_model_tests COMMAND vehicle_model_tests)
add_test(NAME edge_tests          COMMAND edge_tests)
add_test(NAME edge_mpc_horizon_0  COMMAND edge_tests mpc_h0)
add_test(NAME edge_mppi_horizon_0 COMMAND edge_tests mppi_t0)

# Closed-loop smoke scenarios: each must reach the goal without contact.
foreach(scenario
        "map_01;vehicle02;Pure Pursuit;kin"
        "map_02;vehicle01;Stanley;dyn"
        "map_03;vehicle02;LQR;dyn"
        "map_01;vehicle01;MPC;dyn"
        "map_02;vehicle02;MPPI;dyn"
        "map_03;vehicle01;Adaptive Pure Pursuit;dyn")
    list(GET scenario 0 MAP)
    list(GET scenario 1 VEH)
    list(GET scenario 2 ALGO)
    list(GET scenario 3 FID)
    string(REPLACE " " "_" ALGO_ID "${ALGO}")
    add_test(NAME "scenario_${MAP}_${VEH}_${ALGO_ID}_${FID}"
             COMMAND sim_harness --map ${CMAKE_SOURCE_DIR}/data/maps/${MAP}
                                 --vehicle ${CMAKE_SOURCE_DIR}/data/vehicle/${VEH}.json
                                 --algo "${ALGO}" --fidelity ${FID})
    set_tests_properties("scenario_${MAP}_${VEH}_${ALGO_ID}_${FID}" PROPERTIES PASS_REGULAR_EXPRESSION "\"clean\":true")
endforeach()
