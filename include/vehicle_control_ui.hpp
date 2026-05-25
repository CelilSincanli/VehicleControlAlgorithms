#ifndef VEHICLE_CONTROL_UI_HPP
#define VEHICLE_CONTROL_UI_HPP

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>

#include <cmath>
#include <memory>

#include "dripicon_v2.h"
#include "dripicon_v2_icons.hpp"
#include "map/map_loader.hpp"
#include "vehicle/vehicle_loader.hpp"
#include "vehicle/vehicle_bicycle_model.hpp"
#include "vehicle/vehicle_state.hpp"
#include "path_tracking/path_tracking_algorithm.hpp"

class VehicleControlUI {
public:
    VehicleControlUI();
    ~VehicleControlUI();

    bool Initialize();
    void Run();
    void Cleanup();

private:
    GLFWwindow* window;
    ImFont* mainFont;
    ImFont* headingFont;
    ImFont* iconFont;
    ImFont* iconFont2;

    bool vehicleSelected;
    bool algorithmSelected;
    bool pathSelected;

    VehicleLoader            vehicle_loader_;
    std::vector<std::string> available_vehicles_;

    MapLoader                map_loader_;
    std::vector<std::string> available_maps_;

    std::string              selectedAlgorithm_;
    static const std::vector<std::string> kAvailableAlgorithms;

    struct Notification {
        std::string title;
        std::string text;
        std::chrono::steady_clock::time_point start_time;
        float duration;
    };

    std::vector<Notification> notifications;

    enum Screen { MAIN_SCREEN, SIMULATION_SCREEN };
    Screen currentScreen;

    enum SimState { SIM_IDLE, SIM_RUNNING, SIM_PAUSED, SIM_DONE };
    SimState simRunState_;

    vehicle::VehicleState                            simVehicleState_;
    std::unique_ptr<vehicle::KinematicBicycleModel>  simModel_;
    std::unique_ptr<path_tracking::IPathTrackingAlgorithm> simPursuit_;
    std::vector<Point2D>                             simScaledPath_;  // in real meters
    std::vector<float>                               simTraceX_, simTraceY_;  // map units
    double lastSimTime_;
    float  simScale_;
    float  simHalfFront_;   // rear-axle → front bumper  [map units]
    float  simHalfRear_;    // rear-axle → rear bumper   [map units]
    float  simHalfWidth_;   // half vehicle width         [map units]
    float  simWheelbaseMap_;// wheelbase in map units
    float  simTireHL_;      // tire half-length           [map units]
    float  simTireHW_;      // tire half-width            [map units]
    float  simTreadHW_;     // half-tread (axle to tire center) [map units]
    float  simLastDelta_  = 0.0f;
    float  simMaxSpeed_   = 3.0f;

    void RenderUI();
    void RenderMainScreen();
    void RenderSimulationScreen();
    void ShowNotification(const std::string& title, const std::string& text, float duration_ms);
    void RenderNotification();
    void ResetSimulation();
    void InitSimulation();
    void TickSimulation(float dt);
    void DrawVehicle(ImDrawList* dl);
};

#endif // VEHICLE_CONTROL_UI_HPP
