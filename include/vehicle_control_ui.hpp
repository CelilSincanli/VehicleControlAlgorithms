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

    // Per-algorithm live tuning state
    struct TunePP {
        bool  enabled            = false;
        float lookahead_distance = 5.0f;
        float lookahead_gain     = 0.5f;
        float max_speed          = 5.0f;
        int   search_window      = 40;
    } tunePP_;

    struct TuneAPP {
        bool  enabled        = false;
        float min_lookahead  = 1.5f;
        float max_lookahead  = 8.0f;
        float speed_gain     = 0.5f;
        float curvature_gain = 3.0f;
        float max_speed      = 5.0f;
        int   search_window  = 40;
    } tuneAPP_;

    struct TuneLQR {
        bool  enabled         = false;
        float q0              = 1.0f;
        float q1              = 1.0f;
        float q2              = 1.0f;
        float q3              = 1.0f;
        float q4              = 1.0f;
        float r_steering      = 1.0f;
        float r_acceleration  = 1.0f;
        float r_scale         = 4.0f;
        float time_step       = 0.016f;
        float max_speed       = 3.0f;
        int   dare_iterations = 150;
        float dare_threshold  = 0.01f;
        int   search_window   = 40;
    } tuneLQR_;

    struct TuneStanley {
        bool  enabled       = false;
        float stanley_gain  = 0.5f;
        float min_speed     = 0.1f;
        float max_speed     = 5.0f;
        float max_delta     = 1.0f;
        int   search_window = 40;
    } tuneStanley_;

    struct TuneMPC {
        bool  enabled       = false;
        int   horizon_N     = 15;
        int   iterations    = 10;
        float learning_rate = 0.08f;
        float fd_step       = 0.01f;
        float rollout_dt    = 0.1f;
        float max_speed     = 5.0f;
        float w_lat         = 1.0f;
        float w_heading     = 0.5f;
        float w_control     = 0.05f;
        int   search_window = 40;
        float max_delta     = 1.0f;
    } tuneMPC_;

    struct TuneMPPI {
        bool  enabled        = false;
        int   horizon_T      = 15;
        int   num_samples_K  = 128;
        float lambda         = 1.0f;
        float sigma_steer    = 0.3f;
        float rollout_dt     = 0.1f;
        float max_speed      = 5.0f;
        float w_lat          = 1.0f;
        float w_heading      = 0.5f;
        int   search_window  = 40;
        float max_delta      = 1.0f;
        int   rng_seed       = 42;
    } tuneMPPI_;

    bool paramsDirty_ = false;

    struct TooltipState {
        std::string description;
        ImVec2      sliderMin;
        ImVec2      sliderMax;
        double      lastHoverTime = -1.0;
    } tooltipState_;

    void RenderUI();
    void RenderMainScreen();
    void RenderSimulationScreen();
    void ShowNotification(const std::string& title, const std::string& text, float duration_ms);
    void RenderNotification();
    void ResetSimulation();
    void InitSimulation();
    void RebuildAlgorithm();
    void TickSimulation(float dt);
    void DrawVehicle(ImDrawList* dl);
    void RenderParamTuning(float availableHeight);
};

#endif // VEHICLE_CONTROL_UI_HPP
