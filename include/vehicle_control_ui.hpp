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
#include <deque>

#include <cmath>
#include <memory>

#include "dripicon_v2.h"
#include "dripicon_v2_icons.hpp"
#include "map/map_loader.hpp"
#include "vehicle/vehicle_loader.hpp"
#include "vehicle/vehicle_state.hpp"
#include "vehicle/i_vehicle_model.hpp"
#include "vehicle/vehicle_model_factory.hpp"
#include "path_tracking/path_tracking_algorithm.hpp"
#include "path_tracking/path_feasibility.hpp"
#include "tuning_limits.hpp"
#include "sim/sim_rules.hpp"

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

    std::string              selectedVehicleModelKind_ = "Kinematic";
    static const std::vector<std::string> kAvailableVehicleModels;

    struct Notification {
        std::string title;
        std::string text;
        std::chrono::steady_clock::time_point start_time;
        float duration;
    };

    std::vector<Notification> notifications;

    enum Screen { MAIN_SCREEN, SIMULATION_SCREEN };
    Screen currentScreen;

    enum SimState { SIM_IDLE, SIM_RUNNING, SIM_PAUSED, SIM_DONE, SIM_FAILED };
    SimState simRunState_;

    vehicle::VehicleState                            simVehicleState_;
    std::unique_ptr<vehicle::IVehicleModel>          simModel_;
    std::unique_ptr<path_tracking::IPathTrackingAlgorithm> simPursuit_;
    std::vector<Point2D>                             simScaledPath_;  // in real meters
    std::vector<float>                               simTraceX_, simTraceY_;  // map units, capped at kMaxTracePoints
    std::vector<float>                               pathPlotX_, pathPlotY_;  // reference path in map units, built once per map load
    double lastSimTime_;
    double simAccumulator_ = 0.0;            // wall time not yet simulated [s] (fixed-step loop)
    float  simElapsed_     = 0.0f;           // simulated time of the current run [s]
    float  simPathLength_  = 0.0f;           // [m]
    float  simPrevAccel_   = 0.0f;           // last speed command, for the jerk limit [m/s^2]
    sim::Outcome simOutcome_ = sim::Outcome::Running;
    float  simScale_;
    float  simHalfFront_;   // rear-axle → front bumper  [map units]
    float  simHalfRear_;    // rear-axle → rear bumper   [map units]
    float  simHalfWidth_;   // half vehicle width         [map units]
    float  simWheelbaseMap_;// wheelbase in map units
    float  simTireHL_;      // tire half-length           [map units]
    float  simTireHW_;      // tire half-width            [map units]
    float  simTreadHW_;     // half-tread (axle to tire center) [map units]
    float  simLastDelta_  = 0.0f;           // last commanded tire angle [rad] (applied angle is in simVehicleState_)
    float  simMaxSpeed_   = 3.0f;

    // Debug telemetry: one sample per simulation tick, from InitSimulation()
    // (clears the log) until the user saves it. Independent of simTraceX_/Y_,
    // which only records a dot every 0.5 m for the plot — this is every tick.
    struct DebugSample {
        float time_s;
        float x_m, y_m;
        float heading_deg;
        float speed_mps;
        float lateral_speed_mps;
        float yaw_rate_radps;
        float steering_angle_deg;
        float steering_rate_degps;
        float acceleration_cmd_mps2;
        float lookahead_x_m, lookahead_y_m;
        float sideslip_deg;         // atan2(lateral_speed, speed) — valid for every model
        bool  tire_diag_valid;      // false under Kinematic fidelity (no tire model)
        float alpha_front_deg, alpha_rear_deg;
        float fy_front_n, fy_rear_n;
        float applied_steering_deg; // actual tire angle after actuator clamp/rate-limit
        bool  front_saturated, rear_saturated;
        bool  tracking_diag_valid;  // true only for algorithms that implement it (MPC)
        float cross_track_error_m;
        float heading_error_deg;
        int   nearest_path_idx;
        float mpc_final_cost;
        bool  steering_at_clamp;
    };
    // Bounded: the oldest samples are dropped beyond kMaxDebugSamples (10 min at
    // 60 Hz, ~4 MB) so a run that never terminates cannot grow memory forever.
    std::deque<DebugSample> simDebugLog_;
    size_t simDebugDropped_ = 0;
    static constexpr size_t kMaxDebugSamples = 36000;
    static constexpr size_t kMaxTracePoints  = 20000;
    float simDebugElapsedTime_  = 0.0f;
    float simPrevSteeringAngle_ = 0.0f;

    // Vehicle data actually fed into MakeVehicleModel() for the current
    // simModel_ (post live-tuning overlay) — captured for the debug CSV's
    // config metadata header so it never drifts from what simModel_ was
    // really built with. Set once, in RebuildVehicleModel().
    VehicleData simEffectiveVehicleData_;

    // Reference path measured against simEffectiveVehicleData_ (tightest
    // bend, speed cap). Recomputed whenever the vehicle model is rebuilt.
    path_tracking::PathFeasibility simFeasibility_;
    bool simFeasibilityValid_ = false;

    // Live performance read-out (Simulation Info panel)
    double perfFrameMs_     = 0.0;   // smoothed wall time per rendered frame
    double perfTickMs_      = 0.0;   // smoothed controller + vehicle-model time per tick
    double perfLastRssRead_ = -1.0;
    double perfRssMb_       = 0.0;
    double lastActivityTime_ = 0.0;  // last mouse/keyboard input, keeps rendering briefly after it

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
        float q5              = 1.0f;
        float r_steering      = 1.0f;
        float r_acceleration  = 1.0f;
        float r_scale         = 4.0f;
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
        int   iterations    = 30;
        float rollout_dt    = 0.1f;
        float max_speed     = 5.0f;
        float w_lat         = 1.0f;
        float w_heading     = 0.5f;
        float w_control     = 0.05f;
        float w_rate        = 0.1f;
        int   search_window = 40;
        float max_delta     = 1.0f;
    } tuneMPC_;

    struct TuneMPPI {
        bool  enabled        = false;
        int   horizon_T      = 20;
        int   num_samples_K  = 128;
        float lambda         = 1.0f;
        float sigma_steer    = 0.2f;
        float rollout_dt     = 0.15f;
        float max_speed      = 5.0f;
        float w_lat          = 1.0f;
        float w_heading      = 0.5f;
        int   search_window  = 40;
        float max_delta      = 1.0f;
        int   rng_seed       = 42;
    } tuneMPPI_;

    bool paramsDirty_ = false;

    // Vehicle-model subsystem live tuning state. Effective only under
    // ModelFidelity::DynamicLinearTire; the Kinematic model keeps an ideal
    // (instant) steering actuator by design.
    struct TuneTire {
        bool  enabled                   = false;
        float cornering_stiffness_coeff = 6.0f;
    } tuneTire_;

    struct TuneSuspension {
        bool  enabled    = false;
        float cog_height = 0.0f;
    } tuneSuspension_;

    struct TuneSteering {
        bool  enabled                 = false;
        bool  use_rate_limit          = false;
        float max_steering_wheel_rate = 0.0f;
    } tuneSteering_;

    bool modelDirty_ = false;

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
    void RebuildVehicleModel();
    void TickSimulation(float dt);
    void DrawVehicle(ImDrawList* dl);
    void RenderParamTuning(float availableHeight);
    void RenderVehicleModelTuning(float availableHeight);
    void SaveDebugLogCsv();

    bool  IsDynamicModel() const;
    const VehicleData& ActiveVehicleData() const;  // effective (tuned) data once built, else the loaded file
    float MaxSpeedLimit() const;                    // slider/JSON upper bound for every "Max Speed"
    float MaxDeltaLimit() const;                    // slider/JSON upper bound for every "Max Delta"
    float ActuatorRateLimit() const;                // steering rate the controllers plan with [rad/s]
    float CogHeightLimit() const;
    bool  ApplyTuningLimits(std::vector<std::string>& changes);  // clamps the active algorithm's tune struct
    void  NotifyLimited(const std::string& title, const std::vector<std::string>& lines);
    void  RebuildPathPlotCache();
    bool  NeedsContinuousRedraw() const;
    void  UpdatePerfStats(double frame_seconds);
};

#endif // VEHICLE_CONTROL_UI_HPP
