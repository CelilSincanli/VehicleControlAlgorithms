#include "vehicle_control_ui.hpp"
#include "map/map_data.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit_loader.hpp"
#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit.hpp"
#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.hpp"
#include "path_tracking/lqr/lqr.hpp"
#include "path_tracking/lqr/lqr_loader.hpp"
#include "path_tracking/stanley/stanley.hpp"
#include "path_tracking/stanley/stanley_loader.hpp"
#include "path_tracking/mppi/mppi.hpp"
#include "path_tracking/mppi/mppi_loader.hpp"
#include "path_tracking/mpc/mpc.hpp"
#include "path_tracking/mpc/mpc_loader.hpp"
#include <filesystem>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <chrono>
#include <limits>
#include <cstdio>
#ifdef __linux__
#include <unistd.h>
#endif

const std::vector<std::string> VehicleControlUI::kAvailableAlgorithms = {
    "Pure Pursuit",
    "Adaptive Pure Pursuit",
    "LQR",
    "Stanley",
    "MPPI",
    "MPC",
};

const std::vector<std::string> VehicleControlUI::kAvailableVehicleModels = {
    "Kinematic",
    "Dynamic (Linear Tire)",
};

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

VehicleControlUI::VehicleControlUI()
    : window(nullptr), mainFont(nullptr), headingFont(nullptr),
      iconFont(nullptr), iconFont2(nullptr),
      vehicleSelected(false), algorithmSelected(false), pathSelected(false),
      currentScreen(MAIN_SCREEN),
      simRunState_(SIM_IDLE), simVehicleState_{}, simModel_(nullptr), simPursuit_(nullptr),
      lastSimTime_(0.0), simScale_(1.0f),
      simHalfFront_(0.0f), simHalfRear_(0.0f), simHalfWidth_(0.0f),
      simWheelbaseMap_(0.0f), simTireHL_(0.0f), simTireHW_(0.0f), simTreadHW_(0.0f) {}

VehicleControlUI::~VehicleControlUI() {
    Cleanup();
}

bool VehicleControlUI::Initialize() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(1280, 720, "Vehicle Control Algorithms", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    {
        int w, h, ch;
        unsigned char* px = stbi_load(IMAGE_PATH "/vca_application_icon.png", &w, &h, &ch, 4);
        if (px) {
            GLFWimage icon{ w, h, px };
            glfwSetWindowIcon(window, 1, &icon);
            stbi_image_free(px);
        }
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();

    ImPlot::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    mainFont = io.Fonts->AddFontFromFileTTF(
        ASSET_PATH "/nasalization/nasalization rg.ttf", 18.5f, NULL, io.Fonts->GetGlyphRangesDefault());

    headingFont = io.Fonts->AddFontFromFileTTF(
        ASSET_PATH "/nasalization/nasalization rg.ttf", 30.0f, NULL, io.Fonts->GetGlyphRangesDefault());

    ImFontConfig fontConfig;
    fontConfig.PixelSnapH = true;
    fontConfig.FontDataOwnedByAtlas = false;
    static const ImWchar iconRanges[] = { 0xe000, 0xf8ff, 0 };
    iconFont = io.Fonts->AddFontFromMemoryCompressedTTF(
        dripiconfont_compressed_data, dripiconfont_compressed_size, 28.0f, &fontConfig, iconRanges);

    ImFontConfig icons_config;
    icons_config.MergeMode = false;
    icons_config.PixelSnapH = true;
    static const ImWchar iconRanges2[] = { 0x20, 0x7F, 0 };
    iconFont2 = io.Fonts->AddFontFromMemoryCompressedTTF(
        dripiconfont_compressed_data, dripiconfont_compressed_size, 28.0f, &icons_config, iconRanges2);

    return true;
}

void VehicleControlUI::Run() {
    double lastFrame = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        // Redraw at the display rate only while something on screen changes
        // (running simulation, recent input, fading tooltip, notification).
        // Otherwise block until input arrives or 0.25 s passes, so an idle
        // window costs almost no CPU/GPU.
        if (NeedsContinuousRedraw()) glfwPollEvents();
        else                         glfwWaitEventsTimeout(0.25);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {   // any user input keeps continuous rendering on for a short while
            const ImGuiIO& io = ImGui::GetIO();
            bool input = io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f || io.MouseWheel != 0.0f ||
                         ImGui::IsAnyItemActive() || io.InputQueueCharacters.Size > 0;
            for (int b = 0; b < ImGuiMouseButton_COUNT && !input; ++b) input = io.MouseDown[b];
            for (int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END && !input; ++k)
                input = ImGui::IsKeyDown(static_cast<ImGuiKey>(k));
            if (input) lastActivityTime_ = glfwGetTime();
        }

        RenderUI();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        double now = glfwGetTime();
        UpdatePerfStats(now - lastFrame);
        lastFrame = now;
    }
}

void VehicleControlUI::RenderUI() {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("Vehicle Control Algorithms", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    if (currentScreen == MAIN_SCREEN) {
        RenderMainScreen();
    } else if (currentScreen == SIMULATION_SCREEN) {
        RenderSimulationScreen();
    }

    ImGui::End();

    RenderNotification();  // own window, on top of whichever screen is active
}

void VehicleControlUI::RenderMainScreen() {
    ImVec2 window_size = ImGui::GetContentRegionAvail();

    ImGui::PushFont(headingFont);
    ImVec2 text_size = ImGui::CalcTextSize("Vehicle Control Algorithms");
    ImGui::PopFont();

    ImGui::PushFont(iconFont);
    ImVec2 icon_size = ImGui::CalcTextSize(dripicon_v2::map);
    ImGui::PopFont();

    float total_text_width = text_size.x + icon_size.x + 5.0f;
    float button_width = 220.0f;
    float button_height = 40.0f;
    float spacing = 20.0f;

    // 4 buttons + 3 spacers between them
    float total_height = text_size.y + spacing
        + button_height + spacing + button_height
        + spacing + button_height + spacing + button_height;
    float start_y = (window_size.y - total_height) * 0.5f;
    float text_x = (window_size.x - total_text_width) * 0.5f;

    ImGui::SetCursorPos(ImVec2(text_x, start_y));
    ImGui::PushFont(headingFont);
    ImGui::Text("Vehicle Control Algorithms");
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::map);
    ImGui::PopFont();

    float button_x = (window_size.x - button_width) * 0.5f;

    // Button 1: Select Vehicle
    start_y += text_size.y + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::rocket);
    ImGui::PopFont();
    ImGui::SameLine();
    if (ImGui::Button("Select Vehicle", ImVec2(button_width, button_height))) {
        available_vehicles_ = VehicleLoader::DiscoverVehicles(std::string(DATA_PATH) + "/vehicle");
        ImGui::OpenPopup("Select Vehicle##popup");
    }
    if (vehicleSelected) {
        ImGui::SameLine();
        ImGui::PushFont(iconFont2);
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "%s", dripicon_v2::checkmark);
        ImGui::PopFont();
        if (vehicle_loader_.GetVehicle().loaded) {
            ImGui::SameLine();
            ImGui::TextDisabled("(%s)", vehicle_loader_.GetVehicle().name.c_str());
        }
    }

    // Vehicle selector popup
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_Always);
    if (ImGui::BeginPopupModal("Select Vehicle##popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Available Vehicles");
        ImGui::Separator();
        if (available_vehicles_.empty()) {
            ImGui::TextDisabled("No vehicles found in data/vehicle/");
        }
        ImGui::BeginChild("##vehiclelist", ImVec2(300, 180), true);
        for (const auto& path : available_vehicles_) {
            std::string label = std::filesystem::path(path).stem().string();
            if (ImGui::Selectable(label.c_str())) {
                if (vehicle_loader_.Load(path)) {
                    vehicleSelected = true;
                    ResetSimulation();
                    // Vehicle-specific overrides (CoG height, steering rate, ...) belong to
                    // the previous vehicle: drop them so the new one starts from its file.
                    tuneTire_.enabled = tuneSuspension_.enabled = tuneSteering_.enabled = false;
                    NotifyLimited("Vehicle file adjusted: " + label, vehicle_loader_.GetWarnings());
                } else {
                    vehicleSelected = false;  // Load() cleared the data; don't allow Start with it
                    ShowNotification("Vehicle not loaded", label + ": " + vehicle_loader_.GetError(), 8000.0f);
                }
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndChild();
        ImGui::Spacing();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Button 2: Select Algorithm
    start_y += button_height + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::graph_line);
    ImGui::PopFont();
    ImGui::SameLine();
    if (ImGui::Button("Select Algorithm", ImVec2(button_width, button_height))) {
        ImGui::OpenPopup("Select Algorithm##popup");
    }
    if (algorithmSelected) {
        ImGui::SameLine();
        ImGui::PushFont(iconFont2);
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "%s", dripicon_v2::checkmark);
        ImGui::PopFont();
        ImGui::SameLine();
        ImGui::TextDisabled("(%s)", selectedAlgorithm_.c_str());
    }

    // Algorithm selector popup
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_Always);
    if (ImGui::BeginPopupModal("Select Algorithm##popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Available Algorithms");
        ImGui::Separator();
        ImGui::BeginChild("##algolist", ImVec2(300, 180), true);
        for (const auto& name : kAvailableAlgorithms) {
            if (ImGui::Selectable(name.c_str())) {
                selectedAlgorithm_ = name;
                algorithmSelected  = true;
                ResetSimulation();
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndChild();
        ImGui::Spacing();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Button 3: Select Map
    start_y += button_height + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::upload);
    ImGui::PopFont();
    ImGui::SameLine();
    if (ImGui::Button("Select Map", ImVec2(button_width, button_height))) {
        available_maps_ = MapLoader::DiscoverMaps(std::string(DATA_PATH) + "/maps");
        ImGui::OpenPopup("Select Map##popup");
    }
    if (pathSelected) {
        ImGui::SameLine();
        ImGui::PushFont(iconFont2);
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "%s", dripicon_v2::checkmark);
        ImGui::PopFont();
        if (map_loader_.GetMap().loaded) {
            ImGui::SameLine();
            ImGui::TextDisabled("(%s)", map_loader_.GetMap().name.c_str());
        }
    }

    // Map selector popup
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_Always);
    if (ImGui::BeginPopupModal("Select Map##popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Available Maps");
        ImGui::Separator();
        if (available_maps_.empty()) {
            ImGui::TextDisabled("No maps found in data/maps/");
        }
        ImGui::BeginChild("##maplist", ImVec2(300, 180), true);
        for (const auto& dir : available_maps_) {
            std::string label = std::filesystem::path(dir).filename().string();
            if (ImGui::Selectable(label.c_str())) {
                if (map_loader_.Load(dir)) {
                    pathSelected = true;
                    ResetSimulation();
                } else {
                    pathSelected = false;
                    ShowNotification("Map not loaded", label + ": " + map_loader_.GetError(), 8000.0f);
                }
                RebuildPathPlotCache();
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndChild();
        ImGui::Spacing();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Button 4: Start Simulation (requires all 3 selections)
    start_y += button_height + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::media_play);
    ImGui::PopFont();
    ImGui::SameLine();

    bool canStart = vehicleSelected && algorithmSelected && pathSelected;
    ImGui::BeginDisabled(!canStart);
    if (ImGui::Button("Start Simulation", ImVec2(button_width, button_height))) {
        currentScreen = SIMULATION_SCREEN;
    }
    ImGui::EndDisabled();
}

void VehicleControlUI::ResetSimulation() {
    simRunState_     = SIM_IDLE;
    simVehicleState_ = {};
    simLastDelta_    = 0.0f;
    simModel_.reset();
    simPursuit_.reset();
    simScaledPath_.clear();
    simTraceX_.clear();
    simTraceY_.clear();
    lastSimTime_     = 0.0;
    simEffectiveVehicleData_ = VehicleData{};
    simFeasibilityValid_     = false;
}

void VehicleControlUI::InitSimulation() {
    const VehicleData& vd  = vehicle_loader_.GetVehicle();
    const MapData&  map  = map_loader_.GetMap();
    const PathData& path = map_loader_.GetPath();

    // Maps are authored in metres, so plot units == simulation units for every
    // vehicle (simScale_ stays 1; it is kept as the single place to change if a
    // map ever needs a different unit).
    float real_half_w  = vd.wheel_tread * 0.5f + std::max(vd.left_overhang, vd.right_overhang);
    simScale_     = 1.0f;

    // Rear-axle-referenced body dimensions in map units
    simHalfFront_   = (vd.wheelbase + vd.front_overhang) * simScale_;  // rear-axle → front bumper
    simHalfRear_    = vd.rear_overhang * simScale_;                     // rear-axle → rear bumper
    simHalfWidth_   = real_half_w * simScale_;
    simWheelbaseMap_= vd.wheelbase * simScale_;

    // Tire visual dimensions in map units
    simTireHL_   = vd.wheel_radius * simScale_;
    simTireHW_   = vd.wheel_width  * 0.5f * simScale_;
    simTreadHW_  = vd.wheel_tread  * 0.5f * simScale_;

    // Vehicle state in REAL METERS (rear axle reference)
    simVehicleState_         = {};
    simVehicleState_.x       = map.start.x / simScale_;
    simVehicleState_.y       = map.start.y / simScale_;
    simVehicleState_.heading = (path.waypoints.size() >= 2)
        ? std::atan2(path.waypoints[1].y - path.waypoints[0].y,
                     path.waypoints[1].x - path.waypoints[0].x)
        : 0.0f;
    simVehicleState_.speed   = 0.0f;
    simLastDelta_            = 0.0f;
    simAccumulator_          = 0.0;
    simElapsed_              = 0.0f;
    simPrevAccel_            = 0.0f;
    simOutcome_              = sim::Outcome::Running;

    // Convert path waypoints to real meters
    simScaledPath_.clear();
    for (const auto& wp : path.waypoints)
        simScaledPath_.push_back({wp.x / simScale_, wp.y / simScale_});

    simPathLength_ = sim::PathLength(simScaledPath_);

    simTraceX_.clear();
    simTraceY_.clear();
    simTraceX_.push_back(map.start.x);
    simTraceY_.push_back(map.start.y);

    simDebugLog_.clear();
    simDebugDropped_      = 0;
    simDebugElapsedTime_  = 0.0f;
    simPrevSteeringAngle_ = 0.0f;  // steering starts centered before the first tick

    // Seed vehicle-model subsystem tuning sliders from the loaded VehicleData
    // (only when override is not enabled), then build simModel_ from them.
    if (!tuneTire_.enabled)
        tuneTire_.cornering_stiffness_coeff = vd.tire.enabled ? vd.tire.cornering_stiffness_coeff : 6.0f;

    if (!tuneSuspension_.enabled)
        tuneSuspension_.cog_height = std::max(tuning::kCogHeight.lo, std::min(CogHeightLimit(),
            (vd.cog_height > 0.0f) ? vd.cog_height : 0.5f * vd.vehicle_height));

    if (!tuneSteering_.enabled) {
        tuneSteering_.use_rate_limit          = vd.steering.enabled && vd.steering.use_rate_limit;
        tuneSteering_.max_steering_wheel_rate = vd.max_steering_wheel_rate;
    }

    RebuildVehicleModel();

    // Load JSON defaults into tuning structs (only when override is not enabled)
    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        if (!tuneAPP_.enabled) {
            auto fcfg = path_tracking::LoadAdaptivePurePursuitConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/adaptive_pure_pursuit.json");
            NotifyLimited("Config file adjusted: adaptive_pure_pursuit.json", fcfg.warnings);
            tuneAPP_.min_lookahead  = fcfg.min_lookahead;
            tuneAPP_.max_lookahead  = fcfg.max_lookahead;
            tuneAPP_.speed_gain     = fcfg.speed_gain;
            tuneAPP_.curvature_gain = fcfg.curvature_gain;
            tuneAPP_.max_speed      = fcfg.max_speed_mps;
            tuneAPP_.search_window  = fcfg.search_window;
        }
    } else if (selectedAlgorithm_ == "LQR") {
        if (!tuneLQR_.enabled) {
            auto fcfg = path_tracking::LoadLqrConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/lqr.json");
            NotifyLimited("Config file adjusted: lqr.json", fcfg.warnings);
            tuneLQR_.q0             = fcfg.q0;
            tuneLQR_.q1             = fcfg.q1;
            tuneLQR_.q2             = fcfg.q2;
            tuneLQR_.q3             = fcfg.q3;
            tuneLQR_.q4             = fcfg.q4;
            tuneLQR_.q5             = fcfg.q5;
            tuneLQR_.r_steering     = fcfg.r_steering;
            tuneLQR_.r_acceleration = fcfg.r_acceleration;
            tuneLQR_.r_scale        = fcfg.r_scale;
            tuneLQR_.max_speed      = fcfg.max_speed;
            tuneLQR_.dare_iterations = fcfg.dare_iterations;
            tuneLQR_.dare_threshold  = fcfg.dare_threshold;
            tuneLQR_.search_window   = fcfg.search_window;
        }
    } else if (selectedAlgorithm_ == "Stanley") {
        if (!tuneStanley_.enabled) {
            auto fcfg = path_tracking::LoadStanleyConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/stanley.json");
            NotifyLimited("Config file adjusted: stanley.json", fcfg.warnings);
            tuneStanley_.stanley_gain  = fcfg.stanley_gain;
            tuneStanley_.min_speed     = fcfg.min_speed;
            tuneStanley_.max_speed     = fcfg.max_speed_mps;
            tuneStanley_.max_delta     = fcfg.max_delta;
            tuneStanley_.search_window = fcfg.search_window;
        }
    } else if (selectedAlgorithm_ == "MPPI") {
        if (!tuneMPPI_.enabled) {
            auto fcfg = path_tracking::LoadMppiConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/mppi.json");
            NotifyLimited("Config file adjusted: mppi.json", fcfg.warnings);
            tuneMPPI_.horizon_T     = fcfg.horizon_T;
            tuneMPPI_.num_samples_K = fcfg.num_samples_K;
            tuneMPPI_.lambda        = fcfg.lambda;
            tuneMPPI_.sigma_steer   = fcfg.sigma_steer;
            tuneMPPI_.rollout_dt    = fcfg.rollout_dt;
            tuneMPPI_.max_speed     = fcfg.max_speed_mps;
            tuneMPPI_.w_lat         = fcfg.w_lat;
            tuneMPPI_.w_heading     = fcfg.w_heading;
            tuneMPPI_.search_window = fcfg.search_window;
            tuneMPPI_.max_delta     = fcfg.max_delta;
            tuneMPPI_.rng_seed      = fcfg.rng_seed;
        }
    } else if (selectedAlgorithm_ == "MPC") {
        if (!tuneMPC_.enabled) {
            auto fcfg = path_tracking::LoadMpcConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/mpc.json");
            NotifyLimited("Config file adjusted: mpc.json", fcfg.warnings);
            tuneMPC_.horizon_N     = fcfg.horizon_N;
            tuneMPC_.iterations    = fcfg.iterations;
            tuneMPC_.rollout_dt    = fcfg.rollout_dt;
            tuneMPC_.max_speed     = fcfg.max_speed_mps;
            tuneMPC_.w_lat         = fcfg.w_lat;
            tuneMPC_.w_heading     = fcfg.w_heading;
            tuneMPC_.w_control     = fcfg.w_control;
            tuneMPC_.w_rate        = fcfg.w_rate;
            tuneMPC_.search_window = fcfg.search_window;
            tuneMPC_.max_delta     = fcfg.max_delta;
        }
    } else {
        if (!tunePP_.enabled) {
            auto fcfg = path_tracking::LoadPurePursuitConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/pure_pursuit.json");
            NotifyLimited("Config file adjusted: pure_pursuit.json", fcfg.warnings);
            tunePP_.lookahead_distance = fcfg.lookahead_distance;
            tunePP_.lookahead_gain     = fcfg.lookahead_gain;
            tunePP_.max_speed          = fcfg.max_speed_mps;
            tunePP_.search_window      = fcfg.search_window;
        }
    }

    // JSON values (and earlier overrides) are clamped into the same ranges the
    // sliders allow, including this run's speed / steering caps.
    std::vector<std::string> limited;
    ApplyTuningLimits(limited);
    NotifyLimited("Parameters limited", limited);
    if (simFeasibilityValid_ && !simFeasibility_.geometrically_feasible) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "Tightest bend %.1f m, but %s cannot turn tighter than %.1f m.",
                      simFeasibility_.path_min_radius_m, vd.name.c_str(), simFeasibility_.vehicle_min_radius_m);
        ShowNotification("Path too tight for this vehicle", buf, 8000.0f);
    }

    RebuildAlgorithm();
}

void VehicleControlUI::RebuildVehicleModel() {
    VehicleData vd = vehicle_loader_.GetVehicle();  // copy: overlay live tuning below,
                                                     // never mutate the loader's canonical data
    if (tuneTire_.enabled) {
        vd.tire.enabled = true;
        vd.tire.cornering_stiffness_coeff = tuneTire_.cornering_stiffness_coeff;
    }
    if (tuneSuspension_.enabled) {
        vd.suspension.enabled = true;
        vd.cog_height = tuneSuspension_.cog_height;
    }
    if (tuneSteering_.enabled) {
        vd.steering.enabled = true;
        vd.steering.use_rate_limit = tuneSteering_.use_rate_limit;
        vd.max_steering_wheel_rate = tuneSteering_.max_steering_wheel_rate;
    }

    vehicle::ModelFidelity fidelity =
        (selectedVehicleModelKind_ == "Dynamic (Linear Tire)")
            ? vehicle::ModelFidelity::DynamicLinearTire
            : vehicle::ModelFidelity::Kinematic;

    simEffectiveVehicleData_ = vd;  // captured for the debug CSV's config metadata header
    simFeasibility_      = path_tracking::AnalyzePathFeasibility(simScaledPath_, vd);
    simFeasibilityValid_ = !simScaledPath_.empty();

    // NOTE: only simModel_ is (re)built here. simVehicleState_ is deliberately
    // left untouched — a fidelity swap carries x/y/heading/speed over as-is,
    // which is safe in both directions (see plan notes): Kinematic always
    // writes lateral_speed=0 and a kinematic yaw_rate estimate every step, so
    // handing those to a freshly-swapped dynamic model is the physically
    // correct initial condition; going the other way, KinematicBicycleModel
    // never reads lateral_speed and only overwrites yaw_rate, so stale
    // dynamic-model state is simply inert on the next step.
    simModel_   = vehicle::MakeVehicleModel(fidelity, vd);
    modelDirty_ = false;
}

void VehicleControlUI::RebuildAlgorithm() {
    const VehicleData& vd = vehicle_loader_.GetVehicle();

    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        simMaxSpeed_ = tuneAPP_.max_speed;
        path_tracking::AdaptivePurePursuitConfig cfg;
        cfg.min_lookahead  = tuneAPP_.min_lookahead;
        cfg.max_lookahead  = tuneAPP_.max_lookahead;
        cfg.speed_gain     = tuneAPP_.speed_gain;
        cfg.curvature_gain = tuneAPP_.curvature_gain;
        cfg.wheelbase      = vd.wheelbase;
        cfg.search_window  = tuneAPP_.search_window;
        simPursuit_ = std::make_unique<path_tracking::AdaptivePurePursuit>(cfg);
    } else if (selectedAlgorithm_ == "LQR") {
        simMaxSpeed_ = tuneLQR_.max_speed;
        path_tracking::LqrConfig cfg;
        cfg.q0              = tuneLQR_.q0;
        cfg.q1              = tuneLQR_.q1;
        cfg.q2              = tuneLQR_.q2;
        cfg.q3              = tuneLQR_.q3;
        cfg.q4              = tuneLQR_.q4;
        cfg.q5              = tuneLQR_.q5;
        cfg.r_steering      = tuneLQR_.r_steering;
        cfg.r_acceleration  = tuneLQR_.r_acceleration;
        cfg.r_scale         = tuneLQR_.r_scale;
        cfg.time_step       = sim::kControlDt;   // the real control period of the loop
        cfg.max_delta       = MaxDeltaLimit();
        cfg.max_delta_rate  = ActuatorRateLimit();
        cfg.max_speed       = tuneLQR_.max_speed;
        cfg.dare_iterations = tuneLQR_.dare_iterations;
        cfg.dare_threshold  = tuneLQR_.dare_threshold;
        cfg.wheelbase       = vd.wheelbase;
        cfg.search_window   = tuneLQR_.search_window;
        simPursuit_ = std::make_unique<path_tracking::Lqr>(cfg);
    } else if (selectedAlgorithm_ == "Stanley") {
        simMaxSpeed_ = tuneStanley_.max_speed;
        path_tracking::StanleyConfig cfg;
        cfg.stanley_gain  = tuneStanley_.stanley_gain;
        cfg.min_speed     = tuneStanley_.min_speed;
        cfg.max_speed     = tuneStanley_.max_speed;
        cfg.max_delta     = std::min(tuneStanley_.max_delta, MaxDeltaLimit());
        cfg.wheelbase     = vd.wheelbase;
        cfg.search_window = tuneStanley_.search_window;
        simPursuit_ = std::make_unique<path_tracking::Stanley>(cfg);
    } else if (selectedAlgorithm_ == "MPPI") {
        simMaxSpeed_ = tuneMPPI_.max_speed;
        path_tracking::MppiConfig cfg;
        cfg.horizon_T     = tuneMPPI_.horizon_T;
        cfg.num_samples_K = tuneMPPI_.num_samples_K;
        cfg.lambda        = tuneMPPI_.lambda;
        cfg.sigma_steer   = tuneMPPI_.sigma_steer;
        cfg.rollout_dt    = tuneMPPI_.rollout_dt;
        cfg.max_speed     = tuneMPPI_.max_speed;
        cfg.w_lat         = tuneMPPI_.w_lat;
        cfg.w_heading     = tuneMPPI_.w_heading;
        cfg.wheelbase     = vd.wheelbase;
        cfg.search_window = tuneMPPI_.search_window;
        cfg.max_delta     = std::min(tuneMPPI_.max_delta, MaxDeltaLimit());
        cfg.rng_seed      = tuneMPPI_.rng_seed;
        cfg.control_dt    = sim::kControlDt;
        cfg.max_delta_rate = ActuatorRateLimit();
        simPursuit_ = std::make_unique<path_tracking::Mppi>(cfg);
    } else if (selectedAlgorithm_ == "MPC") {
        simMaxSpeed_ = tuneMPC_.max_speed;
        path_tracking::MpcConfig cfg;
        cfg.horizon_N     = tuneMPC_.horizon_N;
        cfg.iterations    = tuneMPC_.iterations;
        cfg.rollout_dt    = tuneMPC_.rollout_dt;
        cfg.max_speed     = tuneMPC_.max_speed;
        cfg.w_lat         = tuneMPC_.w_lat;
        cfg.w_heading     = tuneMPC_.w_heading;
        cfg.w_control     = tuneMPC_.w_control;
        cfg.w_rate        = tuneMPC_.w_rate;
        cfg.control_dt    = sim::kControlDt;
        cfg.wheelbase     = vd.wheelbase;
        cfg.search_window = tuneMPC_.search_window;
        cfg.max_delta      = std::min(tuneMPC_.max_delta, MaxDeltaLimit());
        cfg.max_delta_rate = ActuatorRateLimit();
        simPursuit_ = std::make_unique<path_tracking::Mpc>(cfg);
    } else {
        simMaxSpeed_ = tunePP_.max_speed;
        path_tracking::PurePursuitConfig cfg;
        cfg.lookahead_distance = tunePP_.lookahead_distance;
        cfg.lookahead_gain     = tunePP_.lookahead_gain;
        cfg.wheelbase          = vd.wheelbase;
        cfg.search_window      = tunePP_.search_window;
        simPursuit_ = std::make_unique<path_tracking::PurePursuit>(cfg);
    }
    simPursuit_->SetPath(simScaledPath_);
    paramsDirty_ = false;
}

void VehicleControlUI::TickSimulation(float dt) {
    if (simRunState_ != SIM_RUNNING || !simModel_ || !simPursuit_) return;

    if (modelDirty_) {
        RebuildVehicleModel();
        std::vector<std::string> limited;  // e.g. switching to Dynamic lowers the Max Speed cap
        if (ApplyTuningLimits(limited)) {
            paramsDirty_ = true;
            NotifyLimited("Parameters limited", limited);
        }
        // plan with the new model's actuator, keeping the controller's state
        simPursuit_->SetActuatorRate(ActuatorRateLimit());
    }
    if (paramsDirty_) RebuildAlgorithm();

    const auto tickStart = std::chrono::steady_clock::now();
    float delta = simPursuit_->ComputeSteering(simVehicleState_);
    simLastDelta_ = delta;

    path_tracking::TrackingDiagnostics track_diag;
    bool track_diag_valid = simPursuit_->GetLastTrackingDiagnostics(track_diag);

    vehicle::VehicleControls ctrl;
    ctrl.steering_angle = delta;
    ctrl.acceleration   = sim::SpeedCommand(simMaxSpeed_, simVehicleState_.speed, simPrevAccel_, ActiveVehicleData(), dt);
    simPrevAccel_       = ctrl.acceleration;

    simVehicleState_ = simModel_->Step(simVehicleState_, ctrl, dt);
    simElapsed_     += dt;
    const double tickMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tickStart).count();
    perfTickMs_ = (perfTickMs_ <= 0.0) ? tickMs : 0.95 * perfTickMs_ + 0.05 * tickMs;

    // Debug telemetry: one sample every tick (dense; simTraceX_/Y_ below is
    // sparse and plot-only). steering_rate is the actuator rate implied by
    // the commanded steering angle, independent of whatever rate limit the
    // active vehicle model may or may not be enforcing internally.
    simDebugElapsedTime_ += dt;
    const float kRadToDeg = 180.0f / 3.14159265f;
    float steering_rate = (ctrl.steering_angle - simPrevSteeringAngle_) / dt;
    simPrevSteeringAngle_ = ctrl.steering_angle;
    Point2D lookahead = simPursuit_->GetLookaheadPoint();

    vehicle::TireDiagnostics diag;
    bool diag_valid = simModel_->GetLastTireDiagnostics(diag);
    float sideslip_deg = std::atan2(simVehicleState_.lateral_speed, simVehicleState_.speed) * kRadToDeg;

    simDebugLog_.push_back(DebugSample{
        simDebugElapsedTime_,
        simVehicleState_.x, simVehicleState_.y,
        simVehicleState_.heading * kRadToDeg,
        simVehicleState_.speed,
        simVehicleState_.lateral_speed,
        simVehicleState_.yaw_rate,
        ctrl.steering_angle * kRadToDeg,
        steering_rate * kRadToDeg,
        ctrl.acceleration,
        lookahead.x, lookahead.y,
        sideslip_deg,
        diag_valid,
        diag_valid ? diag.alpha_front * kRadToDeg : 0.0f,
        diag_valid ? diag.alpha_rear  * kRadToDeg : 0.0f,
        diag_valid ? diag.fy_front : 0.0f,
        diag_valid ? diag.fy_rear  : 0.0f,
        simVehicleState_.steering_angle * kRadToDeg,
        diag_valid && diag.front_saturated,
        diag_valid && diag.rear_saturated,
        track_diag_valid,
        track_diag_valid ? track_diag.cross_track_error_m : 0.0f,
        track_diag_valid ? track_diag.heading_error_deg   : 0.0f,
        track_diag_valid ? track_diag.nearest_idx         : -1,
        track_diag_valid ? track_diag.last_cost           : 0.0f,
        track_diag_valid && track_diag.steering_at_clamp
    });
    if (simDebugLog_.size() > kMaxDebugSamples) { simDebugLog_.pop_front(); ++simDebugDropped_; }

    // Run ends on the goal (radius or goal line), a collision, leaving the map
    // or a timeout (sim::CheckTermination, shared with the test harness).
    simOutcome_ = sim::CheckTermination(simVehicleState_, ActiveVehicleData(), map_loader_.GetMap(),
                                        simScaledPath_, simPathLength_, simElapsed_, simMaxSpeed_);
    if (simOutcome_ == sim::Outcome::Goal)         simRunState_ = SIM_DONE;
    else if (simOutcome_ != sim::Outcome::Running) simRunState_ = SIM_FAILED;

    // Record trace dot every 0.5 m of travel
    float nx = simVehicleState_.x * simScale_;
    float ny = simVehicleState_.y * simScale_;
    if (simTraceX_.empty()) {
        simTraceX_.push_back(nx);
        simTraceY_.push_back(ny);
    } else {
        float ldx = nx - simTraceX_.back();
        float ldy = ny - simTraceY_.back();
        if (ldx*ldx + ldy*ldy >= 0.25f) {
            simTraceX_.push_back(nx);
            simTraceY_.push_back(ny);
            if (simTraceX_.size() > kMaxTracePoints) {  // drop the oldest chunk, keep the recent trail
                simTraceX_.erase(simTraceX_.begin(), simTraceX_.begin() + 1000);
                simTraceY_.erase(simTraceY_.begin(), simTraceY_.begin() + 1000);
            }
        }
    }
}

void VehicleControlUI::SaveDebugLogCsv() {
    if (simDebugLog_.empty()) {
        ShowNotification("Debug Log", "No debug samples recorded yet.", 3000.0f);
        return;
    }

    namespace fs = std::filesystem;
    fs::create_directories(DEBUG_LOG_PATH);

    auto sanitize = [](std::string s) {
        std::replace(s.begin(), s.end(), ' ', '_');
        return s;
    };

    std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));

    std::string filename = std::string(DEBUG_LOG_PATH) + "/debug_" +
                            sanitize(selectedAlgorithm_) + "_" +
                            sanitize(vehicle_loader_.GetVehicle().name) + "_" +
                            sanitize(selectedVehicleModelKind_) + "_" +
                            stamp + ".csv";

    std::ofstream out(filename);
    if (!out.is_open()) {
        ShowNotification("Debug Log", "Failed to write CSV file.", 3000.0f);
        return;
    }

    // Config metadata header: the vehicle-model and control-algorithm params
    // that actually produced this run, so the CSV is self-documenting and
    // never needs cross-referencing a JSON file by filename guess. Parseable
    // as comments by pandas.read_csv(..., comment='#') and most CSV tools.
    const VehicleData& evd = simEffectiveVehicleData_;
    out << "# vehicle_name=" << evd.name << '\n';
    out << "# vehicle_model=" << selectedVehicleModelKind_ << '\n';
    out << "# vehicle_mass_kg=" << evd.mass << '\n';
    out << "# vehicle_a_m=" << evd.a << '\n';
    out << "# vehicle_b_m=" << evd.b << '\n';
    out << "# vehicle_wheelbase_m=" << evd.wheelbase << '\n';
    out << "# vehicle_yaw_inertia=" << evd.yaw_inertia << '\n';
    out << "# vehicle_CA=" << evd.CA << '\n';
    out << "# tire_enabled=" << evd.tire.enabled << '\n';
    out << "# tire_cornering_stiffness_coeff=" << evd.tire.cornering_stiffness_coeff << '\n';
    out << "# tire_friction_coefficient=" << evd.tire.friction_coefficient << '\n';
    out << "# suspension_enabled=" << evd.suspension.enabled << '\n';
    out << "# cog_height_m=" << evd.cog_height << '\n';
    out << "# steering_enabled=" << evd.steering.enabled << '\n';
    out << "# steering_use_rate_limit=" << evd.steering.use_rate_limit << '\n';
    out << "# max_steering_wheel_rate_degps=" << evd.max_steering_wheel_rate << '\n';
    out << "# max_steering_wheel_angle_deg=" << evd.max_steering_wheel_angle << '\n';
    out << "# steering_wheel_to_tire_angle_ratio=" << evd.steering_wheel_to_tire_angle_ratio << '\n';
    out << "# map=" << map_loader_.GetMap().name << '\n';
    out << "# algorithm=" << selectedAlgorithm_ << '\n';
    out << "# debug_samples_dropped=" << simDebugDropped_ << '\n';
    out << "# run_outcome=" << sim::OutcomeName(simOutcome_) << '\n';
    if (simFeasibilityValid_) {
        out << "# path_min_radius_m=" << simFeasibility_.path_min_radius_m << '\n';
        out << "# path_speed_cap_mps=" << simFeasibility_.speed_cap_mps << '\n';
    }

    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        out << "# app_min_lookahead=" << tuneAPP_.min_lookahead << '\n';
        out << "# app_max_lookahead=" << tuneAPP_.max_lookahead << '\n';
        out << "# app_speed_gain=" << tuneAPP_.speed_gain << '\n';
        out << "# app_curvature_gain=" << tuneAPP_.curvature_gain << '\n';
        out << "# app_max_speed=" << tuneAPP_.max_speed << '\n';
        out << "# app_search_window=" << tuneAPP_.search_window << '\n';
    } else if (selectedAlgorithm_ == "LQR") {
        out << "# lqr_q0=" << tuneLQR_.q0 << '\n';
        out << "# lqr_q1=" << tuneLQR_.q1 << '\n';
        out << "# lqr_q2=" << tuneLQR_.q2 << '\n';
        out << "# lqr_q3=" << tuneLQR_.q3 << '\n';
        out << "# lqr_q4=" << tuneLQR_.q4 << '\n';
        out << "# lqr_q5=" << tuneLQR_.q5 << '\n';
        out << "# lqr_r_steering=" << tuneLQR_.r_steering << '\n';
        out << "# lqr_r_acceleration=" << tuneLQR_.r_acceleration << '\n';
        out << "# lqr_r_scale=" << tuneLQR_.r_scale << '\n';
        out << "# lqr_max_speed=" << tuneLQR_.max_speed << '\n';
        out << "# lqr_dare_iterations=" << tuneLQR_.dare_iterations << '\n';
        out << "# lqr_dare_threshold=" << tuneLQR_.dare_threshold << '\n';
        out << "# lqr_search_window=" << tuneLQR_.search_window << '\n';
    } else if (selectedAlgorithm_ == "Stanley") {
        out << "# stanley_gain=" << tuneStanley_.stanley_gain << '\n';
        out << "# stanley_min_speed=" << tuneStanley_.min_speed << '\n';
        out << "# stanley_max_speed=" << tuneStanley_.max_speed << '\n';
        out << "# stanley_max_delta_rad=" << std::min(tuneStanley_.max_delta, vehicle::MaxTireAngle(evd)) << '\n';
        out << "# stanley_search_window=" << tuneStanley_.search_window << '\n';
    } else if (selectedAlgorithm_ == "MPPI") {
        out << "# mppi_horizon_T=" << tuneMPPI_.horizon_T << '\n';
        out << "# mppi_num_samples_K=" << tuneMPPI_.num_samples_K << '\n';
        out << "# mppi_lambda=" << tuneMPPI_.lambda << '\n';
        out << "# mppi_sigma_steer=" << tuneMPPI_.sigma_steer << '\n';
        out << "# mppi_rollout_dt=" << tuneMPPI_.rollout_dt << '\n';
        out << "# mppi_max_speed=" << tuneMPPI_.max_speed << '\n';
        out << "# mppi_w_lat=" << tuneMPPI_.w_lat << '\n';
        out << "# mppi_w_heading=" << tuneMPPI_.w_heading << '\n';
        out << "# mppi_search_window=" << tuneMPPI_.search_window << '\n';
        out << "# mppi_max_delta_rad=" << std::min(tuneMPPI_.max_delta, vehicle::MaxTireAngle(evd)) << '\n';
        out << "# mppi_rng_seed=" << tuneMPPI_.rng_seed << '\n';
    } else if (selectedAlgorithm_ == "MPC") {
        out << "# mpc_horizon_N=" << tuneMPC_.horizon_N << '\n';
        out << "# mpc_iterations=" << tuneMPC_.iterations << '\n';
        out << "# mpc_rollout_dt=" << tuneMPC_.rollout_dt << '\n';
        out << "# mpc_max_speed=" << tuneMPC_.max_speed << '\n';
        out << "# mpc_w_lat=" << tuneMPC_.w_lat << '\n';
        out << "# mpc_w_heading=" << tuneMPC_.w_heading << '\n';
        out << "# mpc_w_control=" << tuneMPC_.w_control << '\n';
        out << "# mpc_w_rate=" << tuneMPC_.w_rate << '\n';
        out << "# mpc_search_window=" << tuneMPC_.search_window << '\n';
        out << "# mpc_max_delta_rad=" << std::min(tuneMPC_.max_delta, vehicle::MaxTireAngle(evd)) << '\n';
        out << "# mpc_max_delta_rate_radps=" << ActuatorRateLimit() << '\n';
    } else {
        out << "# pp_lookahead_distance=" << tunePP_.lookahead_distance << '\n';
        out << "# pp_lookahead_gain=" << tunePP_.lookahead_gain << '\n';
        out << "# pp_max_speed=" << tunePP_.max_speed << '\n';
        out << "# pp_search_window=" << tunePP_.search_window << '\n';
    }

    out << "time_s,x_m,y_m,heading_deg,speed_mps,lateral_speed_mps,yaw_rate_radps,"
           "steering_angle_deg,steering_rate_degps,acceleration_cmd_mps2,"
           "lookahead_x_m,lookahead_y_m,sideslip_deg,alpha_front_deg,alpha_rear_deg,"
           "fy_front_n,fy_rear_n,applied_steering_deg,front_saturated,rear_saturated,"
           "cross_track_error_m,heading_error_deg,nearest_path_idx,mpc_final_cost,"
           "steering_at_clamp\n";
    for (const auto& s : simDebugLog_) {
        out << s.time_s               << ',' << s.x_m                 << ','
            << s.y_m                  << ',' << s.heading_deg          << ','
            << s.speed_mps            << ',' << s.lateral_speed_mps    << ','
            << s.yaw_rate_radps       << ',' << s.steering_angle_deg   << ','
            << s.steering_rate_degps  << ',' << s.acceleration_cmd_mps2 << ','
            << s.lookahead_x_m        << ',' << s.lookahead_y_m        << ','
            << s.sideslip_deg         << ',';
        if (s.tire_diag_valid) {
            out << s.alpha_front_deg      << ',' << s.alpha_rear_deg       << ','
                << s.fy_front_n           << ',' << s.fy_rear_n            << ','
                << s.applied_steering_deg << ',' << s.front_saturated      << ','
                << s.rear_saturated       << ',';
        } else {
            out << ",,,,,,,";
        }
        if (s.tracking_diag_valid) {
            out << s.cross_track_error_m << ',' << s.heading_error_deg    << ','
                << s.nearest_path_idx    << ',' << s.mpc_final_cost       << ','
                << s.steering_at_clamp   << '\n';
        } else {
            out << ",,,,\n";
        }
    }
    out.close();

    ShowNotification("Debug Log Saved", filename, 4000.0f);
}

void VehicleControlUI::DrawVehicle(ImDrawList* dl) {
    // Rear-axle position in map units
    float rax = simVehicleState_.x * simScale_;
    float ray  = simVehicleState_.y * simScale_;
    float psi  = simVehicleState_.heading;

    float lx =  std::cos(psi), ly =  std::sin(psi);   // forward unit (vehicle)
    float wx = -std::sin(psi), wy =  std::cos(psi);   // left-lateral unit (vehicle)

    // Helper: plot-point from rear-axle origin with forward/lateral offsets in map units
    auto pp = [&](float fwd, float lat) -> ImVec2 {
        return ImPlot::PlotToPixels(rax + fwd*lx + lat*wx,
                                    ray + fwd*ly + lat*wy);
    };

    // --- Vehicle body rectangle (rear-axle referenced) ---
    ImVec2 FL = pp( simHalfFront_,  simHalfWidth_);
    ImVec2 FR = pp( simHalfFront_, -simHalfWidth_);
    ImVec2 RR = pp(-simHalfRear_,  -simHalfWidth_);
    ImVec2 RL = pp(-simHalfRear_,   simHalfWidth_);
    dl->AddQuadFilled(FL, FR, RR, RL, IM_COL32(30, 100, 255,  90));
    dl->AddQuad      (FL, FR, RR, RL, IM_COL32(60, 160, 255, 230), 2.0f);

    // --- Tires ---
    // Tire helper: draw one tire centered at (cx_m, cy_m) rotated by tire_psi
    auto drawTire = [&](float cx_m, float cy_m, float tire_psi) {
        float tlx =  std::cos(tire_psi), tly =  std::sin(tire_psi);
        float twx = -std::sin(tire_psi), twy =  std::cos(tire_psi);
        auto tc = [&](float f, float s) -> ImVec2 {
            return ImPlot::PlotToPixels(cx_m + f*tlx + s*twx,
                                        cy_m + f*tly + s*twy);
        };
        ImVec2 A = tc( simTireHL_,  simTireHW_);
        ImVec2 B = tc( simTireHL_, -simTireHW_);
        ImVec2 C = tc(-simTireHL_, -simTireHW_);
        ImVec2 D = tc(-simTireHL_,  simTireHW_);
        dl->AddQuadFilled(A, B, C, D, IM_COL32(20, 20, 20, 210));
        dl->AddQuad      (A, B, C, D, IM_COL32(80, 80, 80, 255), 1.0f);
    };

    // Rear tires (at rear axle, aligned with body heading)
    float rL_x = rax + simTreadHW_ * wx;
    float rL_y = ray + simTreadHW_ * wy;
    float rR_x = rax - simTreadHW_ * wx;
    float rR_y = ray - simTreadHW_ * wy;
    drawTire(rL_x, rL_y, psi);
    drawTire(rR_x, rR_y, psi);

    // Front tires (at front axle, rotated by steering angle delta)
    float fax = rax + simWheelbaseMap_ * lx;
    float fay = ray + simWheelbaseMap_ * ly;
    float fL_x = fax + simTreadHW_ * wx;
    float fL_y = fay + simTreadHW_ * wy;
    float fR_x = fax - simTreadHW_ * wx;
    float fR_y = fay - simTreadHW_ * wy;
    drawTire(fL_x, fL_y, psi + simVehicleState_.steering_angle);   // the wheel angle actually applied
    drawTire(fR_x, fR_y, psi + simVehicleState_.steering_angle);

    // --- Lookahead target dot + line from rear axle ---
    Point2D lh   = simPursuit_->GetLookaheadPoint();
    ImVec2 lhPx  = ImPlot::PlotToPixels(lh.x * simScale_, lh.y * simScale_);
    ImVec2 raPx  = ImPlot::PlotToPixels(rax, ray);
    dl->AddLine        (raPx, lhPx, IM_COL32(100, 180, 255, 160), 1.5f);
    dl->AddCircleFilled(lhPx, 6.0f, IM_COL32(255, 220, 50, 220));
    dl->AddCircle      (lhPx, 6.0f, IM_COL32(255, 255, 255, 180), 12, 1.5f);
}

void VehicleControlUI::RenderVehicleModelTuning(float availableHeight) {
    float startY = ImGui::GetCursorPosY();

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Vehicle Model");

    std::string modelBtnLabel = "Model: " + selectedVehicleModelKind_;
    if (ImGui::Button(modelBtnLabel.c_str(), ImVec2(-1.0f, 0.0f))) {
        ImGui::OpenPopup("Select Vehicle Model##popup");
    }

    // Explicitly centered on the viewport: this button lives low in the
    // right-hand panel, and an unpositioned popup opens anchored near its
    // invoking widget — near the bottom-right corner here — which pushed it
    // partly off-screen. The three main-screen popups don't need this
    // because their buttons already sit near screen center.
    ImVec2 viewportCenter = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(viewportCenter, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_Always);
    if (ImGui::BeginPopupModal("Select Vehicle Model##popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Available Vehicle Models");
        ImGui::Separator();
        ImGui::BeginChild("##modellist", ImVec2(300, 60), true);
        for (const auto& name : kAvailableVehicleModels) {
            if (ImGui::Selectable(name.c_str())) {
                selectedVehicleModelKind_ = name;
                modelDirty_ = true;   // NOT ResetSimulation(): fidelity swaps mid-run by design
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndChild();
        ImGui::Spacing();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Tire/Suspension/Steering sub-sections live in their own scrollable child
    // region so they can never bleed past availableHeight into the algorithm
    // tuning panel below — they scroll instead of overlapping regardless of
    // exactly how many subsystem sections get added here in the future.
    float headerH   = ImGui::GetCursorPosY() - startY;
    float childH    = std::max(availableHeight - headerH, 60.0f);
    ImGui::BeginChild("##vehicleModelSubsystems", ImVec2(0, childH), false);
    ImGui::PushItemWidth(-1.0f);

    auto sliderF = [&](const char* label, float* v, float mn, float mx,
                       float step, const char* fmt, const char* desc, bool* enabledFlag) {
        ImGui::SliderFloat(label, v, mn, mx, fmt);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            *v = std::round(*v / step) * step;
            *v = std::max(mn, std::min(mx, *v));
            modelDirty_ = true;
        }
        if (*enabledFlag && ImGui::IsItemHovered()) {
            tooltipState_.description   = desc;
            tooltipState_.sliderMin     = ImGui::GetItemRectMin();
            tooltipState_.sliderMax     = ImGui::GetItemRectMax();
            tooltipState_.lastHoverTime = glfwGetTime();
        }
    };

    // ---- Tire ----
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Tire (Linear Model)");
    if (ImGui::Checkbox("Override Tire Defaults", &tuneTire_.enabled)) {
        modelDirty_ = true;   // on: apply the slider value; off: back to the vehicle file
    }
    if (!tuneTire_.enabled) ImGui::BeginDisabled();
    sliderF("Cornering Stiffness", &tuneTire_.cornering_stiffness_coeff, tuning::kCorneringStiffness.lo, tuning::kCorneringStiffness.hi, tuning::kCorneringStiffness.step, "%.1f",
        "Linear tire cornering-stiffness multiplier: Fy = -coeff * Fz * slip_angle. "
        "~6.0 is a typical loaded-axle heuristic; higher = stiffer and more responsive "
        "but saturates more abruptly near the friction limit.",
        &tuneTire_.enabled);
    if (!tuneTire_.enabled) ImGui::EndDisabled();

    // ---- Suspension ----
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Suspension (Load Transfer)");
    if (ImGui::Checkbox("Override Suspension Defaults", &tuneSuspension_.enabled)) {
        modelDirty_ = true;   // on: apply the slider value; off: back to the vehicle file
    }
    if (!tuneSuspension_.enabled) ImGui::BeginDisabled();
    sliderF("CoG Height", &tuneSuspension_.cog_height, tuning::kCogHeight.lo, CogHeightLimit(), tuning::kCogHeight.step, "%.2f m",
        "Center-of-gravity height above ground (h). Feeds the rigid suspension "
        "model's dynamic longitudinal load-transfer term. Higher h means more "
        "load transfer under acceleration/braking. Limited to 70% of the vehicle height.",
        &tuneSuspension_.enabled);
    if (!tuneSuspension_.enabled) ImGui::EndDisabled();

    // ---- Steering ----
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Steering Actuator");
    if (ImGui::Checkbox("Override Steering Defaults", &tuneSteering_.enabled)) {
        modelDirty_ = true;   // on: apply the slider value; off: back to the vehicle file
    }
    if (!tuneSteering_.enabled) ImGui::BeginDisabled();
    if (ImGui::Checkbox("Rate-Limited Steering", &tuneSteering_.use_rate_limit)) {
        modelDirty_ = true;
    }
    sliderF("Max Steering Rate", &tuneSteering_.max_steering_wheel_rate, tuning::kSteeringWheelRate.lo, tuning::kSteeringWheelRate.hi, tuning::kSteeringWheelRate.step, "%.0f deg/s",
        "Steering-wheel angular-rate cap, converted to a tire-angle rate via "
        "steering_wheel_to_tire_angle_ratio. Only takes effect while "
        "'Rate-Limited Steering' is checked.",
        &tuneSteering_.enabled);
    if (!tuneSteering_.enabled) ImGui::EndDisabled();

    ImGui::PopItemWidth();
    ImGui::EndChild();  // ##vehicleModelSubsystems
}

void VehicleControlUI::RenderParamTuning(float availableHeight) {
    // Resolve active tuning enabled flag
    bool* enabled = nullptr;
    if      (selectedAlgorithm_ == "Adaptive Pure Pursuit") enabled = &tuneAPP_.enabled;
    else if (selectedAlgorithm_ == "LQR")                   enabled = &tuneLQR_.enabled;
    else if (selectedAlgorithm_ == "Stanley")               enabled = &tuneStanley_.enabled;
    else if (selectedAlgorithm_ == "MPPI")                  enabled = &tuneMPPI_.enabled;
    else if (selectedAlgorithm_ == "MPC")                   enabled = &tuneMPC_.enabled;
    else                                                     enabled = &tunePP_.enabled;

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Parameter Tuning");

    ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(0.2f, 0.9f, 0.4f, 1.0f));
    if (ImGui::Checkbox("Override Defaults", enabled)) {
        if (*enabled) paramsDirty_ = true;
    }
    ImGui::PopStyleColor();

    const bool speedCapped = IsDynamicModel() && simFeasibilityValid_ && MaxSpeedLimit() < tuning::kMaxSpeed.hi;
    if (speedCapped)
        ImGui::TextDisabled("Max Speed limited to %.1f m/s on this map (Dynamic)", MaxSpeedLimit());

    // Slider count for active algorithm
    int N = 4;
    if      (selectedAlgorithm_ == "Adaptive Pure Pursuit") N = 6;
    else if (selectedAlgorithm_ == "LQR")                   N = 13;
    else if (selectedAlgorithm_ == "Stanley")               N = 5;
    else if (selectedAlgorithm_ == "MPC")                   N = 10;
    else if (selectedAlgorithm_ == "MPPI")                  N = 11;

    float headerH = 1.0f
                  + ImGui::GetTextLineHeightWithSpacing()
                  + ImGui::GetFrameHeightWithSpacing()
                  + ImGui::GetStyle().ItemSpacing.y
                  + (speedCapped ? ImGui::GetTextLineHeightWithSpacing() : 0.0f);
    float sliderAreaH = availableHeight - headerH;

    float spacing = ImGui::GetStyle().ItemSpacing.y;
    float perH    = (sliderAreaH - spacing * static_cast<float>(N - 1)) / static_cast<float>(N);
    float minH    = ImGui::GetTextLineHeight() + 4.0f;
    perH = std::max(perH, minH);
    float padY = std::max(2.0f, (perH - ImGui::GetTextLineHeight()) * 0.5f);

    // Bounding/scrollable child: the per-slider height above is a best-effort
    // shrink-to-fit, but when an algorithm has many sliders (e.g. LQR's 13)
    // and little space is available, perH still clamps to a readable minimum
    // and the sliders no longer fit. Without this child they would silently
    // overflow past sliderAreaH into whatever renders next; with it, they
    // scroll instead of bleeding into a sibling panel.
    ImGui::BeginChild("##paramTuningSliders", ImVec2(0, sliderAreaH), false);

    if (!*enabled) ImGui::BeginDisabled();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                        ImVec2(ImGui::GetStyle().FramePadding.x, padY));
    ImGui::PushItemWidth(-1.0f);

    // Apply value + snap only on mouse release; track hover for tooltip
    auto sliderF = [&](const char* label, float* v, float mn, float mx,
                       float step, const char* fmt, const char* desc) {
        ImGui::SliderFloat(label, v, mn, mx, fmt);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            *v = std::round(*v / step) * step;
            *v = std::max(mn, std::min(mx, *v));
            paramsDirty_ = true;
        }
        if (*enabled && ImGui::IsItemHovered()) {
            tooltipState_.description   = desc;
            tooltipState_.sliderMin     = ImGui::GetItemRectMin();
            tooltipState_.sliderMax     = ImGui::GetItemRectMax();
            tooltipState_.lastHoverTime = glfwGetTime();
        }
    };
    auto sliderI = [&](const char* label, int* v, int mn, int mx,
                       int step, const char* desc) {
        ImGui::SliderInt(label, v, mn, mx);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            *v = ((*v - mn + step / 2) / step) * step + mn;
            *v = std::max(mn, std::min(mx, *v));
            paramsDirty_ = true;
        }
        if (*enabled && ImGui::IsItemHovered()) {
            tooltipState_.description   = desc;
            tooltipState_.sliderMin     = ImGui::GetItemRectMin();
            tooltipState_.sliderMax     = ImGui::GetItemRectMax();
            tooltipState_.lastHoverTime = glfwGetTime();
        }
    };

    using namespace tuning;
    const float vMax = MaxSpeedLimit();   // Dynamic fidelity: this path's speed cap
    const float dMax = MaxDeltaLimit();   // vehicle's real max tire angle
    const char* speedDesc = "Target cruise speed the controller accelerates towards. Under the Dynamic model "
                            "the upper limit is this path's speed cap (curvature, tire friction, steering rate).";
    const char* deltaDesc = "Hard clamp on the steering output angle in radians. Limited to the vehicle's "
                            "real maximum tire angle.";

    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        sliderF("Min Lookahead",   &tuneAPP_.min_lookahead,  kAPPMinLookahead.lo,  kAPPMinLookahead.hi,  kAPPMinLookahead.step, "%.1f m",
            "Floor for lookahead distance. Target point never closer than this.");
        sliderF("Max Lookahead",   &tuneAPP_.max_lookahead,  kAPPMaxLookahead.lo,  kAPPMaxLookahead.hi,  kAPPMaxLookahead.step, "%.1f m",
            "Ceiling for lookahead distance. Target point never farther than this.");
        sliderF("Speed Gain",      &tuneAPP_.speed_gain,     kAPPSpeedGain.lo,     kAPPSpeedGain.hi,     kAPPSpeedGain.step,    "%.1f",
            "How strongly current speed scales the lookahead. Higher = farther look at speed.");
        sliderF("Curvature Gain",  &tuneAPP_.curvature_gain, kAPPCurvatureGain.lo, kAPPCurvatureGain.hi, kAPPCurvatureGain.step, "%.1f",
            "Lookahead penalty on path curvature. Higher = shorter look on tight corners.");
        sliderF("Max Speed",       &tuneAPP_.max_speed,      kMaxSpeed.lo, vMax, kMaxSpeed.step, "%.1f m/s", speedDesc);
        sliderI("Search Window",   &tuneAPP_.search_window,  kSearchWindow.lo, kSearchWindow.hi, kSearchWindow.step,
            "Waypoints scanned ahead when searching for the lookahead target point.");

    } else if (selectedAlgorithm_ == "LQR") {
        sliderF("Q0",              &tuneLQR_.q0,             kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Cost weight on lateral (cross-track) position error.");
        sliderF("Q1",              &tuneLQR_.q1,             kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Cost weight on lateral error rate (derivative).");
        sliderF("Q2",              &tuneLQR_.q2,             kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Cost weight on yaw (heading) error. Higher = faster alignment.");
        sliderF("Q3",              &tuneLQR_.q3,             kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Cost weight on yaw rate (derivative of heading).");
        sliderF("Q4",              &tuneLQR_.q4,             kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Cost weight on speed error from target speed.");
        sliderF("Q5",              &tuneLQR_.q5,             kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Cost weight on the wheel angle's deviation from the path's feed-forward angle.");
        sliderF("R Steering",      &tuneLQR_.r_steering,     kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Penalty on the steering rate. Higher = smoother, slower steering.");
        sliderF("R Acceleration",  &tuneLQR_.r_acceleration, kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Penalty on acceleration control effort.");
        sliderF("R Scale",         &tuneLQR_.r_scale,        kLQRWeight.lo, kLQRWeight.hi, kLQRWeight.step, "%.1f",
            "Global multiplier applied to both R weights (r_steering and r_acceleration).");
        sliderF("Max Speed",       &tuneLQR_.max_speed,      kMaxSpeed.lo, vMax, kMaxSpeed.step, "%.1f m/s", speedDesc);
        sliderI("DARE Iterations", &tuneLQR_.dare_iterations, kLQRDareIterations.lo, kLQRDareIterations.hi, kLQRDareIterations.step,
            "Maximum iterations allowed for the DARE (Riccati equation) solver.");
        sliderF("DARE Threshold",  &tuneLQR_.dare_threshold, kLQRDareThreshold.lo, kLQRDareThreshold.hi, kLQRDareThreshold.step, "%.2f",
            "Convergence tolerance for the DARE solver. Smaller = more precise K matrix.");
        sliderI("Search Window",   &tuneLQR_.search_window,  kSearchWindow.lo, kSearchWindow.hi, kSearchWindow.step,
            "Waypoints scanned ahead to find the closest reference path point.");

    } else if (selectedAlgorithm_ == "Stanley") {
        sliderF("Stanley Gain",    &tuneStanley_.stanley_gain, kStanleyGain.lo, kStanleyGain.hi, kStanleyGain.step, "%.1f",
            "Cross-track error gain k. Higher = more aggressive lateral correction.");
        sliderF("Min Speed",       &tuneStanley_.min_speed,  kStanleyMinSpeed.lo, kStanleyMinSpeed.hi, kStanleyMinSpeed.step, "%.1f m/s",
            "Speed floor used in the Stanley formula denominator to prevent large outputs at low speed.");
        sliderF("Max Speed",       &tuneStanley_.max_speed,  kMaxSpeed.lo, vMax, kMaxSpeed.step, "%.1f m/s", speedDesc);
        sliderF("Max Delta",       &tuneStanley_.max_delta,  kMaxDelta.lo, dMax, kMaxDelta.step, "%.2f rad", deltaDesc);
        sliderI("Search Window",   &tuneStanley_.search_window, kSearchWindow.lo, kSearchWindow.hi, kSearchWindow.step,
            "Waypoints scanned ahead to find the closest front-axle reference point.");

    } else if (selectedAlgorithm_ == "MPPI") {
        sliderI("Horizon T",       &tuneMPPI_.horizon_T,     kMPPIHorizon.lo, kMPPIHorizon.hi, kMPPIHorizon.step,
            "Number of time steps in each rollout trajectory.");
        sliderI("Num Samples K",   &tuneMPPI_.num_samples_K, kMPPISamples.lo, kMPPISamples.hi, kMPPISamples.step,
            "Random trajectories sampled per control step. More = better quality, slower.");
        sliderF("Lambda",          &tuneMPPI_.lambda,        kMPPILambda.lo, kMPPILambda.hi, kMPPILambda.step, "%.1f",
            "Temperature parameter. Lower concentrates weight on the best-cost samples.");
        sliderF("Sigma Steer",     &tuneMPPI_.sigma_steer,   kMPPISigma.lo, kMPPISigma.hi, kMPPISigma.step, "%.2f rad",
            "Standard deviation of the steering noise added to each sample trajectory.");
        sliderF("Rollout dt",      &tuneMPPI_.rollout_dt,    kMPPIRolloutDt.lo, kMPPIRolloutDt.hi, kMPPIRolloutDt.step, "%.2f s",
            "Simulation time step used inside each rollout.");
        sliderF("Max Speed",       &tuneMPPI_.max_speed,     kMaxSpeed.lo, vMax, kMaxSpeed.step, "%.1f m/s", speedDesc);
        sliderF("W Lateral",       &tuneMPPI_.w_lat,         kMPPIWeight.lo, kMPPIWeight.hi, kMPPIWeight.step, "%.1f",
            "Cost weight on lateral (cross-track) error in trajectory evaluation.");
        sliderF("W Heading",       &tuneMPPI_.w_heading,     kMPPIWeight.lo, kMPPIWeight.hi, kMPPIWeight.step, "%.1f",
            "Cost weight on heading (yaw) error in trajectory evaluation.");
        sliderI("Search Window",   &tuneMPPI_.search_window, kSearchWindow.lo, kSearchWindow.hi, kSearchWindow.step,
            "Waypoints scanned ahead during rollout cost evaluation.");
        sliderF("Max Delta",       &tuneMPPI_.max_delta,     kMaxDelta.lo, dMax, kMaxDelta.step, "%.2f rad", deltaDesc);
        sliderI("RNG Seed",        &tuneMPPI_.rng_seed,      kMPPISeed.lo, kMPPISeed.hi, kMPPISeed.step,
            "Random seed for noise generation. Change for different sample patterns.");

    } else if (selectedAlgorithm_ == "MPC") {
        sliderI("Horizon N",       &tuneMPC_.horizon_N,      kMPCHorizon.lo, kMPCHorizon.hi, kMPCHorizon.step,
            "Number of prediction steps in the MPC horizon.");
        sliderI("Iterations",      &tuneMPC_.iterations,     kMPCIterations.lo, kMPCIterations.hi, kMPCIterations.step,
            "QP solver iterations per control step. More = closer to the optimum, slower.");
        sliderF("Rollout dt",      &tuneMPC_.rollout_dt,     kMPCRolloutDt.lo, kMPCRolloutDt.hi, kMPCRolloutDt.step, "%.2f s",
            "Simulation time step used inside each prediction rollout.");
        sliderF("Max Speed",       &tuneMPC_.max_speed,      kMaxSpeed.lo, vMax, kMaxSpeed.step, "%.1f m/s", speedDesc);
        sliderF("W Lateral",       &tuneMPC_.w_lat,          kMPCWeight.lo, kMPCWeight.hi, kMPCWeight.step, "%.1f",
            "Cost weight on lateral (cross-track) error over the horizon.");
        sliderF("W Heading",       &tuneMPC_.w_heading,      kMPCWeight.lo, kMPCWeight.hi, kMPCWeight.step, "%.1f",
            "Cost weight on heading (yaw) error over the horizon.");
        sliderF("W Control",       &tuneMPC_.w_control,      kMPCWControl.lo, kMPCWControl.hi, kMPCWControl.step, "%.2f",
            "Penalty on the wheel angle's deviation from the path's feed-forward angle.");
        sliderF("W Rate",          &tuneMPC_.w_rate,         kMPCWRate.lo, kMPCWRate.hi, kMPCWRate.step, "%.2f",
            "Penalty on steering rate. Higher = smoother, slower steering.");
        sliderI("Search Window",   &tuneMPC_.search_window,  kSearchWindow.lo, kSearchWindow.hi, kSearchWindow.step,
            "Waypoints scanned ahead during rollout cost evaluation.");
        sliderF("Max Delta",       &tuneMPC_.max_delta,      kMaxDelta.lo, dMax, kMaxDelta.step, "%.2f rad", deltaDesc);

    } else { // Pure Pursuit
        sliderF("Lookahead Dist",  &tunePP_.lookahead_distance, kPPLookaheadDist.lo, kPPLookaheadDist.hi, kPPLookaheadDist.step, "%.1f m",
            "Base lookahead distance. Added to the speed-scaled term: Ld = base + gain * speed.");
        sliderF("Lookahead Gain",  &tunePP_.lookahead_gain,     kPPLookaheadGain.lo, kPPLookaheadGain.hi, kPPLookaheadGain.step, "%.1f",
            "Speed multiplier for lookahead. Higher = farther look at higher speeds.");
        sliderF("Max Speed",       &tunePP_.max_speed,          kMaxSpeed.lo, vMax, kMaxSpeed.step, "%.1f m/s", speedDesc);
        sliderI("Search Window",   &tunePP_.search_window,      kSearchWindow.lo, kSearchWindow.hi, kSearchWindow.step,
            "Waypoints scanned ahead when searching for the lookahead target point.");
    }

    ImGui::PopItemWidth();
    ImGui::PopStyleVar();
    if (!*enabled) ImGui::EndDisabled();
    ImGui::EndChild();  // ##paramTuningSliders
}

void VehicleControlUI::RenderSimulationScreen() {
    ImGui::Spacing();

    ImGuiIO& io = ImGui::GetIO();
    float screenWidth  = io.DisplaySize.x;
    float screenHeight = io.DisplaySize.y;

    float frameWidth1  = screenWidth  * 0.99f;
    float frameHeight1 = screenHeight * 0.06f;

    float frameWidth2  = screenWidth  * 0.72f;
    float frameHeight2 = screenHeight * 0.88f;

    float frameWidth3  = screenWidth  * 0.25f;
    float frameHeight3 = screenHeight * 0.88f;

    // Header bar with back button
    ImGui::BeginChild("FrameArea", ImVec2(frameWidth1, frameHeight1), false, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::PushFont(iconFont2);
        if (ImGui::Button("U", ImVec2(50.0f, 35.0f))) {
            ResetSimulation();
            currentScreen = MAIN_SCREEN;
        }
        ImGui::PopFont();
    }
    ImGui::EndChild();

    // Fixed-step simulation: wall time accumulates and the controller and
    // vehicle model always advance in sim::kControlDt steps, so results do not
    // depend on the frame rate (at most 15 steps per frame to stay responsive
    // after a stall; at most 0.25 s of backlog is kept).
    if (simRunState_ == SIM_RUNNING) {
        double now = glfwGetTime();
        simAccumulator_ = std::min(simAccumulator_ + (now - lastSimTime_), 0.25);
        lastSimTime_ = now;
        for (int steps = 0; steps < 15 && simAccumulator_ >= sim::kControlDt && simRunState_ == SIM_RUNNING; ++steps) {
            simAccumulator_ -= sim::kControlDt;
            TickSimulation(sim::kControlDt);
        }
    }

    // Cartesian map
    ImGui::SetCursorPos(ImVec2((screenWidth - frameWidth2) * 0.015f, frameHeight1 + 20.0f));
    ImGui::BeginChild("CoordinateFrame", ImVec2(frameWidth2, frameHeight2), false, ImGuiWindowFlags_NoScrollbar);
    {
        ImVec2 availableSize = ImGui::GetContentRegionAvail();
        if (ImPlot::BeginPlot("Vehicle Control Algorithms", availableSize,
                ImPlotFlags_NoFrame | ImPlotFlags_NoMenus | ImPlotFlags_Equal)) {
            const MapData& map  = map_loader_.GetMap();
            const PathData& path = map_loader_.GetPath();

            float w = map.loaded ? map.world_width  : 10.0f;
            float h = map.loaded ? map.world_height :  5.0f;
            ImPlot::SetupAxes("X-Axis", "Y-Axis");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, w, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, h, ImGuiCond_Always);

            if (map.loaded) {
                ImDrawList* dl = ImPlot::GetPlotDrawList();
                for (const auto& obs : map.obstacles) {
                    ImVec2 p_min = ImPlot::PlotToPixels(obs.x,             obs.y + obs.height);
                    ImVec2 p_max = ImPlot::PlotToPixels(obs.x + obs.width, obs.y);
                    dl->AddRectFilled(p_min, p_max, IM_COL32(100, 100, 120, 230));
                    dl->AddRect      (p_min, p_max, IM_COL32(160, 160, 180, 255), 0.0f, 0, 1.5f);
                }
            }

            if (path.loaded && !pathPlotX_.empty()) {
                ImPlot::SetNextLineStyle(ImVec4(0.3f, 0.9f, 0.3f, 1.0f), 2.0f);
                ImPlot::PlotLine("Path", pathPlotX_.data(), pathPlotY_.data(), (int)pathPlotX_.size());
            }

            if (map.loaded) {
                float sx = map.start.x, sy = map.start.y;
                float gx = map.goal.x,  gy = map.goal.y;
                ImPlot::SetNextLineStyle(ImVec4(0.0f, 1.0f, 0.2f, 1.0f));
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 10.0f, ImVec4(0.0f, 1.0f, 0.2f, 1.0f));
                ImPlot::PlotScatter("Start", &sx, &sy, 1);
                ImPlot::SetNextLineStyle(ImVec4(0.9f, 0.3f, 0.2f, 1.0f));
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 10.0f, ImVec4(0.9f, 0.3f, 0.2f, 1.0f));
                ImPlot::PlotScatter("Goal", &gx, &gy, 1);
            }

            if (simRunState_ != SIM_IDLE && !simTraceX_.empty()) {
                ImPlot::SetNextLineStyle(ImVec4(0.15f, 0.35f, 1.0f, 1.0f));
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.0f,
                    ImVec4(0.15f, 0.35f, 1.0f, 1.0f), 0.0f,
                    ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                ImPlot::PlotScatter("Trace", simTraceX_.data(), simTraceY_.data(),
                                    (int)simTraceX_.size());
                ImDrawList* dl = ImPlot::GetPlotDrawList();
                DrawVehicle(dl);
            }

            ImPlot::EndPlot();
        }
    }
    ImGui::EndChild();

    // Right panel
    ImGui::SetCursorPos(ImVec2(screenWidth - frameWidth3 - 10.0f, frameHeight1 + 20.0f));
    ImGui::BeginChild("RightFrame", ImVec2(frameWidth3, frameHeight3), false, ImGuiWindowFlags_NoScrollbar);
    {
        const char* simBtnLabel = (simRunState_ == SIM_RUNNING) ? "Stop"
                                : (simRunState_ == SIM_PAUSED)  ? "Resume"
                                : (simRunState_ == SIM_DONE ||
                                   simRunState_ == SIM_FAILED)  ? "Restart"
                                :                                 "Start Simulation";
        ImVec2 textSize   = ImGui::CalcTextSize(simBtnLabel);
        float buttonWidth  = textSize.x * 4.0f;
        float buttonHeight = textSize.y * 2.0f;
        float buttonX      = (frameWidth3 - buttonWidth) * 0.5f;

        // Start / Stop / Resume / Restart button near top
        ImGui::SetCursorPos(ImVec2(buttonX, 20.0f));
        if (ImGui::Button(simBtnLabel, ImVec2(buttonWidth, buttonHeight))) {
            if (simRunState_ == SIM_RUNNING) {
                simRunState_ = SIM_PAUSED;
            } else if (simRunState_ == SIM_PAUSED) {
                lastSimTime_ = glfwGetTime();
                simRunState_ = SIM_RUNNING;
            } else {
                InitSimulation();
                simRunState_ = SIM_RUNNING;
                lastSimTime_ = glfwGetTime();
            }
        }

        // Save Debug CSV — only actionable once a simulation run has been
        // started at least once (there's a log to save).
        ImGui::SetCursorPos(ImVec2(buttonX, 20.0f + buttonHeight + 8.0f));
        ImGui::BeginDisabled(simRunState_ == SIM_IDLE || simDebugLog_.empty());
        if (ImGui::Button("Save Debug CSV", ImVec2(buttonWidth, buttonHeight * 0.7f))) {
            SaveDebugLogCsv();
        }
        ImGui::EndDisabled();

        // Info panel
        float infoY = buttonHeight + 40.0f + buttonHeight * 0.7f + 8.0f;
        const float rowH = 22.0f;
        ImGui::SetCursorPos(ImVec2(10.0f, infoY));
        ImGui::Separator();
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f));
        ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Simulation Info");
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH));
        ImGui::Text("Algorithm    : %s", selectedAlgorithm_.c_str());
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 2));
        ImGui::Text("Vehicle Type : %s", vehicle_loader_.GetVehicle().name.c_str());
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 3));
        ImGui::Text("Model        : %s", selectedVehicleModelKind_.c_str());
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 4));
        ImGui::Text("Map          : %s", map_loader_.GetMap().name.c_str());
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 5));
        ImGui::Text("Speed        : %.1f km/h", simVehicleState_.speed * 3.6f);
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 6));
        ImGui::Text("Steering     : %.1f deg (wheel %.1f)", simLastDelta_ * (180.0f / 3.14159265f),
                    simVehicleState_.steering_angle * (180.0f / 3.14159265f));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Commanded tire angle, and in brackets the angle the steering actuator has\n"
                              "actually reached (rate and travel limited).");
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 7));
        const std::string failText = std::string("Failed: ") + sim::OutcomeName(simOutcome_);
        const char* statusStr = (simRunState_ == SIM_RUNNING) ? "Running"
                              : (simRunState_ == SIM_PAUSED)  ? "Paused"
                              : (simRunState_ == SIM_DONE)    ? "Done: goal reached"
                              : (simRunState_ == SIM_FAILED)  ? failText.c_str()
                              :                                 "Idle";
        ImVec4 statusColor = (simRunState_ == SIM_RUNNING) ? ImVec4(0.2f, 0.9f, 0.4f, 1.0f)
                           : (simRunState_ == SIM_PAUSED)  ? ImVec4(0.6f, 0.8f, 1.0f, 1.0f)
                           : (simRunState_ == SIM_DONE)    ? ImVec4(1.0f, 0.7f, 0.2f, 1.0f)
                           : (simRunState_ == SIM_FAILED)  ? ImVec4(1.0f, 0.35f, 0.3f, 1.0f)
                           :                                  ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::Text("Status       :");
        ImGui::SameLine();
        ImGui::TextColored(statusColor, "%s", statusStr);
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 8));
        ImGui::TextDisabled("Perf         : %.0f fps | tick %.2f ms | %.0f MB",
                            perfFrameMs_ > 0.0 ? 1000.0 / perfFrameMs_ : 0.0, perfTickMs_, perfRssMb_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Frame rate (drops to a few fps while idle by design), controller + vehicle-model\n"
                              "time per simulation tick, and resident memory of the process.");

        float statusRowBottom = infoY + 8.0f + rowH * 9;
        float remainingHeight = frameHeight3 - statusRowBottom - 24.0f;

        float modelTuningHeight = 210.0f;
        float algoTuningHeight  = remainingHeight - modelTuningHeight - ImGui::GetStyle().ItemSpacing.y;

        ImGui::SetCursorPos(ImVec2(10.0f, statusRowBottom + 8.0f));
        RenderVehicleModelTuning(modelTuningHeight);

        ImGui::SetCursorPos(ImVec2(10.0f, statusRowBottom + 8.0f + modelTuningHeight + ImGui::GetStyle().ItemSpacing.y));
        RenderParamTuning(algoTuningHeight);
    }
    ImGui::EndChild();  // RightFrame

    // Parameter hint tooltip — drawn on the foreground draw list so it is always
    // on top regardless of ImGui window ordering or simulation restarts.
    if (tooltipState_.lastHoverTime >= 0.0) {
        double elapsed  = glfwGetTime() - tooltipState_.lastHoverTime;
        float  relAlpha = (elapsed <= 0.0) ? 1.0f
                        : std::max(0.0f, 1.0f - (float)(elapsed / 0.25));
        if (relAlpha > 0.0f) {
            const char* txt    = tooltipState_.description.c_str();
            float       tipW   = tooltipState_.sliderMax.x - tooltipState_.sliderMin.x;
            float       padX   = 8.0f;
            float       padY   = 6.0f;
            float       maxTW  = tipW - padX * 2.0f;

            ImVec2 textSz = ImGui::CalcTextSize(txt, nullptr, false, maxTW);
            float  rectH  = textSz.y + padY * 2.0f;

            float  screenH = ImGui::GetIO().DisplaySize.y;
            bool   above   = tooltipState_.sliderMax.y > screenH * 0.6f;
            ImVec2 rMin    = above
                ? ImVec2(tooltipState_.sliderMin.x, tooltipState_.sliderMin.y - 4.0f - rectH)
                : ImVec2(tooltipState_.sliderMin.x, tooltipState_.sliderMax.y + 4.0f);
            ImVec2 rMax    = ImVec2(rMin.x + tipW, rMin.y + rectH);

            ImU32 bgCol  = IM_COL32(20, 20, 35,  (int)(255 * 0.75f * relAlpha));
            ImU32 brCol  = IM_COL32(100, 120, 160, (int)(200        * relAlpha));
            ImU32 txtCol = IM_COL32(255, 255, 255, (int)(255        * relAlpha));

            ImDrawList* dl = ImGui::GetForegroundDrawList();
            dl->AddRectFilled(rMin, rMax, bgCol, 4.0f);
            dl->AddRect      (rMin, rMax, brCol, 4.0f);

            // Center single-line text; left-align wrapped multi-line text
            float textX = (textSz.y <= ImGui::GetTextLineHeight() + 1.0f)
                        ? rMin.x + (tipW - textSz.x) * 0.5f
                        : rMin.x + padX;
            dl->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
                        ImVec2(textX, rMin.y + padY), txtCol,
                        txt, nullptr, maxTW);
        }
    }
}

void VehicleControlUI::ShowNotification(const std::string& title, const std::string& text, float duration_ms) {
    Notification n;
    n.title      = title;
    n.text       = text;
    n.duration   = duration_ms / 1000.0f;
    n.start_time = std::chrono::steady_clock::now();
    notifications.push_back(n);
}

void VehicleControlUI::RenderNotification() {
    const auto now = std::chrono::steady_clock::now();
    notifications.erase(std::remove_if(notifications.begin(), notifications.end(),
        [&](const Notification& n) {
            return std::chrono::duration<float>(now - n.start_time).count() > n.duration;
        }), notifications.end());
    // Nothing to show: return before any SetNextWindow*() call. ImGui keeps
    // that data until the next Begin(), so it would otherwise leak into the
    // next window drawn (e.g. resize the main window on the following frame).
    if (notifications.empty()) return;

    // Bottom-right corner, fixed width, height grows with the text (a list of
    // clamped parameters can be several lines long).
    ImGui::SetNextWindowPos(
        ImVec2(ImGui::GetIO().DisplaySize.x - 10.0f, ImGui::GetIO().DisplaySize.y - 10.0f),
        ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(340, 0), ImVec2(340, 420));
    ImGui::Begin("Notification", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove);
    for (size_t i = 0; i < notifications.size(); ++i) {
        if (i > 0) ImGui::Spacing();
        ImGui::Text("%s", notifications[i].title.c_str());
        ImGui::Separator();
        ImGui::TextWrapped("%s", notifications[i].text.c_str());
    }
    ImGui::End();
}

void VehicleControlUI::Cleanup() {
    if (window) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();   // ImPlot first: its context lives inside ImGui's
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        window = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Tuning limits
// ---------------------------------------------------------------------------

bool VehicleControlUI::IsDynamicModel() const {
    return selectedVehicleModelKind_ == "Dynamic (Linear Tire)";
}

const VehicleData& VehicleControlUI::ActiveVehicleData() const {
    return simEffectiveVehicleData_.loaded ? simEffectiveVehicleData_ : vehicle_loader_.GetVehicle();
}

float VehicleControlUI::MaxSpeedLimit() const {
    float hi = tuning::kMaxSpeed.hi;
    if (IsDynamicModel() && simFeasibilityValid_)
        hi = std::min(hi, simFeasibility_.speed_cap_mps);
    return std::max(tuning::kMaxSpeed.lo, hi);
}

float VehicleControlUI::ActuatorRateLimit() const {
    // The controllers plan with the actuator of the model they drive: the
    // vehicle's real rate under the Dynamic model when its file enables rate
    // limiting, otherwise the ideal (instant) steering of the Kinematic model.
    const VehicleData& vd = ActiveVehicleData();
    if (IsDynamicModel() && vd.steering.enabled && vd.steering.use_rate_limit && vd.max_steering_wheel_rate > 0.0f)
        return vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio;
    return std::numeric_limits<float>::max();
}

float VehicleControlUI::MaxDeltaLimit() const {
    float tire = vehicle::MaxTireAngle(ActiveVehicleData());
    if (!(tire > 0.0f)) return tuning::kMaxDelta.hi;
    return std::max(tuning::kMaxDelta.lo, std::min(tuning::kMaxDelta.hi, tire));
}

float VehicleControlUI::CogHeightLimit() const {
    float h = ActiveVehicleData().vehicle_height;
    if (!(h > 0.0f)) return tuning::kCogHeight.hi;
    return std::max(tuning::kCogHeight.lo,
                    std::min(tuning::kCogHeight.hi, tuning::kCogHeightShareOfVehicleHeight * h));
}

bool VehicleControlUI::ApplyTuningLimits(std::vector<std::string>& changes) {
    using namespace tuning;
    const size_t before = changes.size();
    auto cf = [&](const char* name, float& v, const FloatRange& r, float hi) {
        float c = std::isfinite(v) ? std::max(r.lo, std::min(std::max(r.lo, hi), v)) : r.lo;
        if (c != v) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%s %.3g -> %.3g", name, v, c);
            changes.emplace_back(buf);
            v = c;
        }
    };
    auto ci = [&](const char* name, int& v, const IntRange& r) {
        int c = std::max(r.lo, std::min(r.hi, v));
        if (c != v) {
            changes.emplace_back(std::string(name) + " " + std::to_string(v) + " -> " + std::to_string(c));
            v = c;
        }
    };
    const float vMax = MaxSpeedLimit();
    const float dMax = MaxDeltaLimit();

    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        cf("Min Lookahead",  tuneAPP_.min_lookahead,  kAPPMinLookahead,  kAPPMinLookahead.hi);
        cf("Max Lookahead",  tuneAPP_.max_lookahead,  kAPPMaxLookahead,  kAPPMaxLookahead.hi);
        cf("Speed Gain",     tuneAPP_.speed_gain,     kAPPSpeedGain,     kAPPSpeedGain.hi);
        cf("Curvature Gain", tuneAPP_.curvature_gain, kAPPCurvatureGain, kAPPCurvatureGain.hi);
        cf("Max Speed",      tuneAPP_.max_speed,      kMaxSpeed,         vMax);
        ci("Search Window",  tuneAPP_.search_window,  kSearchWindow);
    } else if (selectedAlgorithm_ == "LQR") {
        cf("Q0", tuneLQR_.q0, kLQRWeight, kLQRWeight.hi);
        cf("Q1", tuneLQR_.q1, kLQRWeight, kLQRWeight.hi);
        cf("Q2", tuneLQR_.q2, kLQRWeight, kLQRWeight.hi);
        cf("Q3", tuneLQR_.q3, kLQRWeight, kLQRWeight.hi);
        cf("Q4", tuneLQR_.q4, kLQRWeight, kLQRWeight.hi);
        cf("Q5", tuneLQR_.q5, kLQRWeight, kLQRWeight.hi);
        cf("R Steering",      tuneLQR_.r_steering,     kLQRWeight,        kLQRWeight.hi);
        cf("R Acceleration",  tuneLQR_.r_acceleration, kLQRWeight,        kLQRWeight.hi);
        cf("R Scale",         tuneLQR_.r_scale,        kLQRWeight,        kLQRWeight.hi);
        cf("Max Speed",       tuneLQR_.max_speed,      kMaxSpeed,         vMax);
        ci("DARE Iterations", tuneLQR_.dare_iterations, kLQRDareIterations);
        cf("DARE Threshold",  tuneLQR_.dare_threshold, kLQRDareThreshold, kLQRDareThreshold.hi);
        ci("Search Window",   tuneLQR_.search_window,  kSearchWindow);
    } else if (selectedAlgorithm_ == "Stanley") {
        cf("Stanley Gain", tuneStanley_.stanley_gain, kStanleyGain,     kStanleyGain.hi);
        cf("Min Speed",    tuneStanley_.min_speed,    kStanleyMinSpeed, kStanleyMinSpeed.hi);
        cf("Max Speed",    tuneStanley_.max_speed,    kMaxSpeed,        vMax);
        cf("Max Delta",    tuneStanley_.max_delta,    kMaxDelta,        dMax);
        ci("Search Window", tuneStanley_.search_window, kSearchWindow);
    } else if (selectedAlgorithm_ == "MPPI") {
        ci("Horizon T",     tuneMPPI_.horizon_T,     kMPPIHorizon);
        ci("Num Samples K", tuneMPPI_.num_samples_K, kMPPISamples);
        cf("Lambda",        tuneMPPI_.lambda,        kMPPILambda,    kMPPILambda.hi);
        cf("Sigma Steer",   tuneMPPI_.sigma_steer,   kMPPISigma,     kMPPISigma.hi);
        cf("Rollout dt",    tuneMPPI_.rollout_dt,    kMPPIRolloutDt, kMPPIRolloutDt.hi);
        cf("Max Speed",     tuneMPPI_.max_speed,     kMaxSpeed,      vMax);
        cf("W Lateral",     tuneMPPI_.w_lat,         kMPPIWeight,    kMPPIWeight.hi);
        cf("W Heading",     tuneMPPI_.w_heading,     kMPPIWeight,    kMPPIWeight.hi);
        ci("Search Window", tuneMPPI_.search_window, kSearchWindow);
        cf("Max Delta",     tuneMPPI_.max_delta,     kMaxDelta,      dMax);
        ci("RNG Seed",      tuneMPPI_.rng_seed,      kMPPISeed);
    } else if (selectedAlgorithm_ == "MPC") {
        ci("Horizon N",     tuneMPC_.horizon_N,     kMPCHorizon);
        ci("Iterations",    tuneMPC_.iterations,    kMPCIterations);
        cf("Rollout dt",    tuneMPC_.rollout_dt,    kMPCRolloutDt,    kMPCRolloutDt.hi);
        cf("Max Speed",     tuneMPC_.max_speed,     kMaxSpeed,        vMax);
        cf("W Lateral",     tuneMPC_.w_lat,         kMPCWeight,       kMPCWeight.hi);
        cf("W Heading",     tuneMPC_.w_heading,     kMPCWeight,       kMPCWeight.hi);
        cf("W Control",     tuneMPC_.w_control,     kMPCWControl,     kMPCWControl.hi);
        cf("W Rate",        tuneMPC_.w_rate,        kMPCWRate,        kMPCWRate.hi);
        ci("Search Window", tuneMPC_.search_window, kSearchWindow);
        cf("Max Delta",     tuneMPC_.max_delta,     kMaxDelta,        dMax);
    } else {
        cf("Lookahead Dist", tunePP_.lookahead_distance, kPPLookaheadDist, kPPLookaheadDist.hi);
        cf("Lookahead Gain", tunePP_.lookahead_gain,     kPPLookaheadGain, kPPLookaheadGain.hi);
        cf("Max Speed",      tunePP_.max_speed,          kMaxSpeed,        vMax);
        ci("Search Window",  tunePP_.search_window,      kSearchWindow);
    }
    return changes.size() > before;
}

void VehicleControlUI::NotifyLimited(const std::string& title, const std::vector<std::string>& lines) {
    if (lines.empty()) return;
    std::string text;
    for (const auto& l : lines) text += (text.empty() ? "" : "\n") + l;
    ShowNotification(title, text, 4000.0f + 800.0f * static_cast<float>(std::min<size_t>(lines.size(), 8)));
}

void VehicleControlUI::RebuildPathPlotCache() {
    pathPlotX_.clear();
    pathPlotY_.clear();
    for (const auto& wp : map_loader_.GetPath().waypoints) {
        pathPlotX_.push_back(wp.x);
        pathPlotY_.push_back(wp.y);
    }
}

// ---------------------------------------------------------------------------
// Rendering cadence and performance read-out
// ---------------------------------------------------------------------------

bool VehicleControlUI::NeedsContinuousRedraw() const {
    const double now = glfwGetTime();
    return simRunState_ == SIM_RUNNING
        || !notifications.empty()
        || (tooltipState_.lastHoverTime >= 0.0 && now - tooltipState_.lastHoverTime < 0.3)
        || now - lastActivityTime_ < 0.5;
}

void VehicleControlUI::UpdatePerfStats(double frame_seconds) {
    const double ms = frame_seconds * 1000.0;
    perfFrameMs_ = (perfFrameMs_ <= 0.0) ? ms : 0.9 * perfFrameMs_ + 0.1 * ms;

    const double now = glfwGetTime();
    if (now - perfLastRssRead_ < 1.0) return;
    perfLastRssRead_ = now;
#ifdef __linux__
    if (FILE* f = std::fopen("/proc/self/statm", "r")) {
        long pages_total = 0, pages_resident = 0;
        if (std::fscanf(f, "%ld %ld", &pages_total, &pages_resident) == 2)
            perfRssMb_ = static_cast<double>(pages_resident) * static_cast<double>(sysconf(_SC_PAGESIZE)) / (1024.0 * 1024.0);
        std::fclose(f);
    }
#endif
}
