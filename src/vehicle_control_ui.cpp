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

const std::vector<std::string> VehicleControlUI::kAvailableAlgorithms = {
    "Pure Pursuit",
    "Adaptive Pure Pursuit",
    "LQR",
    "Stanley",
    "MPPI",
    "MPC",
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
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        RenderUI();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
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
}

void VehicleControlUI::InitSimulation() {
    const VehicleData& vd  = vehicle_loader_.GetVehicle();
    const MapData&  map  = map_loader_.GetMap();
    const PathData& path = map_loader_.GetPath();

    // simScale_: map_units = real_meters * simScale_
    float real_half_w  = vd.wheel_tread * 0.5f + std::max(vd.left_overhang, vd.right_overhang);
    simScale_     = map.vehicle_radius / real_half_w;

    // Rear-axle-referenced body dimensions in map units
    simHalfFront_   = (vd.wheelbase + vd.front_overhang) * simScale_;  // rear-axle → front bumper
    simHalfRear_    = vd.rear_overhang * simScale_;                     // rear-axle → rear bumper
    simHalfWidth_   = real_half_w * simScale_;                          // == map.vehicle_radius
    simWheelbaseMap_= vd.wheelbase * simScale_;

    // Tire visual dimensions in map units
    simTireHL_   = vd.wheel_radius * simScale_;
    simTireHW_   = vd.wheel_width  * 0.5f * simScale_;
    simTreadHW_  = vd.wheel_tread  * 0.5f * simScale_;

    // Vehicle state in REAL METERS (rear axle reference)
    simVehicleState_         = {};
    simVehicleState_.x       = map.start.x / simScale_;
    simVehicleState_.y       = map.start.y / simScale_;
    simVehicleState_.heading = 0.0f;
    simVehicleState_.speed   = 0.0f;
    simLastDelta_            = 0.0f;

    // Convert path waypoints to real meters
    simScaledPath_.clear();
    for (const auto& wp : path.waypoints)
        simScaledPath_.push_back({wp.x / simScale_, wp.y / simScale_});

    simTraceX_.clear();
    simTraceY_.clear();
    simTraceX_.push_back(map.start.x);
    simTraceY_.push_back(map.start.y);

    simModel_ = std::make_unique<vehicle::KinematicBicycleModel>(vd);

    // Load JSON defaults into tuning structs (only when override is not enabled)
    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        if (!tuneAPP_.enabled) {
            auto fcfg = path_tracking::LoadAdaptivePurePursuitConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/adaptive_pure_pursuit.json");
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
            tuneLQR_.q0             = fcfg.q0;
            tuneLQR_.q1             = fcfg.q1;
            tuneLQR_.q2             = fcfg.q2;
            tuneLQR_.q3             = fcfg.q3;
            tuneLQR_.q4             = fcfg.q4;
            tuneLQR_.r_steering     = fcfg.r_steering;
            tuneLQR_.r_acceleration = fcfg.r_acceleration;
            tuneLQR_.r_scale        = fcfg.r_scale;
            tuneLQR_.time_step      = fcfg.time_step;
            tuneLQR_.max_speed      = fcfg.max_speed;
            tuneLQR_.dare_iterations = fcfg.dare_iterations;
            tuneLQR_.dare_threshold  = fcfg.dare_threshold;
            tuneLQR_.search_window   = fcfg.search_window;
        }
    } else if (selectedAlgorithm_ == "Stanley") {
        if (!tuneStanley_.enabled) {
            auto fcfg = path_tracking::LoadStanleyConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/stanley.json");
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
            tuneMPC_.horizon_N     = fcfg.horizon_N;
            tuneMPC_.iterations    = fcfg.iterations;
            tuneMPC_.learning_rate = fcfg.learning_rate;
            tuneMPC_.fd_step       = fcfg.fd_step;
            tuneMPC_.rollout_dt    = fcfg.rollout_dt;
            tuneMPC_.max_speed     = fcfg.max_speed_mps;
            tuneMPC_.w_lat         = fcfg.w_lat;
            tuneMPC_.w_heading     = fcfg.w_heading;
            tuneMPC_.w_control     = fcfg.w_control;
            tuneMPC_.search_window = fcfg.search_window;
            tuneMPC_.max_delta     = fcfg.max_delta;
        }
    } else {
        if (!tunePP_.enabled) {
            auto fcfg = path_tracking::LoadPurePursuitConfig(
                            std::string(CONFIG_PATH) + "/path_tracking/pure_pursuit.json");
            tunePP_.lookahead_distance = fcfg.lookahead_distance;
            tunePP_.lookahead_gain     = fcfg.lookahead_gain;
            tunePP_.max_speed          = fcfg.max_speed_mps;
            tunePP_.search_window      = fcfg.search_window;
        }
    }

    RebuildAlgorithm();
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
        cfg.r_steering      = tuneLQR_.r_steering;
        cfg.r_acceleration  = tuneLQR_.r_acceleration;
        cfg.r_scale         = tuneLQR_.r_scale;
        cfg.time_step       = tuneLQR_.time_step;
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
        cfg.max_delta     = tuneStanley_.max_delta;
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
        cfg.max_delta     = tuneMPPI_.max_delta;
        cfg.rng_seed      = tuneMPPI_.rng_seed;
        simPursuit_ = std::make_unique<path_tracking::Mppi>(cfg);
    } else if (selectedAlgorithm_ == "MPC") {
        simMaxSpeed_ = tuneMPC_.max_speed;
        path_tracking::MpcConfig cfg;
        cfg.horizon_N     = tuneMPC_.horizon_N;
        cfg.iterations    = tuneMPC_.iterations;
        cfg.learning_rate = tuneMPC_.learning_rate;
        cfg.fd_step       = tuneMPC_.fd_step;
        cfg.rollout_dt    = tuneMPC_.rollout_dt;
        cfg.max_speed     = tuneMPC_.max_speed;
        cfg.w_lat         = tuneMPC_.w_lat;
        cfg.w_heading     = tuneMPC_.w_heading;
        cfg.w_control     = tuneMPC_.w_control;
        cfg.wheelbase     = vd.wheelbase;
        cfg.search_window = tuneMPC_.search_window;
        cfg.max_delta     = tuneMPC_.max_delta;
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

    if (paramsDirty_) RebuildAlgorithm();

    // Goal in real meters
    const MapData& map = map_loader_.GetMap();
    float gx = map.goal.x / simScale_;
    float gy = map.goal.y / simScale_;
    float dx = gx - simVehicleState_.x;
    float dy = gy - simVehicleState_.y;
    if (dx*dx + dy*dy < 1.0f) { simRunState_ = SIM_DONE; return; }  // 1 m radius

    float delta = simPursuit_->ComputeSteering(simVehicleState_);
    simLastDelta_ = delta;

    vehicle::VehicleControls ctrl;
    ctrl.steering_angle = delta;
    ctrl.acceleration   = 1.0f * (simMaxSpeed_ - simVehicleState_.speed);

    simVehicleState_ = simModel_->Step(simVehicleState_, ctrl, dt);

    // Record trace dot every 0.5 m of travel (in map units)
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
        }
    }
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
    drawTire(fL_x, fL_y, psi + simLastDelta_);
    drawTire(fR_x, fR_y, psi + simLastDelta_);

    // --- Lookahead target dot + line from rear axle ---
    Point2D lh   = simPursuit_->GetLookaheadPoint();
    ImVec2 lhPx  = ImPlot::PlotToPixels(lh.x * simScale_, lh.y * simScale_);
    ImVec2 raPx  = ImPlot::PlotToPixels(rax, ray);
    dl->AddLine        (raPx, lhPx, IM_COL32(100, 180, 255, 160), 1.5f);
    dl->AddCircleFilled(lhPx, 6.0f, IM_COL32(255, 220, 50, 220));
    dl->AddCircle      (lhPx, 6.0f, IM_COL32(255, 255, 255, 180), 12, 1.5f);
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

    // Slider count for active algorithm
    int N = 4;
    if      (selectedAlgorithm_ == "Adaptive Pure Pursuit") N = 6;
    else if (selectedAlgorithm_ == "LQR")                   N = 13;
    else if (selectedAlgorithm_ == "Stanley")               N = 5;
    else if (selectedAlgorithm_ == "MPC")                   N = 11;
    else if (selectedAlgorithm_ == "MPPI")                  N = 11;

    float headerH = 1.0f
                  + ImGui::GetTextLineHeightWithSpacing()
                  + ImGui::GetFrameHeightWithSpacing()
                  + ImGui::GetStyle().ItemSpacing.y;
    float sliderAreaH = availableHeight - headerH;

    float spacing = ImGui::GetStyle().ItemSpacing.y;
    float perH    = (sliderAreaH - spacing * (N - 1)) / static_cast<float>(N);
    float minH    = ImGui::GetTextLineHeight() + 4.0f;
    perH = std::max(perH, minH);
    float padY = std::max(2.0f, (perH - ImGui::GetTextLineHeight()) * 0.5f);

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

    if (selectedAlgorithm_ == "Adaptive Pure Pursuit") {
        sliderF("Min Lookahead",   &tuneAPP_.min_lookahead,  0.1f,  8.0f,  0.1f, "%.1f m",
            "Floor for lookahead distance. Target point never closer than this.");
        sliderF("Max Lookahead",   &tuneAPP_.max_lookahead,  8.0f,  20.0f, 0.1f, "%.1f m",
            "Ceiling for lookahead distance. Target point never farther than this.");
        sliderF("Speed Gain",      &tuneAPP_.speed_gain,     0.1f,  1.0f,  0.1f, "%.1f",
            "How strongly current speed scales the lookahead. Higher = farther look at speed.");
        sliderF("Curvature Gain",  &tuneAPP_.curvature_gain, 0.1f,  5.0f,  0.1f, "%.1f",
            "Lookahead penalty on path curvature. Higher = shorter look on tight corners.");
        sliderF("Max Speed",       &tuneAPP_.max_speed,      0.1f,  20.0f, 0.1f, "%.1f m/s",
            "Target cruise speed the controller accelerates towards.");
        sliderI("Search Window",   &tuneAPP_.search_window,  20,    80,    1,
            "Waypoints scanned ahead when searching for the lookahead target point.");

    } else if (selectedAlgorithm_ == "LQR") {
        sliderF("Q0",              &tuneLQR_.q0,              0.1f,  4.0f,  0.1f,  "%.1f",
            "Cost weight on lateral (cross-track) position error.");
        sliderF("Q1",              &tuneLQR_.q1,              0.1f,  4.0f,  0.1f,  "%.1f",
            "Cost weight on lateral error rate (derivative).");
        sliderF("Q2",              &tuneLQR_.q2,              0.1f,  4.0f,  0.1f,  "%.1f",
            "Cost weight on yaw (heading) error. Higher = faster alignment.");
        sliderF("Q3",              &tuneLQR_.q3,              0.1f,  4.0f,  0.1f,  "%.1f",
            "Cost weight on yaw rate (derivative of heading).");
        sliderF("Q4",              &tuneLQR_.q4,              0.1f,  4.0f,  0.1f,  "%.1f",
            "Cost weight on speed error from target speed.");
        sliderF("R Steering",      &tuneLQR_.r_steering,      0.1f,  4.0f,  0.1f,  "%.1f",
            "Penalty on steering control effort. Higher = smoother, less aggressive steering.");
        sliderF("R Acceleration",  &tuneLQR_.r_acceleration,  0.1f,  4.0f,  0.1f,  "%.1f",
            "Penalty on acceleration control effort.");
        sliderF("R Scale",         &tuneLQR_.r_scale,         0.1f,  4.0f,  0.1f,  "%.1f",
            "Global multiplier applied to both R weights (r_steering and r_acceleration).");
        sliderF("Time Step",       &tuneLQR_.time_step,       0.01f, 0.05f, 0.01f, "%.2f s",
            "Discrete time step used inside the LQR linearisation and DARE solver.");
        sliderF("Max Speed",       &tuneLQR_.max_speed,       0.1f,  20.0f, 0.1f,  "%.1f m/s",
            "Target cruise speed the controller accelerates towards.");
        sliderI("DARE Iterations", &tuneLQR_.dare_iterations, 100,   250,   10,
            "Maximum iterations allowed for the DARE (Riccati equation) solver.");
        sliderF("DARE Threshold",  &tuneLQR_.dare_threshold,  0.01f, 0.05f, 0.01f, "%.2f",
            "Convergence tolerance for the DARE solver. Smaller = more precise K matrix.");
        sliderI("Search Window",   &tuneLQR_.search_window,   20,    80,    1,
            "Waypoints scanned ahead to find the closest reference path point.");

    } else if (selectedAlgorithm_ == "Stanley") {
        sliderF("Stanley Gain",    &tuneStanley_.stanley_gain,  0.1f, 1.0f,  0.1f, "%.1f",
            "Cross-track error gain k. Higher = more aggressive lateral correction.");
        sliderF("Min Speed",       &tuneStanley_.min_speed,     0.1f, 1.0f,  0.1f, "%.1f m/s",
            "Speed floor used in the Stanley formula denominator to prevent large outputs at low speed.");
        sliderF("Max Speed",       &tuneStanley_.max_speed,     5.0f, 20.0f, 1.0f, "%.1f m/s",
            "Target cruise speed the controller accelerates towards.");
        sliderF("Max Delta",       &tuneStanley_.max_delta,     0.1f, 2.0f,  0.1f, "%.1f rad",
            "Hard clamp on the steering output angle in radians.");
        sliderI("Search Window",   &tuneStanley_.search_window, 20,   80,    1,
            "Waypoints scanned ahead to find the closest front-axle reference point.");

    } else if (selectedAlgorithm_ == "MPPI") {
        sliderI("Horizon T",       &tuneMPPI_.horizon_T,      5,    25,    1,
            "Number of time steps in each rollout trajectory.");
        sliderI("Num Samples K",   &tuneMPPI_.num_samples_K,  50,   200,   1,
            "Random trajectories sampled per control step. More = better quality, slower.");
        sliderF("Lambda",          &tuneMPPI_.lambda,         0.5f, 2.0f,  0.1f, "%.1f",
            "Temperature parameter. Lower concentrates weight on the best-cost samples.");
        sliderF("Sigma Steer",     &tuneMPPI_.sigma_steer,    0.1f, 1.0f,  0.1f, "%.1f rad",
            "Standard deviation of the steering noise added to each sample trajectory.");
        sliderF("Rollout dt",      &tuneMPPI_.rollout_dt,     0.01f, 0.5f, 0.01f, "%.2f s",
            "Simulation time step used inside each rollout.");
        sliderF("Max Speed",       &tuneMPPI_.max_speed,      0.1f, 20.0f, 0.1f, "%.1f m/s",
            "Target cruise speed the controller accelerates towards.");
        sliderF("W Lateral",       &tuneMPPI_.w_lat,          0.1f, 2.0f,  0.1f, "%.1f",
            "Cost weight on lateral (cross-track) error in trajectory evaluation.");
        sliderF("W Heading",       &tuneMPPI_.w_heading,      0.1f, 2.0f,  0.1f, "%.1f",
            "Cost weight on heading (yaw) error in trajectory evaluation.");
        sliderI("Search Window",   &tuneMPPI_.search_window,  20,   80,    1,
            "Waypoints scanned ahead during rollout cost evaluation.");
        sliderF("Max Delta",       &tuneMPPI_.max_delta,      0.1f, 2.0f,  0.1f, "%.1f rad",
            "Hard clamp on the steering output angle in radians.");
        sliderI("RNG Seed",        &tuneMPPI_.rng_seed,       20,   60,    1,
            "Random seed for noise generation. Change for different sample patterns.");

    } else if (selectedAlgorithm_ == "MPC") {
        sliderI("Horizon N",       &tuneMPC_.horizon_N,      5,    25,    1,
            "Number of prediction steps in the MPC horizon.");
        sliderI("Iterations",      &tuneMPC_.iterations,     5,    25,    1,
            "Gradient descent iterations performed per control step.");
        sliderF("Learning Rate",   &tuneMPC_.learning_rate,  0.01f, 0.2f,  0.01f, "%.2f",
            "Gradient descent step size. Too large causes oscillation; too small is slow.");
        sliderF("FD Step",         &tuneMPC_.fd_step,        0.001f, 0.02f, 0.001f, "%.3f",
            "Finite difference perturbation for Jacobian estimation in radians.");
        sliderF("Rollout dt",      &tuneMPC_.rollout_dt,     0.01f, 0.2f,  0.01f, "%.2f s",
            "Simulation time step used inside each prediction rollout.");
        sliderF("Max Speed",       &tuneMPC_.max_speed,      0.1f, 20.0f,  0.1f,  "%.1f m/s",
            "Target cruise speed the controller accelerates towards.");
        sliderF("W Lateral",       &tuneMPC_.w_lat,          0.1f, 2.0f,   0.1f,  "%.1f",
            "Cost weight on lateral (cross-track) error over the horizon.");
        sliderF("W Heading",       &tuneMPC_.w_heading,      0.1f, 2.0f,   0.1f,  "%.1f",
            "Cost weight on heading (yaw) error over the horizon.");
        sliderF("W Control",       &tuneMPC_.w_control,      0.01f, 0.5f,  0.01f, "%.2f",
            "Penalty on control effort magnitude. Higher = smoother steering.");
        sliderI("Search Window",   &tuneMPC_.search_window,  20,   80,    1,
            "Waypoints scanned ahead during rollout cost evaluation.");
        sliderF("Max Delta",       &tuneMPC_.max_delta,      0.1f, 2.0f,   0.1f,  "%.1f rad",
            "Hard clamp on the steering output angle in radians.");

    } else { // Pure Pursuit
        sliderF("Lookahead Dist",  &tunePP_.lookahead_distance, 0.1f, 10.0f, 0.1f, "%.1f m",
            "Base lookahead distance. Added to the speed-scaled term: Ld = base + gain * speed.");
        sliderF("Lookahead Gain",  &tunePP_.lookahead_gain,     0.1f, 1.0f,  0.1f, "%.1f",
            "Speed multiplier for lookahead. Higher = farther look at higher speeds.");
        sliderF("Max Speed",       &tunePP_.max_speed,          0.1f, 20.0f, 0.1f, "%.1f m/s",
            "Target cruise speed the controller accelerates towards.");
        sliderI("Search Window",   &tunePP_.search_window,      20,   80,    1,
            "Waypoints scanned ahead when searching for the lookahead target point.");
    }

    ImGui::PopItemWidth();
    ImGui::PopStyleVar();
    if (!*enabled) ImGui::EndDisabled();
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

    // Per-frame simulation tick
    if (simRunState_ == SIM_RUNNING) {
        double now = glfwGetTime();
        float dt = std::min((float)(now - lastSimTime_), 0.1f);
        lastSimTime_ = now;
        TickSimulation(dt);
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

            if (path.loaded) {
                std::vector<float> px, py;
                for (const auto& wp : path.waypoints) {
                    px.push_back(wp.x);
                    py.push_back(wp.y);
                }
                ImPlot::SetNextLineStyle(ImVec4(0.3f, 0.9f, 0.3f, 1.0f), 2.0f);
                ImPlot::PlotLine("Path", px.data(), py.data(), (int)px.size());
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
        RenderNotification();

        const char* simBtnLabel = (simRunState_ == SIM_RUNNING) ? "Stop"
                                : (simRunState_ == SIM_PAUSED)  ? "Resume"
                                : (simRunState_ == SIM_DONE)    ? "Restart"
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

        // Info panel
        float infoY = buttonHeight + 40.0f;
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
        ImGui::Text("Map          : %s", map_loader_.GetMap().name.c_str());
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 4));
        ImGui::Text("Speed        : %.1f km/h", simVehicleState_.speed * 3.6f);
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 5));
        ImGui::Text("Steering     : %.1f deg", simLastDelta_ * (180.0f / 3.14159265f));
        ImGui::SetCursorPos(ImVec2(10.0f, infoY + 8.0f + rowH * 6));
        const char* statusStr = (simRunState_ == SIM_RUNNING) ? "Running"
                              : (simRunState_ == SIM_DONE)    ? "Done"
                              :                                 "Idle";
        ImVec4 statusColor = (simRunState_ == SIM_RUNNING) ? ImVec4(0.2f, 0.9f, 0.4f, 1.0f)
                           : (simRunState_ == SIM_DONE)    ? ImVec4(1.0f, 0.7f, 0.2f, 1.0f)
                           :                                  ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::Text("Status       :");
        ImGui::SameLine();
        ImGui::TextColored(statusColor, "%s", statusStr);

        float statusRowBottom = infoY + 8.0f + rowH * 7;
        float tuningHeight    = frameHeight3 - statusRowBottom - 24.0f;

        ImGui::SetCursorPos(ImVec2(10.0f, statusRowBottom + 8.0f));
        RenderParamTuning(tuningHeight);
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
    ImGui::SetNextWindowPos(
        ImVec2(ImGui::GetIO().DisplaySize.x - 250, ImGui::GetIO().DisplaySize.y - 100),
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(240, 80), ImGuiCond_Always);

    for (int i = 0; i < (int)notifications.size();) {
        Notification& n = notifications[i];
        float elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - n.start_time).count();
        if (elapsed > n.duration) {
            notifications.erase(notifications.begin() + i);
        } else {
            ImGui::Begin("Notification", nullptr,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove);
            ImGui::Text("%s", n.title.c_str());
            ImGui::Separator();
            ImGui::TextWrapped("%s", n.text.c_str());
            ImGui::End();
            ++i;
        }
    }
}

void VehicleControlUI::Cleanup() {
    if (window) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        window = nullptr;
    }
    ImPlot::DestroyContext();
}
