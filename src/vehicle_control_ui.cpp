#include "vehicle_control_ui.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

VehicleControlUI::VehicleControlUI()
    : window(nullptr), mainFont(nullptr), headingFont(nullptr),
      iconFont(nullptr), iconFont2(nullptr),
      algorithmSelected(false), pathSelected(false),
      currentScreen(MAIN_SCREEN) {}

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

    // 3 buttons + 2 spacers between them
    float total_height = text_size.y + spacing + button_height + spacing + button_height + spacing + button_height;
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

    // Button 1: Select Algorithm
    start_y += text_size.y + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::rocket);
    ImGui::PopFont();
    ImGui::SameLine();
    if (ImGui::Button("Select Algorithm", ImVec2(button_width, button_height))) {
        algorithmSelected = true;
    }
    if (algorithmSelected) {
        ImGui::SameLine();
        ImGui::PushFont(iconFont2);
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "%s", dripicon_v2::checkmark);
        ImGui::PopFont();
    }

    // Button 2: Select Path
    start_y += button_height + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::upload);
    ImGui::PopFont();
    ImGui::SameLine();
    if (ImGui::Button("Select Path", ImVec2(button_width, button_height))) {
        pathSelected = true;
    }
    if (pathSelected) {
        ImGui::SameLine();
        ImGui::PushFont(iconFont2);
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "%s", dripicon_v2::checkmark);
        ImGui::PopFont();
    }

    // Button 3: Start Simulation (disabled until both above are selected)
    start_y += button_height + spacing;
    ImGui::SetCursorPos(ImVec2(button_x, start_y));
    ImGui::PushFont(iconFont);
    ImGui::Text("%s", dripicon_v2::media_play);
    ImGui::PopFont();
    ImGui::SameLine();

    bool canStart = algorithmSelected && pathSelected;
    ImGui::BeginDisabled(!canStart);
    if (ImGui::Button("Start Simulation", ImVec2(button_width, button_height))) {
        currentScreen = SIMULATION_SCREEN;
    }
    ImGui::EndDisabled();
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
            currentScreen = MAIN_SCREEN;
        }
        ImGui::PopFont();
    }
    ImGui::EndChild();

    // Cartesian map
    ImGui::SetCursorPos(ImVec2((screenWidth - frameWidth2) * 0.015f, frameHeight1 + 20.0f));
    ImGui::BeginChild("CoordinateFrame", ImVec2(frameWidth2, frameHeight2), false, ImGuiWindowFlags_NoScrollbar);
    {
        ImVec2 availableSize = ImGui::GetContentRegionAvail();
        if (ImPlot::BeginPlot("Vehicle Control Algorithms", availableSize,
                ImPlotFlags_NoLegend | ImPlotFlags_NoFrame | ImPlotFlags_NoMenus)) {
            ImPlot::SetupAxes("X-Axis", "Y-Axis");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, 10);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 5);
            ImPlot::EndPlot();
        }
    }
    ImGui::EndChild();

    // Right panel
    ImGui::SetCursorPos(ImVec2(screenWidth - frameWidth3 - 10.0f, frameHeight1 + 20.0f));
    ImGui::BeginChild("RightFrame", ImVec2(frameWidth3, frameHeight3), false, ImGuiWindowFlags_NoScrollbar);
    {
        RenderNotification();

        const char* saveButtonText = "Save Diagram";
        ImVec2 textSize   = ImGui::CalcTextSize(saveButtonText);
        float buttonWidth  = textSize.x * 4.0f;
        float buttonHeight = textSize.y * 2.0f;
        float buttonX      = (frameWidth3 - buttonWidth) * 0.5f;
        float buttonY      = frameHeight3 - buttonHeight - 20.0f;

        ImGui::SetCursorPos(ImVec2(buttonX, buttonY));
        if (ImGui::Button(saveButtonText, ImVec2(buttonWidth, buttonHeight))) {
            std::cout << "Save Diagram clicked" << std::endl;
        }
    }
    ImGui::EndChild();
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
