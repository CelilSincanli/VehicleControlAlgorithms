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

#include "dripicon_v2.h"
#include "dripicon_v2_icons.hpp"

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

    bool algorithmSelected;
    bool pathSelected;

    struct Notification {
        std::string title;
        std::string text;
        std::chrono::steady_clock::time_point start_time;
        float duration;
    };

    std::vector<Notification> notifications;

    enum Screen { MAIN_SCREEN, SIMULATION_SCREEN };
    Screen currentScreen;

    void RenderUI();
    void RenderMainScreen();
    void RenderSimulationScreen();
    void ShowNotification(const std::string& title, const std::string& text, float duration_ms);
    void RenderNotification();
};

#endif // VEHICLE_CONTROL_UI_HPP
