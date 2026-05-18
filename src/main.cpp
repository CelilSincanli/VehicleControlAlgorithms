#include "vehicle_control_ui.hpp"

int main() {
    VehicleControlUI ui;
    if (!ui.Initialize()) return -1;
    ui.Run();
    return 0;
}
