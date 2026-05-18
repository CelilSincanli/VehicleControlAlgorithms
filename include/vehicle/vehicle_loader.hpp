#pragma once
#include "vehicle_data.hpp"
#include <string>
#include <vector>

class VehicleLoader {
public:
    bool Load(const std::string& json_path);

    const VehicleData& GetVehicle() const { return vehicle_; }

    static std::vector<std::string> DiscoverVehicles(const std::string& vehicle_dir);

private:
    VehicleData vehicle_;
};
