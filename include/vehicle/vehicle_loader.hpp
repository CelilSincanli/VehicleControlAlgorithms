#pragma once
#include "vehicle_data.hpp"
#include <string>
#include <vector>

class VehicleLoader {
public:
    bool Load(const std::string& json_path);

    const VehicleData& GetVehicle() const { return vehicle_; }

    // Fields the last Load() had to clamp into their physical range or that
    // were missing (then set to the nearest allowed value), one line each.
    const std::vector<std::string>& GetWarnings() const { return warnings_; }

    // Why the last Load() returned false (missing file or JSON syntax error).
    const std::string& GetError() const { return error_; }

    static std::vector<std::string> DiscoverVehicles(const std::string& vehicle_dir);

private:
    VehicleData              vehicle_;
    std::vector<std::string> warnings_;
    std::string              error_;

    void Validate(const std::vector<std::string>& present_keys);
};
