#include "vehicle/aero/simple_aero_model.hpp"
#include <cmath>

namespace vehicle {

SimpleAeroModel::SimpleAeroModel(const SimpleAeroConfig& config) : config_(config) {}

float SimpleAeroModel::ComputeDragForce(float vx) const {
    return -config_.mass * config_.CA * vx * std::abs(vx);
}

} // namespace vehicle
