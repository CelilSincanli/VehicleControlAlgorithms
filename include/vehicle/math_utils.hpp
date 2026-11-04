#pragma once

namespace vehicle {

// ---------------------------------------------------------------------------
// Small numeric helpers shared across the vehicle model subsystems.
// Extracted here because the std::max(-x, std::min(x, v)) clamp idiom was
// about to be duplicated across the new tire/steering code on top of the
// four places it already appears (stanley.cpp, mpc.cpp, mppi.cpp,
// vehicle_bicycle_model.cpp).
// ---------------------------------------------------------------------------

template <typename T>
constexpr T Clamp(T value, T lo, T hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

} // namespace vehicle
