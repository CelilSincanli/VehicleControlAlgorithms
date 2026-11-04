#pragma once

// ---------------------------------------------------------------------------
// Allowed range of every live-tunable parameter.
// Single source of truth: RenderParamTuning()/RenderVehicleModelTuning() draw
// their sliders from these ranges, and InitSimulation() clamps the JSON values
// in config/path_tracking/*.json into the same ranges (with a notification).
//
// Two upper bounds are further tightened per run, from the loaded vehicle and
// path (see path_tracking::AnalyzePathFeasibility):
//   * every "Max Delta"  -> the vehicle's real max tire angle
//   * every "Max Speed"  -> the path's speed cap, under Dynamic fidelity only
// ---------------------------------------------------------------------------
namespace tuning {

struct FloatRange { float lo, hi, step; };
struct IntRange   { int   lo, hi, step; };

// Shared by several algorithms (loaders also clamp into these; see config_reader.hpp)
inline constexpr FloatRange kMaxSpeed      {0.5f,   15.0f,  0.1f};   // [m/s]
inline constexpr FloatRange kMaxDelta      {0.05f,  1.0f,   0.01f};  // [rad]
inline constexpr IntRange   kSearchWindow  {20,     80,     1};      // [waypoints]

// Pure Pursuit
inline constexpr FloatRange kPPLookaheadDist {0.5f, 10.0f, 0.1f};    // [m]
inline constexpr FloatRange kPPLookaheadGain {0.1f, 1.0f,  0.1f};

// Adaptive Pure Pursuit
inline constexpr FloatRange kAPPMinLookahead {0.5f, 8.0f,  0.1f};    // [m]
inline constexpr FloatRange kAPPMaxLookahead {8.0f, 20.0f, 0.1f};    // [m]
inline constexpr FloatRange kAPPSpeedGain    {0.1f, 1.0f,  0.1f};
inline constexpr FloatRange kAPPCurvatureGain{0.1f, 5.0f,  0.1f};

// LQR
inline constexpr FloatRange kLQRWeight       {0.1f,  4.0f,  0.1f};   // q0..q5, r_steering, r_acceleration, r_scale
inline constexpr IntRange   kLQRDareIterations{100,  250,   10};
inline constexpr FloatRange kLQRDareThreshold{0.01f, 0.05f, 0.01f};

// Stanley
inline constexpr FloatRange kStanleyGain     {0.1f, 1.0f, 0.1f};
inline constexpr FloatRange kStanleyMinSpeed {0.1f, 1.0f, 0.1f};     // [m/s]

// MPPI
inline constexpr IntRange   kMPPIHorizon     {5,     25,   1};
inline constexpr IntRange   kMPPISamples     {50,    200,  1};
inline constexpr FloatRange kMPPILambda      {0.5f,  2.0f, 0.1f};
inline constexpr FloatRange kMPPISigma       {0.05f, 0.5f, 0.05f};   // [rad] noise above the tire limit is pure clamping
inline constexpr FloatRange kMPPIRolloutDt   {0.02f, 0.2f, 0.01f};   // [s]
inline constexpr FloatRange kMPPIWeight      {0.1f,  2.0f, 0.1f};    // w_lat, w_heading
inline constexpr IntRange   kMPPISeed        {1,     100,  1};

// MPC
inline constexpr IntRange   kMPCHorizon      {5,      25,    1};
inline constexpr IntRange   kMPCIterations   {5,      100,   1};      // QP solver iterations
inline constexpr FloatRange kMPCWRate        {0.0f,   2.0f,  0.01f};  // steering-rate weight
inline constexpr FloatRange kMPCRolloutDt    {0.02f,  0.2f,  0.01f};  // [s]
inline constexpr FloatRange kMPCWeight       {0.1f,   2.0f,  0.1f};   // w_lat, w_heading
inline constexpr FloatRange kMPCWControl     {0.01f,  0.5f,  0.01f};

// Vehicle model subsystems
inline constexpr FloatRange kCorneringStiffness{2.0f,  12.0f,  0.1f};
inline constexpr FloatRange kCogHeight         {0.2f,  2.5f,   0.05f}; // [m] hi also capped at 0.7 x vehicle_height
inline constexpr FloatRange kSteeringWheelRate {90.0f, 720.0f, 5.0f};  // [deg/s] at the steering wheel
inline constexpr float      kCogHeightShareOfVehicleHeight = 0.7f;

} // namespace tuning
