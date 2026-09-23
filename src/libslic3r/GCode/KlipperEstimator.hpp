///|/ Copyright (c) 2022 Lasse Dalegaard
///|/
///|/ Ported from klipper_estimator (https://github.com/Annex-Engineering/klipper_estimator),
///|/ released under the MIT license.
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCode_KlipperEstimator_hpp_
#define slic3r_GCode_KlipperEstimator_hpp_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "libslic3r/Point.hpp"

namespace Slic3r {
namespace KlipperEstimator {

// Print time estimation for printers running the Klipper firmware, using an implementation of Klipper's
// lookahead planner (junction deviation, smoothed "accel to decel" / "minimum cruise ratio" trapezoids).
// The printer limits are taken from the live printer configuration (see PrinterLimits::from_moonraker_settings()).

struct FirmwareRetractionOptions
{
    double retract_length{ 0. };
    double unretract_extra_length{ 0. };
    double unretract_speed{ 0. };
    double retract_speed{ 0. };
    double lift_z{ 0. };
};

// Additional per-axis or extruder limits applied to each move.
struct MoveChecker
{
    enum class Type { Axis, Extruder };
    Type   type{ Type::Axis };
    // Unit vector of the limited axis, only used by Type::Axis.
    Vec3d  axis{ Vec3d::Zero() };
    double max_velocity{ 0. };
    double max_accel{ 0. };
};

struct PrinterLimits
{
    double                                   max_velocity{ 100. };
    double                                   max_acceleration{ 100. };
    std::optional<double>                    max_accel_to_decel{ 50. };
    std::optional<double>                    minimum_cruise_ratio;
    double                                   square_corner_velocity{ 5. };
    double                                   instant_corner_velocity{ 1. };
    std::optional<FirmwareRetractionOptions> firmware_retraction;
    // [gcode_arcs] resolution, arcs are ignored when not set.
    std::optional<double>                    mm_per_arc_segment;
    std::vector<MoveChecker>                 move_checkers;

    // Derived values, updated by the setters and recalculate().
    double                                   junction_deviation{ 0. };
    double                                   accel_to_decel{ 50. };

    PrinterLimits() { this->recalculate(); }

    void recalculate();
    void set_max_velocity(double v) { max_velocity = v; }
    void set_max_acceleration(double v);
    void set_max_accel_to_decel(double v);
    void set_minimum_cruise_ratio(double v);
    void set_square_corner_velocity(double scv);

    // Compact JSON serialization, used to pass the limits to the background slicing process through the configuration.
    std::string                         serialize() const;
    static std::optional<PrinterLimits> deserialize(const std::string &str);

    // Parse the body of the Moonraker response to GET /printer/objects/query?configfile=settings.
    // Returns std::nullopt and fills in error if the response could not be parsed.
    static std::optional<PrinterLimits> from_moonraker_settings(const std::string &json, std::string *error = nullptr);
};

// Estimates the execution time of each line of a G-code file.
// Feed the lines one by one with process_line(), then call finalize().
class Estimator
{
public:
    explicit Estimator(const PrinterLimits &limits);
    ~Estimator();

    // Processes a single line of G-code (without the trailing new line).
    // Lines are numbered consecutively from 1.
    void process_line(std::string_view line);

    struct Result
    {
        double             total_time{ 0. };
        // Time spent executing each line, indexed by 1-based line number (line_times[0] is unused).
        std::vector<float> line_times;
    };

    // Flushes the planner and returns the result. The estimator must not be used afterwards.
    Result finalize();

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

// Convenience wrapper for estimating a whole G-code string (for example in tests).
Estimator::Result estimate(const PrinterLimits &limits, std::string_view gcode);

} // namespace KlipperEstimator
} // namespace Slic3r

#endif // slic3r_GCode_KlipperEstimator_hpp_
