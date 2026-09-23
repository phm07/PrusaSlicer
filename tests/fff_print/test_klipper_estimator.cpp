#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <boost/filesystem.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/KlipperEstimator.hpp"
#include "libslic3r/Utils.hpp"
#include "test_data.hpp"

using namespace Slic3r;
using namespace Slic3r::KlipperEstimator;
using Catch::Approx;

// Limits without the "accel to decel" smoothing, so that the moves follow plain trapezoids.
static PrinterLimits trapezoid_limits()
{
    PrinterLimits limits;
    limits.set_max_velocity(100.);
    limits.set_max_acceleration(1000.);
    limits.set_minimum_cruise_ratio(0.);
    limits.set_square_corner_velocity(5.);
    return limits;
}

TEST_CASE("Klipper estimator: single moves", "[KlipperEstimator]") {
    PrinterLimits limits = trapezoid_limits();

    SECTION("Straight move accelerates, cruises and decelerates") {
        // 0.1 s to accelerate over 5 mm, 90 mm cruise at 100 mm/s, 0.1 s to decelerate.
        const Estimator::Result result = estimate(limits, "G1 X100 F6000\n");
        CHECK(result.total_time == Approx(1.1));
        REQUIRE(result.line_times.size() == 2);
        CHECK(result.line_times[1] == Approx(1.1));
    }

    SECTION("Feed rate is capped by max_velocity") {
        CHECK(estimate(limits, "G1 X100 F60000\n").total_time == Approx(1.1));
    }

    SECTION("Minimum cruise ratio limits the peak velocity of short moves") {
        limits.set_minimum_cruise_ratio(0.5);
        // Peak velocity^2 is limited to accel * (1 - ratio) * distance = 5000.
        const double v = std::sqrt(5000.);
        const double expected = 2. * (v / 1000.) + (10. - 2. * 2.5) / v;
        CHECK(estimate(limits, "G1 X10 F6000\n").total_time == Approx(expected));
    }

    SECTION("Square corner velocity limits the junction speed of a 90 degree corner") {
        // Each move: accelerate 0 -> 100 (5 mm, 0.1 s), decelerate 100 -> 5 (4.9875 mm, 0.095 s), cruise the rest.
        const double move_time = 0.1 + 0.095 + (100. - 5. - 4.9875) / 100.;
        const Estimator::Result result = estimate(limits, "G1 X100 F6000\nG1 Y100\n");
        CHECK(result.total_time == Approx(2. * move_time));
        CHECK(result.line_times[1] == Approx(move_time));
        CHECK(result.line_times[2] == Approx(move_time));
    }

    SECTION("Extrude only moves are limited by the extruder limits") {
        limits.move_checkers.push_back({ MoveChecker::Type::Extruder, Vec3d::Zero(), 10., 1000. });
        // Accelerate to 10 mm/s in 0.01 s over 0.05 mm, decelerate the same.
        CHECK(estimate(limits, "G1 E10 F6000\n").total_time == Approx(0.01 + 0.99 + 0.01));
    }

    SECTION("Axis limits slow down moves along that axis") {
        limits.move_checkers.push_back({ MoveChecker::Type::Axis, Vec3d::UnitZ(), 10., 100. });
        // 0.1 s to accelerate to 10 mm/s over 0.5 mm, same to decelerate, 9 mm cruise.
        CHECK(estimate(limits, "G1 Z10 F6000\n").total_time == Approx(0.1 + 0.9 + 0.1));
        // X moves are not affected.
        CHECK(estimate(limits, "G1 X100 F6000\n").total_time == Approx(1.1));
    }
}

TEST_CASE("Klipper estimator: G-code handling", "[KlipperEstimator]") {
    PrinterLimits limits = trapezoid_limits();

    SECTION("Dwell and indeterminate operations") {
        const Estimator::Result result = estimate(limits, "G4 P500\nM109 S200\nG28\n; ESTIMATOR_ADD_TIME 12.5 Heating\nM104 S200\n");
        CHECK(result.line_times[1] == Approx(0.5));
        CHECK(result.line_times[2] == Approx(0.1));
        CHECK(result.line_times[3] == Approx(0.1));
        CHECK(result.line_times[4] == Approx(12.5));
        CHECK(result.line_times[5] == 0.f);
        CHECK(result.total_time == Approx(13.2));
    }

    SECTION("Relative and absolute positioning") {
        // Two relative moves of 50 mm each, then back to X0 in absolute mode.
        const double t50 = estimate(limits, "G1 X50 F6000\n").total_time;
        const double t100 = estimate(limits, "G1 X100 F6000\n").total_time;
        const Estimator::Result result = estimate(limits, "G91\nG1 X50 F6000\nG4 P0\nG1 X50\nG90\nG4 P0\nG1 X0\n");
        CHECK(result.line_times[2] == Approx(t50));
        CHECK(result.line_times[4] == Approx(t50));
        CHECK(result.line_times[7] == Approx(t100));
    }

    SECTION("Acceleration changes") {
        // 0.05 s to accelerate to 100 mm/s over 2.5 mm, 95 mm cruise.
        CHECK(estimate(limits, "M204 S2000\nG1 X100 F6000\n").total_time == Approx(1.05));
        CHECK(estimate(limits, "M204 P2000 T3000\nG1 X100 F6000\n").total_time == Approx(1.05));
        CHECK(estimate(limits, "SET_VELOCITY_LIMIT ACCEL=2000 ; comment\nG1 X100 F6000\n").total_time == Approx(1.05));
        // 0.05 s to accelerate to 50 mm/s over 1.25 mm, 97.5 mm cruise.
        CHECK(estimate(limits, "SET_VELOCITY_LIMIT VELOCITY=50\nG1 X100 F6000\n").total_time == Approx(0.05 + 1.95 + 0.05));
    }

    SECTION("Comments, line numbers and unknown commands") {
        const Estimator::Result result = estimate(limits, ";TYPE:Perimeter\nN10 G1 X100 F6000 ; move\nMY_MACRO A=1\n{garbage}\n");
        REQUIRE(result.line_times.size() == 5);
        CHECK(result.line_times[2] == Approx(1.1));
        CHECK(result.total_time == Approx(1.1));
    }

    SECTION("Arcs are ignored without gcode_arcs") {
        CHECK(estimate(limits, "G2 X10 Y0 I5 J0 F6000\n").total_time == 0.);
    }

    SECTION("Arcs are segmented with gcode_arcs") {
        limits.mm_per_arc_segment = 0.1;
        // Half circle of radius 50 at 10 mm/s: the corners of the segments are negligible.
        const Estimator::Result result = estimate(limits, "G1 X0 Y0 F600\nG2 X100 Y0 I50 J0\n");
        CHECK(result.line_times[2] == Approx(50. * PI / 10.).epsilon(0.01));
    }

    SECTION("Firmware retraction") {
        const std::string gcode = "G1 X10 F6000\nG10\nG1 X20\nG11\nG1 X30\n";
        // Without [firmware_retraction], G10 / G11 are ignored and the moves form a single 30 mm move.
        CHECK(estimate(limits, gcode).total_time == Approx(estimate(limits, "G1 X30 F6000\n").total_time));
        limits.firmware_retraction = FirmwareRetractionOptions{ 1., 0.5, 10., 20., 0. };
        const Estimator::Result result = estimate(limits, gcode);
        // Retract 1 mm at 20 mm/s, unretract 1.5 mm at 10 mm/s, extrude only moves without extruder limits accelerate instantly.
        CHECK(result.line_times[2] == Approx(1. / 20.));
        CHECK(result.line_times[4] == Approx(1.5 / 10.));
        // The toolhead stops for the extrude only moves.
        const double t10 = estimate(limits, "G1 X10 F6000\n").total_time;
        CHECK(result.total_time == Approx(3. * t10 + 1. / 20. + 1.5 / 10.));
    }
}

static const std::string moonraker_response = R"({"result": {"eventtime": 1234.5, "status": {"configfile": {"settings": {
    "printer": {"kinematics": "cartesian", "max_velocity": 300.0, "max_accel": 3000.0, "minimum_cruise_ratio": 0.5,
                "square_corner_velocity": 5.0, "max_z_velocity": 15.0, "max_z_accel": 100.0},
    "extruder": {"max_extrude_only_velocity": 120.0, "max_extrude_only_accel": 1500.0, "instantaneous_corner_velocity": 1.0,
                 "nozzle_diameter": 0.4},
    "firmware_retraction": {"retract_length": 0.8, "retract_speed": 35.0, "unretract_extra_length": 0.0, "unretract_speed": 30.0},
    "gcode_arcs": {"resolution": 0.1},
    "gcode_macro print_start": {"gcode": "G28"}
}}}}})";

TEST_CASE("Klipper estimator: printer limits", "[KlipperEstimator]") {
    SECTION("Parsed from the Moonraker printer settings") {
        std::string error;
        std::optional<PrinterLimits> limits = PrinterLimits::from_moonraker_settings(moonraker_response, &error);
        REQUIRE(limits.has_value());
        CHECK(error.empty());
        CHECK(limits->max_velocity == 300.);
        CHECK(limits->max_acceleration == 3000.);
        CHECK(limits->accel_to_decel == Approx(1500.));
        CHECK(limits->square_corner_velocity == 5.);
        CHECK(limits->instant_corner_velocity == 1.);
        REQUIRE(limits->mm_per_arc_segment.has_value());
        CHECK(*limits->mm_per_arc_segment == Approx(0.1));
        REQUIRE(limits->firmware_retraction.has_value());
        CHECK(limits->firmware_retraction->retract_length == Approx(0.8));
        CHECK(limits->firmware_retraction->lift_z == 0.);
        REQUIRE(limits->move_checkers.size() == 2);
        CHECK(limits->move_checkers[0].type == MoveChecker::Type::Axis);
        CHECK(limits->move_checkers[0].axis == Vec3d::UnitZ());
        CHECK(limits->move_checkers[0].max_velocity == 15.);
        CHECK(limits->move_checkers[1].type == MoveChecker::Type::Extruder);
        CHECK(limits->move_checkers[1].max_accel == 1500.);
    }

    SECTION("Older Klipper versions use max_accel_to_decel") {
        std::string response = moonraker_response;
        response.replace(response.find("\"minimum_cruise_ratio\": 0.5"), 27, "\"max_accel_to_decel\": 1000");
        std::optional<PrinterLimits> limits = PrinterLimits::from_moonraker_settings(response);
        REQUIRE(limits.has_value());
        CHECK(limits->accel_to_decel == Approx(1000.));
    }

    SECTION("Invalid responses are rejected") {
        std::string error;
        CHECK(!PrinterLimits::from_moonraker_settings("not json", &error).has_value());
        CHECK(!error.empty());
        CHECK(!PrinterLimits::from_moonraker_settings(R"({"result": {"status": {"configfile": {"settings": {}}}}})").has_value());
        CHECK(!PrinterLimits::deserialize("").has_value());
        CHECK(!PrinterLimits::deserialize("{}").has_value());
    }

    SECTION("Serialization round trip") {
        const PrinterLimits limits = *PrinterLimits::from_moonraker_settings(moonraker_response);
        const std::string serialized = limits.serialize();
        CHECK(serialized.find('\n') == std::string::npos);
        const std::optional<PrinterLimits> restored = PrinterLimits::deserialize(serialized);
        REQUIRE(restored.has_value());
        CHECK(restored->serialize() == serialized);
        CHECK(restored->accel_to_decel == Approx(limits.accel_to_decel));
        CHECK(restored->junction_deviation == Approx(limits.junction_deviation));
        CHECK(restored->move_checkers.size() == limits.move_checkers.size());
        const std::string gcode = "G1 X100 Y20 F12000\nG1 Z5\nG1 E-1\nG2 X0 Y20 I-50 J0\nG10\nG11\n";
        CHECK(estimate(*restored, gcode).total_time == Approx(estimate(limits, gcode).total_time));
    }
}

TEST_CASE("Klipper estimator: G-code export", "[KlipperEstimator]") {
    DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor", "klipper" },
        { "remaining_times", "1" },
    });

    auto export_gcode = [&config](GCodeProcessorResult &result) {
        Print print;
        Model model;
        Test::init_print({ Test::TestMesh::cube_20x20x20 }, print, model, config);
        const boost::filesystem::path temp = boost::filesystem::unique_path();
        print.set_status_silent();
        print.process();
        print.export_gcode(temp.string(), &result, nullptr);
        boost::nowide::ifstream t(temp.string(), std::ios::binary);
        std::string gcode((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        t.close();
        boost::nowide::remove(temp.string().c_str());
        return std::make_pair(gcode, print.print_statistics());
    };
    static constexpr size_t Normal = size_t(PrintEstimatedStatistics::ETimeMode::Normal);

    SECTION("Without printer limits, the default estimate is used") {
        GCodeProcessorResult result;
        const auto [gcode, stats] = export_gcode(result);
        CHECK(!result.print_statistics.klipper_estimate);
        CHECK(!stats.klipper_estimate);
    }

    SECTION("With printer limits, the Klipper estimate is used") {
        GCodeProcessorResult default_result;
        export_gcode(default_result);

        const PrinterLimits limits = *PrinterLimits::from_moonraker_settings(moonraker_response);
        config.set("klipper_estimator_limits", limits.serialize());
        GCodeProcessorResult result;
        const auto [gcode, stats] = export_gcode(result);

        REQUIRE(result.print_statistics.klipper_estimate);
        CHECK(stats.klipper_estimate);
        // The limits are not a setting, they are not stored into the G-code.
        CHECK(gcode.find("klipper_estimator_limits") == std::string::npos);

        // The estimate matches the one of the final G-code.
        const Estimator::Result estimate = KlipperEstimator::estimate(limits, gcode);
        const float time = result.print_statistics.modes[Normal].time;
        CHECK(time == Approx(estimate.total_time).epsilon(1e-5));
        CHECK(time != Approx(default_result.print_statistics.modes[Normal].time));
        CHECK(stats.estimated_normal_print_time == get_time_dhms(time));
        CHECK(gcode.find("; estimated printing time (normal mode) = " + get_time_dhms(time) + "\n") != std::string::npos);

        // The time is distributed to the moves.
        double moves_time = 0.;
        for (const GCodeProcessorResult::MoveVertex &move : result.moves)
            moves_time += move.time[Normal];
        CHECK(moves_time == Approx(time).epsilon(1e-4));

        // The lines M73 follow the new estimate.
        const size_t first_m73 = gcode.find("M73 P");
        REQUIRE(first_m73 != std::string::npos);
        CHECK(gcode.substr(first_m73, gcode.find('\n', first_m73) - first_m73) == "M73 P0 R" + std::to_string(int((time + 0.5f) / 60.f)));
        CHECK(gcode.find("M73 P100 R0\n") != std::string::npos);

        // The ends of lines match the rewritten file.
        REQUIRE(result.lines_ends.size() == 1);
        CHECK(result.lines_ends.front().back() == gcode.size());
        CHECK(result.lines_ends.front().size() == size_t(std::count(gcode.begin(), gcode.end(), '\n')));
    }
}
