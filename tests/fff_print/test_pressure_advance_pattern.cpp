#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <fstream>

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>
#include <string>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "LocalesUtils.hpp"
#include "libslic3r/PressureAdvancePattern.hpp"

using namespace Slic3r;
using namespace Catch;

constexpr bool debug_files = false;

namespace {

// Values of all the pressure advance commands with the given prefix, in order.
std::vector<double> pa_values(const std::string &gcode, const std::string &prefix)
{
    std::vector<double> out;
    for (size_t pos = gcode.find(prefix); pos != std::string::npos; pos = gcode.find(prefix, pos + 1))
        if (pos == 0 || gcode[pos - 1] == '\n')
            out.emplace_back(string_to_double_decimal_point(gcode.substr(pos + prefix.size(), gcode.find_first_of(" \n", pos + prefix.size()) - pos - prefix.size())));
    return out;
}

struct Extrusions {
    BoundingBoxf       bbox;
    std::vector<float> z;
    int                count = 0;
};

Extrusions parse_extrusions(const std::string &gcode)
{
    Extrusions out;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&out](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > 0) {
            out.bbox.merge(Vec2d(self.x(), self.y()));
            out.bbox.merge(Vec2d(line.new_X(self), line.new_Y(self)));
            if (out.z.empty() || std::abs(out.z.back() - self.z()) > EPSILON)
                out.z.emplace_back(self.z());
            ++ out.count;
        }
    });
    return out;
}

DynamicPrintConfig klipper_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor", "klipper" },
        { "bed_shape", "0x0,250x0,250x210,0x210" },
        { "layer_height", 0.2 },
        { "first_layer_height", 0.25 },
        { "nozzle_diameter", "0.4" },
        { "start_gcode", "" },
        { "end_gcode", "" },
    });
    return config;
}

} // namespace

TEST_CASE("Pressure advance pattern: pressure advance commands", "[PressureAdvancePattern]")
{
    DynamicPrintConfig           config = klipper_config();
    PressureAdvancePatternParams params;
    REQUIRE(params.num_patterns() == 17);

    const std::string        gcode  = generate_pressure_advance_pattern(config, params);
    const std::vector<double> values = pa_values(gcode, "SET_PRESSURE_ADVANCE ADVANCE=");

    // Start value, start value for the numbering, one per chevron and layer, final reset.
    REQUIRE(values.size() == size_t(1 + 1 + params.num_patterns() * params.num_layers + 1));
    CHECK(values.front() == Approx(0.));
    CHECK(values[1] == Approx(0.));
    for (int layer = 0; layer < params.num_layers; ++ layer)
        for (int i = 0; i < params.num_patterns(); ++ i) {
            // Numbering is printed on the second layer, before the chevrons.
            const size_t idx = 1 + (layer >= 1 ? 1 : 0) + layer * params.num_patterns() + i;
            INFO("layer " << layer << " pattern " << i);
            CHECK(values[idx] == Approx(0.005 * i));
        }
    CHECK(values.back() == Approx(0.));

    // Values are rounded, no floating point noise.
    CHECK(gcode.find("ADVANCE=0.015 ;") != std::string::npos);
    CHECK(gcode.find("0.01500") == std::string::npos);

    if (debug_files) {
        std::ofstream file("pressure_advance_pattern.gcode");
        file << gcode;
    }
    CHECK(gcode.find("M117 PA 0.08\n") != std::string::npos);
}

TEST_CASE("Pressure advance pattern: geometry", "[PressureAdvancePattern]")
{
    DynamicPrintConfig           config = klipper_config();
    PressureAdvancePatternParams params;

    SECTION("Centered on the bed, all layers printed") {
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        const Extrusions  ex    = parse_extrusions(gcode);
        REQUIRE(ex.count > 0);
        CHECK(ex.bbox.min.x() > 0.);
        CHECK(ex.bbox.min.y() > 0.);
        CHECK(ex.bbox.max.x() < 250.);
        CHECK(ex.bbox.max.y() < 210.);
        CHECK(ex.bbox.center().x() == Approx(125.).margin(3.));
        CHECK(ex.bbox.center().y() == Approx(105.).margin(3.));
        REQUIRE(ex.z.size() == size_t(params.num_layers));
        CHECK(ex.z.front() == Approx(0.25));
        CHECK(ex.z.back() == Approx(0.25 + 0.2 * (params.num_layers - 1)));

        const Vec2d size = pressure_advance_pattern_size(config, params);
        CHECK(size.x() == Approx(ex.bbox.size().x()).margin(0.01));
        CHECK(size.y() == Approx(ex.bbox.size().y()).margin(0.01));
    }

    SECTION("Rotated by 90 degrees") {
        const Vec2d size = pressure_advance_pattern_size(config, params);
        params.print_dir = 90.;
        const Vec2d rotated = pressure_advance_pattern_size(config, params);
        CHECK(rotated.x() == Approx(size.y()).margin(0.01));
        CHECK(rotated.y() == Approx(size.x()).margin(0.01));
    }

    SECTION("Anchor layer prints the chevrons from the second layer") {
        params.anchor = PressureAdvancePatternParams::Anchor::Layer;
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        // No numbering reset on the first layer, one command per chevron for the remaining layers.
        CHECK(pa_values(gcode, "SET_PRESSURE_ADVANCE ADVANCE=").size() == size_t(1 + 1 + params.num_patterns() * (params.num_layers - 1) + 1));
        CHECK(parse_extrusions(gcode).z.size() == size_t(params.num_layers));
    }

    SECTION("Does not fit the bed") {
        config.set_deserialize_strict({ { "bed_shape", "0x0,60x0,60x60,0x60" } });
        CHECK_THROWS_AS(generate_pressure_advance_pattern(config, params), InvalidArgument);
    }
}

TEST_CASE("Pressure advance pattern: firmware flavors", "[PressureAdvancePattern]")
{
    DynamicPrintConfig           config = klipper_config();
    PressureAdvancePatternParams params;
    params.show_on_display = false;

    SECTION("Marlin") {
        config.set_deserialize_strict({ { "gcode_flavor", "marlin2" } });
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        CHECK(pa_values(gcode, "M900 K").size() == size_t(3 + params.num_patterns() * params.num_layers));
        CHECK(gcode.find("SET_PRESSURE_ADVANCE") == std::string::npos);
        CHECK(gcode.find("M117") == std::string::npos);
    }
    SECTION("RepRapFirmware") {
        config.set_deserialize_strict({ { "gcode_flavor", "reprapfirmware" } });
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        CHECK(pa_values(gcode, "M572 D0 S").size() == size_t(3 + params.num_patterns() * params.num_layers));
    }
    SECTION("Unsupported flavor") {
        config.set_deserialize_strict({ { "gcode_flavor", "reprap" } });
        CHECK(! pressure_advance_supported(gcfRepRapSprinter));
        CHECK_THROWS_AS(generate_pressure_advance_pattern(config, params), InvalidArgument);
    }
}

TEST_CASE("Pressure advance pattern: custom G-code and temperatures", "[PressureAdvancePattern]")
{
    DynamicPrintConfig           config = klipper_config();
    PressureAdvancePatternParams params;
    config.set_deserialize_strict({
        { "first_layer_temperature", "215" },
        { "temperature", "210" },
        { "first_layer_bed_temperature", "60" },
        { "bed_temperature", "60" },
    });

    SECTION("Temperatures are emitted if the start G-code does not set them") {
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        CHECK(gcode.find("M190 S60") != std::string::npos);
        CHECK(gcode.find("M109 S215") != std::string::npos);
        // Switch to the other layers temperature.
        CHECK(gcode.find("M104 S210") != std::string::npos);
    }

    SECTION("Placeholders are processed, temperatures set by the start G-code are not duplicated") {
        config.set_deserialize_strict({
            { "start_gcode", "PRINT_START BED=[first_layer_bed_temperature] EXTRUDER={first_layer_temperature[initial_extruder]} MIN={first_layer_print_min[0]}\nM190 S[first_layer_bed_temperature]\nM109 S[first_layer_temperature]" },
            { "end_gcode", "PRINT_END Z={max_layer_z}" },
        });
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        CHECK(gcode.find("PRINT_START BED=60 EXTRUDER=215 MIN=") != std::string::npos);
        CHECK(gcode.find("PRINT_END Z=0.85") != std::string::npos);
        // The start G-code sets the temperatures, they are not emitted again.
        CHECK(gcode.find("M190 S60") == gcode.rfind("M190 S60"));
        CHECK(gcode.find("M109 S215") == gcode.rfind("M109 S215"));
        // The start G-code comes before the pattern.
        CHECK(gcode.find("PRINT_START") < gcode.find("SET_PRESSURE_ADVANCE"));
        CHECK(gcode.rfind("SET_PRESSURE_ADVANCE") < gcode.find("PRINT_END"));
    }

    SECTION("Fan is enabled after the disabled first layers") {
        config.set_deserialize_strict({
            { "cooling", "1" },
            { "max_fan_speed", "80" },
            { "disable_fan_first_layers", "2" },
        });
        const std::string gcode = generate_pressure_advance_pattern(config, params);
        const size_t      fan_on = gcode.find("M106 S204");
        REQUIRE(fan_on != std::string::npos);
        CHECK(gcode.find("M107") < gcode.find(";Z:0.25"));
        CHECK(fan_on > gcode.find(";Z:0.45"));
        CHECK(fan_on < gcode.find(";Z:0.65"));
    }

    SECTION("Invalid placeholder is reported") {
        config.set_deserialize_strict({ { "start_gcode", "{nonexistent_variable}" } });
        CHECK_THROWS_AS(generate_pressure_advance_pattern(config, params), InvalidArgument);
    }

    SECTION("Filament pressure advance is restored at the end") {
        config.set_deserialize_strict({ { "filament_pressure_advance", "0.045" } });
        const std::vector<double> values = pa_values(generate_pressure_advance_pattern(config, params), "SET_PRESSURE_ADVANCE ADVANCE=");
        CHECK(values.back() == Approx(0.045));
    }
}

TEST_CASE("Pressure advance pattern: loadable by the G-code viewer", "[PressureAdvancePattern]")
{
    DynamicPrintConfig           config = klipper_config();
    PressureAdvancePatternParams params;
    config.set_key_value("printhost_apikey", new ConfigOptionString("secret"));
    const std::string gcode = generate_pressure_advance_pattern(config, params);

    CHECK(gcode.find("; prusaslicer_config = begin\n") != std::string::npos);
    CHECK(gcode.find("; prusaslicer_config = end\n") != std::string::npos);
    CHECK(gcode.find("secret") == std::string::npos);

    const boost::filesystem::path path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("pa_pattern_%%%%-%%%%.gcode");
    {
        boost::nowide::ofstream file(path.string(), std::ios::binary);
        file << gcode;
    }
    GCodeProcessor processor;
    REQUIRE_NOTHROW(processor.process_file(path.string()));
    const GCodeProcessorResult result = processor.extract_result();
    boost::filesystem::remove(path);
    CHECK(! result.moves.empty());
    // Layers of the chevrons.
    CHECK(result.print_statistics.modes[0].time > 0.f);
}

TEST_CASE("Pressure advance pattern: extruder type presets", "[PressureAdvancePattern]")
{
    DynamicPrintConfig config = klipper_config();
    // Prusa MINI sized bed.
    config.set_deserialize_strict({ { "bed_shape", "0x0,180x0,180x180,0x180" } });

    for (const char *flavor : { "klipper", "marlin2", "reprapfirmware" })
        for (PressureAdvanceExtruderType type : { PressureAdvanceExtruderType::DirectDrive, PressureAdvanceExtruderType::Bowden }) {
            config.set_deserialize_strict({ { "gcode_flavor", flavor } });
            const GCodeFlavor gcode_flavor = config.option<ConfigOptionEnum<GCodeFlavor>>("gcode_flavor")->value;
            PressureAdvancePatternParams params;
            set_pressure_advance_range(params, gcode_flavor, type);
            INFO("flavor " << flavor << " bowden " << (type == PressureAdvanceExtruderType::Bowden));
            CHECK(is_pressure_advance_range(params, gcode_flavor, type));
            CHECK(params.num_patterns() > 10);
            CHECK(params.num_patterns() <= 21);
            CHECK_NOTHROW(generate_pressure_advance_pattern(config, params));
        }

    PressureAdvancePatternParams params;
    set_pressure_advance_range(params, gcfKlipper, PressureAdvanceExtruderType::Bowden);
    CHECK(params.pa_end > 0.5);
    CHECK(! is_pressure_advance_range(params, gcfKlipper, PressureAdvanceExtruderType::DirectDrive));
    CHECK(! is_pressure_advance_range(params, gcfMarlinFirmware, PressureAdvanceExtruderType::Bowden));
}
