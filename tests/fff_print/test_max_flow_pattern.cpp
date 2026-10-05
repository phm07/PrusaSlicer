#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/MaxFlowPattern.hpp"

using namespace Slic3r;
using namespace Catch;

constexpr bool debug_files = false;

namespace {

DynamicPrintConfig test_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor", "marlin2" },
        { "bed_shape", "0x0,250x0,250x210,0x210" },
        { "layer_height", 0.2 },
        { "first_layer_height", 0.25 },
        { "nozzle_diameter", "0.4" },
        { "filament_diameter", "1.75" },
        { "use_relative_e_distances", "1" },
        { "start_gcode", "" },
        { "end_gcode", "" },
    });
    return config;
}

const double filament_area = PI * 1.75 * 1.75 / 4.;

struct Extrusion {
    Vec2d  from;
    Vec2d  to;
    float  z;
    double e;
    double feedrate;

    double length() const { return (to - from).norm(); }
    // Volumetric flow in mm^3/s.
    double flow() const { return e * filament_area / length() * feedrate / 60.; }
};

// Extrusions with relative E distances.
std::vector<Extrusion> parse_extrusions(const std::string &gcode)
{
    std::vector<Extrusion> out;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&out](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("G1") && line.has(Axis::E) && line.e() > 0 && line.dist_XY(self) > 0)
            out.push_back({ Vec2d(self.x(), self.y()), Vec2d(line.new_X(self), line.new_Y(self)), self.z(), line.e(), line.new_F(self) });
    });
    return out;
}

BoundingBoxf extents(const std::vector<Extrusion> &extrusions)
{
    BoundingBoxf bbox;
    for (const Extrusion &ex : extrusions)
        bbox.merge(ex.from), bbox.merge(ex.to);
    return bbox;
}

// Extrusions of the wall, above the first layer, grouped by layer.
std::map<float, std::vector<Extrusion>> wall_layers(const std::vector<Extrusion> &extrusions, float first_layer_height)
{
    std::map<float, std::vector<Extrusion>> out;
    for (const Extrusion &ex : extrusions)
        if (ex.z > first_layer_height + 0.001f)
            out[ex.z].push_back(ex);
    return out;
}

size_t count(const std::string &gcode, const std::string &what)
{
    size_t n = 0;
    for (size_t pos = gcode.find(what); pos != std::string::npos; pos = gcode.find(what, pos + 1))
        ++ n;
    return n;
}

} // namespace

TEST_CASE("Max flow pattern: bands", "[MaxFlowPattern]")
{
    DynamicPrintConfig   config = test_config();
    MaxFlowPatternParams params;
    REQUIRE(params.num_bands() == 16);

    const std::string gcode = generate_max_flow_pattern(config, params);
    if (debug_files) {
        std::ofstream file("max_flow_pattern.gcode");
        file << gcode;
    }
    const std::vector<Extrusion> extrusions = parse_extrusions(gcode);
    const auto layers = wall_layers(extrusions, 0.25f);

    SECTION("A first layer base and ten layers per band") {
        std::set<float> first_layer;
        for (const Extrusion &ex : extrusions)
            if (ex.z < 0.26f)
                first_layer.insert(ex.z);
        REQUIRE(first_layer.size() == 1);
        CHECK(*first_layer.begin() == Approx(0.25));
        REQUIRE(layers.size() == 16 * 10);
        CHECK(layers.begin()->first == Approx(0.45));
        CHECK(layers.rbegin()->first == Approx(0.25 + 160 * 0.2));
    }

    SECTION("Each band is printed at its volumetric flow") {
        int layer = 0;
        for (const auto &[z, layer_extrusions] : layers) {
            const int band = layer ++ / 10;
            for (const Extrusion &ex : layer_extrusions)
                REQUIRE(ex.flow() == Approx(params.band_flow(band)).epsilon(0.005));
        }
        CHECK(params.band_flow(15) == Approx(20.));
    }

    SECTION("Stadium centered on the bed, with straight walls along X") {
        for (const auto &[z, layer_extrusions] : layers) {
            const BoundingBoxf bbox = extents(layer_extrusions);
            REQUIRE(bbox.min.x() == Approx(125. - 50. - 10.).margin(0.001));
            REQUIRE(bbox.max.x() == Approx(125. + 50. + 10.).margin(0.001));
            REQUIRE(bbox.min.y() == Approx(105. - 10.).margin(0.001));
            REQUIRE(bbox.max.y() == Approx(105. + 10.).margin(0.001));
            // The layer is a closed loop starting and ending at its leftmost point.
            REQUIRE(layer_extrusions.front().from.isApprox(Vec2d(65., 105.)));
            REQUIRE(layer_extrusions.back().to.isApprox(Vec2d(65., 105.)));
            const size_t straight = std::count_if(layer_extrusions.begin(), layer_extrusions.end(),
                [](const Extrusion &ex) { return std::abs(ex.length() - 100.) < 0.001 && std::abs(ex.from.y() - ex.to.y()) < 0.001; });
            REQUIRE(straight == 2);
        }
        // The base is wider than the wall on both sides.
        const BoundingBoxf bbox = extents(extrusions);
        CHECK(bbox.min.y() < 94.);
        CHECK(bbox.max.y() > 116.);
        CHECK(bbox.center().x() == Approx(125.).margin(0.001));
        CHECK(bbox.center().y() == Approx(105.).margin(0.001));
        const Vec2d size = max_flow_pattern_size(config, params);
        CHECK(size.x() == Approx(bbox.size().x()).margin(0.01));
        CHECK(size.y() == Approx(bbox.size().y()).margin(0.01));
    }

    SECTION("No retractions on the layer changes") {
        config.set_deserialize_strict({ { "retract_length", "0.8" }, { "retract_layer_change", "1" } });
        int retractions = 0;
        GCodeReader parser;
        parser.parse_buffer(generate_max_flow_pattern(config, params), [&retractions](GCodeReader &, const GCodeReader::GCodeLine &line) {
            if (line.cmd_is("G1") && line.has(Axis::E) && line.e() < 0 && ! line.has(Axis::X))
                ++ retractions;
        });
        // Before the travel to the pattern and at the end.
        CHECK(retractions == 2);
    }

    SECTION("The flow is shown on the display at the start of each band") {
        CHECK(count(gcode, "\nM117 Flow ") == 16);
        const size_t first = gcode.find("\nM117 Flow 5 mm3/s\n");
        REQUIRE(first != std::string::npos);
        CHECK(gcode.find("\nM117 Flow 20 mm3/s\n") != std::string::npos);
        // After the first layer.
        CHECK(first > gcode.find(";Z:0.45\n"));
        params.show_on_display = false;
        CHECK(generate_max_flow_pattern(config, params).find("M117") == std::string::npos);
    }

    SECTION("Band table in the header") {
        CHECK(gcode.find("; Maximum volumetric flow calibration pattern\n") != std::string::npos);
        CHECK(gcode.find(";  - Flow start: 5 mm3/s, end: 20 mm3/s, step: 1 mm3/s, bands: 16\n") != std::string::npos);
        const size_t last = gcode.find(";    16     30.25 - 32.25 ");
        REQUIRE(last != std::string::npos);
        const std::string line = gcode.substr(last, gcode.find('\n', last) - last);
        CHECK(line.find(" 20 ") != std::string::npos);
        // Speed of the last band, the flow divided by the extrusion cross section.
        CHECK(line.find(" 245.") != std::string::npos);
    }
}

TEST_CASE("Max flow pattern: print settings", "[MaxFlowPattern]")
{
    DynamicPrintConfig   config = test_config();
    MaxFlowPatternParams params;

    SECTION("The volumetric speed limits apply to the base only") {
        config.set_deserialize_strict({ { "max_volumetric_speed", "3" }, { "filament_max_volumetric_speed", "4" } });
        const std::vector<Extrusion> extrusions = parse_extrusions(generate_max_flow_pattern(config, params));
        double max_base_flow = 0.;
        for (const Extrusion &ex : extrusions)
            if (ex.z < 0.26f)
                max_base_flow = std::max(max_base_flow, ex.flow());
        CHECK(max_base_flow == Approx(3.).epsilon(0.005));
        const auto layers = wall_layers(extrusions, 0.25f);
        CHECK(layers.rbegin()->second.front().flow() == Approx(20.).epsilon(0.005));
    }

    SECTION("The extrusion multiplier is applied on top of the flow") {
        config.set_deserialize_strict({ { "extrusion_multiplier", "0.9" } });
        const auto layers = wall_layers(parse_extrusions(generate_max_flow_pattern(config, params)), 0.25f);
        CHECK(layers.begin()->second.front().flow() == Approx(5. * 0.9).epsilon(0.005));
    }

    SECTION("Custom layer height, extrusion width and band height") {
        params.layer_height    = 0.3;
        params.extrusion_width = 0.6;
        params.band_height     = 2.;
        const MaxFlowPatternLayout layout = max_flow_pattern_layout(config, params);
        CHECK(layout.layers_per_band == 7);
        CHECK(layout.band_height == Approx(2.1));
        CHECK(layout.extrusion_width == Approx(0.6));
        CHECK(layout.total_height == Approx(0.25 + 16 * 2.1));
        const std::string gcode = generate_max_flow_pattern(config, params);
        CHECK(gcode.find(";WIDTH:0.6\n") != std::string::npos);
        const auto layers = wall_layers(parse_extrusions(gcode), 0.25f);
        REQUIRE(layers.size() == 16 * 7);
        CHECK(layers.begin()->first == Approx(0.55));
        CHECK(layers.rbegin()->second.front().flow() == Approx(20.).epsilon(0.005));
    }

    SECTION("Wall acceleration of the solid infill") {
        config.set_deserialize_strict({
            { "machine_limits_usage", "ignore" },
            { "default_acceleration", "1000" },
            { "first_layer_acceleration", "500" },
            { "solid_infill_acceleration", "3000" },
        });
        const std::string gcode = generate_max_flow_pattern(config, params);
        const size_t base = gcode.find("M204 P500");
        REQUIRE(base != std::string::npos);
        CHECK(gcode.find("M204 P3000") > base);
    }
}

TEST_CASE("Max flow pattern: machine limits", "[MaxFlowPattern]")
{
    DynamicPrintConfig   config = test_config();
    MaxFlowPatternParams params;
    // The last band at 20 mm^3/s is printed at about 246 mm/s.
    REQUIRE_NOTHROW(generate_max_flow_pattern(config, params));

    SECTION("Maximum feedrate of X") {
        config.set_deserialize_strict({ { "machine_max_feedrate_x", "200,200" } });
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        params.flow_end = 16.;
        CHECK_NOTHROW(generate_max_flow_pattern(config, params));
        params.flow_end = 20.;
        config.set_deserialize_strict({ { "machine_limits_usage", "ignore" } });
        CHECK_NOTHROW(generate_max_flow_pattern(config, params));
    }

    SECTION("Maximum feedrate of E") {
        config.set_deserialize_strict({ { "machine_max_feedrate_e", "5,5" } });
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
    }

    SECTION("Acceleration too low for the length of the straight walls") {
        config.set_deserialize_strict({ { "machine_max_acceleration_extruding", "1000,1000" } });
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        CHECK_THROWS_AS(max_flow_pattern_size(config, params), InvalidArgument);
        params.length = 125.;
        CHECK_NOTHROW(generate_max_flow_pattern(config, params));
    }

    SECTION("Acceleration of the print preset") {
        config.set_deserialize_strict({ { "machine_limits_usage", "ignore" }, { "default_acceleration", "1000" } });
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        config.set_deserialize_strict({ { "solid_infill_acceleration", "2000" } });
        CHECK_NOTHROW(generate_max_flow_pattern(config, params));
        // Unknown acceleration, not checked.
        config.set_deserialize_strict({ { "default_acceleration", "0" } });
        params.length = 20.;
        CHECK_NOTHROW(generate_max_flow_pattern(config, params));
    }
}

TEST_CASE("Max flow pattern: invalid parameters", "[MaxFlowPattern]")
{
    DynamicPrintConfig   config = test_config();
    MaxFlowPatternParams params;

    SECTION("Flow range") {
        params.flow_end = params.flow_start;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        params.flow_end  = 20.;
        params.flow_step = 0.;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        params.flow_step = 0.1;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
    }

    SECTION("Layer height above the maximum of the nozzle") {
        params.layer_height = 0.35;
        CHECK_THROWS_AS(max_flow_pattern_layout(config, params), InvalidArgument);
        config.set_deserialize_strict({ { "max_layer_height", "0.36" } });
        CHECK_NOTHROW(max_flow_pattern_layout(config, params));
    }

    SECTION("Extrusion width") {
        params.extrusion_width = 0.1;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        params.extrusion_width = 1.3;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
    }

    SECTION("Taller than the maximum print height") {
        config.set_deserialize_strict({ { "max_print_height", "30" } });
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
    }

    SECTION("Does not fit the bed") {
        config.set_deserialize_strict({ { "machine_limits_usage", "ignore" } });
        params.length = 240.;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
        params.length = 10.;
        CHECK_THROWS_AS(generate_max_flow_pattern(config, params), InvalidArgument);
    }
}

TEST_CASE("Max flow pattern: flow at a measured height", "[MaxFlowPattern]")
{
    MaxFlowPatternParams       params;
    const MaxFlowPatternLayout layout = max_flow_pattern_layout(test_config(), params);
    CHECK(layout.band_height == Approx(2.));
    CHECK(max_flow_at_height(params, layout, 0.1) == Approx(5.));
    CHECK(max_flow_at_height(params, layout, 2.2) == Approx(5.));
    CHECK(max_flow_at_height(params, layout, 2.25) == Approx(6.));
    CHECK(max_flow_at_height(params, layout, 2.3) == Approx(6.));
    CHECK(max_flow_at_height(params, layout, 31.) == Approx(20.));
    CHECK(max_flow_at_height(params, layout, 100.) == Approx(20.));
}

TEST_CASE("Max flow pattern: loadable by the G-code viewer", "[MaxFlowPattern]")
{
    const std::string gcode = generate_max_flow_pattern(test_config(), MaxFlowPatternParams());

    const boost::filesystem::path path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("max_flow_pattern_%%%%-%%%%.gcode");
    {
        boost::nowide::ofstream file(path.string(), std::ios::binary);
        file << gcode;
    }
    GCodeProcessor processor;
    REQUIRE_NOTHROW(processor.process_file(path.string()));
    const GCodeProcessorResult result = processor.extract_result();
    boost::filesystem::remove(path);
    CHECK(! result.moves.empty());
    CHECK(result.print_statistics.modes[0].time > 0.f);
    float max_z = 0.f;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves)
        max_z = std::max(max_z, move.position.z());
    CHECK(max_z >= 32.25f - 0.001f);
}
