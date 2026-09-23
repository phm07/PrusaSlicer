#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionMultiplierPattern.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

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
        { "first_layer_height", 0.2 },
        { "nozzle_diameter", "0.4" },
        { "use_relative_e_distances", "1" },
        { "start_gcode", "" },
        { "end_gcode", "" },
    });
    return config;
}

struct Extrusion {
    Vec2d  from;
    Vec2d  to;
    float  z;
    double e;
    double feedrate;
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

// Extruded filament length per square of the given layer. The squares are found by their position, relative to
// the extents of the whole pattern.
std::vector<double> extrusion_per_square(const std::vector<Extrusion> &extrusions, float z, const ExtrusionMultiplierPatternParams &params)
{
    BoundingBoxf bbox;
    for (const Extrusion &ex : extrusions)
        bbox.merge(ex.from), bbox.merge(ex.to);
    const int columns = int(std::ceil(std::sqrt(double(params.num_squares())) - EPSILON));
    const int rows    = (params.num_squares() + columns - 1) / columns;
    const double pitch = params.square_size + params.spacing;
    std::vector<double> out(params.num_squares(), 0.);
    for (const Extrusion &ex : extrusions)
        if (std::abs(ex.z - z) < EPSILON) {
            // Nearest square center, the extrusions at the square edges are half a square away from it.
            const Vec2d mid    = (ex.from + ex.to) / 2. - bbox.min - Vec2d(params.square_size, params.square_size) / 2.;
            const int   column = int(std::round(mid.x() / pitch));
            const int   row    = rows - 1 - int(std::round(mid.y() / pitch));
            const int   idx    = row * columns + column;
            REQUIRE(idx < params.num_squares());
            out[idx] += ex.e;
        }
    return out;
}

std::vector<float> layers(const std::vector<Extrusion> &extrusions)
{
    std::vector<float> out;
    for (const Extrusion &ex : extrusions)
        if (out.empty() || std::abs(out.back() - ex.z) > EPSILON)
            out.emplace_back(ex.z);
    return out;
}

} // namespace

TEST_CASE("Extrusion multiplier pattern: parameters", "[ExtrusionMultiplierPattern]")
{
    ExtrusionMultiplierPatternParams params;

    SECTION("Coarse pass around the filament's value") {
        set_extrusion_multiplier_range(params, 0.95, ExtrusionMultiplierPass::Coarse);
        CHECK(params.em_start == Approx(0.89));
        CHECK(params.em_end == Approx(1.01));
        CHECK(params.num_squares() == 7);
        CHECK(params.square_value(3) == Approx(0.95));
        CHECK(params.square_label(0) == "0.89");
        CHECK(params.square_label(6) == "1.01");
        CHECK(is_extrusion_multiplier_range(params, 0.95, ExtrusionMultiplierPass::Coarse));
        CHECK(! is_extrusion_multiplier_range(params, 0.95, ExtrusionMultiplierPass::Fine));
        CHECK(! is_extrusion_multiplier_range(params, 1., ExtrusionMultiplierPass::Coarse));
    }

    SECTION("Fine pass") {
        set_extrusion_multiplier_range(params, 1., ExtrusionMultiplierPass::Fine);
        CHECK(params.num_squares() == 9);
        CHECK(params.square_label(0) == "0.980");
        CHECK(params.square_label(1) == "0.985");
        CHECK(params.square_label(4) == "1.000");
    }

    SECTION("Labels show the start value exactly") {
        params.em_start = 0.915;
        params.em_end   = 0.955;
        params.em_step  = 0.01;
        CHECK(params.square_label(0) == "0.915");
        CHECK(params.square_label(4) == "0.955");
    }

    SECTION("The range is kept positive") {
        set_extrusion_multiplier_range(params, 0.03, ExtrusionMultiplierPass::Coarse);
        CHECK(params.em_start > 0.);
        CHECK(params.num_squares() == 7);
    }
}

TEST_CASE("Extrusion multiplier pattern: extrusion multipliers", "[ExtrusionMultiplierPattern]")
{
    DynamicPrintConfig               config = test_config();
    ExtrusionMultiplierPatternParams params;
    set_extrusion_multiplier_range(params, 0.95, ExtrusionMultiplierPass::Coarse);
    config.set_deserialize_strict({ { "extrusion_multiplier", "0.95" } });

    const std::string            gcode      = generate_extrusion_multiplier_pattern(config, params);
    const std::vector<Extrusion> extrusions = parse_extrusions(gcode);
    if (debug_files) {
        std::ofstream file("extrusion_multiplier_pattern.gcode");
        file << gcode;
    }

    // The squares have the same geometry, the extruded length scales with the extrusion multiplier of the square.
    for (float z : { 0.2f, 0.4f, 1.0f }) {
        const std::vector<double> e = extrusion_per_square(extrusions, z, params);
        for (int i = 0; i < params.num_squares(); ++ i) {
            INFO("z " << z << " square " << i);
            CHECK(e[i] / e[3] == Approx(params.square_value(i) / 0.95).epsilon(0.002));
        }
    }

    SECTION("The values are absolute, independent of the filament's extrusion multiplier") {
        config.set_deserialize_strict({ { "extrusion_multiplier", "0.8" } });
        const std::vector<double> e_ref = extrusion_per_square(extrusions, 0.4f, params);
        const std::vector<double> e     = extrusion_per_square(parse_extrusions(generate_extrusion_multiplier_pattern(config, params)), 0.4f, params);
        for (int i = 0; i < params.num_squares(); ++ i)
            CHECK(e[i] == Approx(e_ref[i]).epsilon(0.002));
    }

    SECTION("A solid layer fills the square") {
        // Extruded volume of a layer of a square, compared to the volume of the square: the perimeters
        // and the infill are spaced to fill the square at an extrusion multiplier of 1.
        const std::vector<double> e = extrusion_per_square(extrusions, 0.6f, params);
        const double filament_area = PI * 1.75 * 1.75 / 4.;
        const double volume        = e[3] / 0.95 * filament_area;
        CHECK(volume == Approx(params.square_size * params.square_size * 0.2).epsilon(0.03));
    }
}

TEST_CASE("Extrusion multiplier pattern: layout", "[ExtrusionMultiplierPattern]")
{
    DynamicPrintConfig               config = test_config();
    ExtrusionMultiplierPatternParams params;
    set_extrusion_multiplier_range(params, 1., ExtrusionMultiplierPass::Coarse);

    SECTION("Centered on the bed, solid layers and label layers") {
        const std::vector<Extrusion> extrusions = parse_extrusions(generate_extrusion_multiplier_pattern(config, params));
        BoundingBoxf bbox;
        for (const Extrusion &ex : extrusions)
            bbox.merge(ex.from), bbox.merge(ex.to);
        // 7 squares in 3 x 3 grid.
        CHECK(bbox.size().x() == Approx(3 * 30. + 2 * 5.).margin(1.));
        CHECK(bbox.size().y() == Approx(3 * 30. + 2 * 5.).margin(1.));
        CHECK(bbox.center().x() == Approx(125.).margin(0.5));
        CHECK(bbox.center().y() == Approx(105.).margin(0.5));
        const std::vector<float> z = layers(extrusions);
        REQUIRE(z.size() == size_t(params.num_layers + 2));
        CHECK(z.front() == Approx(0.2));
        CHECK(z.back() == Approx(0.2 * (params.num_layers + 2)));

        const Vec2d size = extrusion_multiplier_pattern_size(config, params);
        CHECK(size.x() == Approx(bbox.size().x()).margin(0.01));
    }

    SECTION("Without labels") {
        params.labels = false;
        CHECK(layers(parse_extrusions(generate_extrusion_multiplier_pattern(config, params))).size() == size_t(params.num_layers));
    }

    SECTION("Does not fit the bed") {
        config.set_deserialize_strict({ { "bed_shape", "0x0,80x0,80x80,0x80" } });
        CHECK_THROWS_AS(generate_extrusion_multiplier_pattern(config, params), InvalidArgument);
    }

    SECTION("Squares too small for the labels") {
        params.square_size = 12.;
        CHECK_THROWS_AS(generate_extrusion_multiplier_pattern(config, params), InvalidArgument);
        params.labels = false;
        CHECK_NOTHROW(generate_extrusion_multiplier_pattern(config, params));
    }

    SECTION("Squares too small for the perimeters") {
        params.labels      = false;
        params.square_size = 3.;
        params.perimeters  = 4;
        CHECK_THROWS_AS(generate_extrusion_multiplier_pattern(config, params), InvalidArgument);
    }

    SECTION("Invalid range") {
        params.em_end = params.em_start;
        CHECK_THROWS_AS(generate_extrusion_multiplier_pattern(config, params), InvalidArgument);
    }
}

TEST_CASE("Extrusion multiplier pattern: print settings", "[ExtrusionMultiplierPattern]")
{
    DynamicPrintConfig               config = test_config();
    ExtrusionMultiplierPatternParams params;
    set_extrusion_multiplier_range(params, 1., ExtrusionMultiplierPass::Coarse);
    config.set_deserialize_strict({
        { "first_layer_speed", "21" },
        { "solid_infill_speed", "43" },
        { "top_solid_infill_speed", "37" },
        { "perimeter_speed", "47" },
        { "external_perimeter_speed", "29" },
        { "default_acceleration", "1000" },
        { "top_solid_infill_acceleration", "700" },
        { "first_layer_temperature", "215" },
        { "temperature", "210" },
    });

    SECTION("Speeds, accelerations and temperatures from the presets") {
        const std::string gcode = generate_extrusion_multiplier_pattern(config, params);
        std::map<double, int> feedrates;
        for (const Extrusion &ex : parse_extrusions(gcode))
            ++ feedrates[ex.feedrate];
        for (double speed : { 21., 43., 37., 47., 29. }) {
            INFO("speed " << speed);
            CHECK(feedrates.count(speed * 60.) == 1);
        }
        CHECK(gcode.find("M204 P700") != std::string::npos);
        CHECK(gcode.find("M204 P1000") != std::string::npos);
        CHECK(gcode.find("M109 S215") != std::string::npos);
        CHECK(gcode.find("M104 S210") != std::string::npos);
    }

    SECTION("Volumetric speed limit") {
        config.set_deserialize_strict({ { "filament_max_volumetric_speed", "2" } });
        for (const Extrusion &ex : parse_extrusions(generate_extrusion_multiplier_pattern(config, params)))
            // Cross section of a 0.45 x 0.2 mm extrusion is below 0.09 mm2.
            CHECK(ex.feedrate < 2. / 0.07 * 60.);
    }

    SECTION("Custom G-code") {
        config.set_deserialize_strict({
            { "start_gcode", "START EM={extrusion_multiplier[0]} FILE=[input_filename_base]" },
            { "end_gcode", "END Z={max_layer_z}" },
        });
        const std::string gcode = generate_extrusion_multiplier_pattern(config, params);
        CHECK(gcode.find("START EM=1 FILE=extrusion_multiplier_pattern") != std::string::npos);
        CHECK(gcode.find("END Z=1.4") != std::string::npos);
    }
}

TEST_CASE("Extrusion multiplier pattern: loadable by the G-code viewer", "[ExtrusionMultiplierPattern]")
{
    DynamicPrintConfig               config = test_config();
    ExtrusionMultiplierPatternParams params;
    set_extrusion_multiplier_range(params, 1., ExtrusionMultiplierPass::Fine);
    const std::string gcode = generate_extrusion_multiplier_pattern(config, params);

    const boost::filesystem::path path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("em_pattern_%%%%-%%%%.gcode");
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
}
