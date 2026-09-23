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
#include "libslic3r/FirstLayerPattern.hpp"
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
        { "first_layer_height", 0.25 },
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

BoundingBoxf extents(const std::vector<Extrusion> &extrusions)
{
    BoundingBoxf bbox;
    for (const Extrusion &ex : extrusions)
        bbox.merge(ex.from), bbox.merge(ex.to);
    return bbox;
}

// Extruded filament length per square, the squares found by clustering the extrusions by their position.
// The squares are at least 1 mm apart, the extrusions of a square are connected or less than a square apart.
std::vector<std::pair<BoundingBoxf, double>> squares(const std::vector<Extrusion> &extrusions, double square_size)
{
    std::vector<std::pair<BoundingBoxf, double>> out;
    for (const Extrusion &ex : extrusions) {
        const Vec2d mid = (ex.from + ex.to) / 2.;
        auto it = std::find_if(out.begin(), out.end(), [&mid, square_size](const auto &sq) {
            return (sq.first.center() - mid).cwiseAbs().maxCoeff() < square_size / 2. + 0.5;
        });
        if (it == out.end()) {
            BoundingBoxf bbox(ex.from, ex.to);
            out.emplace_back(bbox, 0.);
            it = std::prev(out.end());
        }
        it->first.merge(ex.from);
        it->first.merge(ex.to);
        it->second += ex.e;
    }
    return out;
}

} // namespace

TEST_CASE("First layer pattern: layout", "[FirstLayerPattern]")
{
    DynamicPrintConfig      config = test_config();
    FirstLayerPatternParams params;

    SECTION("Squares spread over the bed from the margin") {
        const std::string gcode = generate_first_layer_pattern(config, params);
        if (debug_files) {
            std::ofstream file("first_layer_pattern.gcode");
            file << gcode;
        }
        const std::vector<Extrusion> extrusions = parse_extrusions(gcode);
        // A single layer at the first layer height.
        std::set<float> z;
        for (const Extrusion &ex : extrusions)
            z.insert(ex.z);
        REQUIRE(z.size() == 1);
        CHECK(*z.begin() == Approx(0.25));

        // The outer extrusions are half an extrusion width inside the squares.
        const BoundingBoxf bbox = extents(extrusions);
        CHECK(bbox.min.x() == Approx(10.).margin(0.5));
        CHECK(bbox.min.y() == Approx(10.).margin(0.5));
        CHECK(bbox.max.x() == Approx(240.).margin(0.5));
        CHECK(bbox.max.y() == Approx(200.).margin(0.5));

        const auto sq = squares(extrusions, params.square_size);
        REQUIRE(sq.size() == 9);
        const Vec2d size = first_layer_pattern_size(config, params);
        CHECK(size.x() == Approx(bbox.size().x()).margin(0.01));
        CHECK(size.y() == Approx(bbox.size().y()).margin(0.01));
        std::set<std::pair<int, int>> centers;
        for (const auto &[square_bbox, e] : sq) {
            CHECK(square_bbox.size().x() == Approx(params.square_size).margin(0.5));
            CHECK(square_bbox.size().y() == Approx(params.square_size).margin(0.5));
            // The squares are printed the same way.
            CHECK(e == Approx(sq.front().second).epsilon(0.001));
            centers.emplace(int(std::round(square_bbox.center().x())), int(std::round(square_bbox.center().y())));
        }
        CHECK(centers == std::set<std::pair<int, int>>{
            { 25, 25 }, { 125, 25 }, { 225, 25 }, { 25, 105 }, { 125, 105 }, { 225, 105 }, { 25, 185 }, { 125, 185 }, { 225, 185 } });
    }

    SECTION("A square is filled") {
        // Extruded volume of a square compared to its volume, the perimeters and the infill are spaced to fill it.
        const auto sq = squares(parse_extrusions(generate_first_layer_pattern(config, params)), params.square_size);
        const double filament_area = PI * 1.75 * 1.75 / 4.;
        CHECK(sq.front().second * filament_area == Approx(params.square_size * params.square_size * 0.25).epsilon(0.03));
    }

    SECTION("A single column and row are centered") {
        params.columns = 1;
        params.rows    = 1;
        const auto sq = squares(parse_extrusions(generate_first_layer_pattern(config, params)), params.square_size);
        REQUIRE(sq.size() == 1);
        CHECK(sq.front().first.center().x() == Approx(125.).margin(0.1));
        CHECK(sq.front().first.center().y() == Approx(105.).margin(0.1));
    }

    SECTION("Round bed: the squares are pulled towards the center") {
        std::string bed;
        for (int i = 0; i < 72; ++ i) {
            const double a = 2. * PI * i / 72.;
            bed += (i == 0 ? "" : ",") + std::to_string(100. + 100. * std::cos(a)) + "x" + std::to_string(100. + 100. * std::sin(a));
        }
        config.set_deserialize_strict({ { "bed_shape", bed } });
        const std::vector<Extrusion> extrusions = parse_extrusions(generate_first_layer_pattern(config, params));
        const auto sq = squares(extrusions, params.square_size);
        CHECK(sq.size() == 9);
        // The corner squares touch the margin.
        double max_dist = 0.;
        for (const Extrusion &ex : extrusions)
            max_dist = std::max(max_dist, (ex.to - Vec2d(100., 100.)).norm());
        CHECK(max_dist < 100. - params.margin);
        CHECK(max_dist > 100. - params.margin - 1.);
        const BoundingBoxf bbox = extents(extrusions);
        CHECK(bbox.center().x() == Approx(100.).margin(0.1));
        CHECK(bbox.center().y() == Approx(100.).margin(0.1));
    }

    SECTION("Does not fit the bed") {
        params.columns = 8;
        CHECK_THROWS_AS(generate_first_layer_pattern(config, params), InvalidArgument);
        params.columns = 1;
        params.margin  = 100.;
        CHECK_THROWS_AS(generate_first_layer_pattern(config, params), InvalidArgument);
    }

    SECTION("Squares too small for the perimeters") {
        params.square_size = 3.;
        params.perimeters  = 4;
        CHECK_THROWS_AS(generate_first_layer_pattern(config, params), InvalidArgument);
    }

    SECTION("Invalid parameters") {
        params.rows = 0;
        CHECK_THROWS_AS(generate_first_layer_pattern(config, params), InvalidArgument);
        params.rows       = 3;
        params.perimeters = 0;
        CHECK_THROWS_AS(generate_first_layer_pattern(config, params), InvalidArgument);
    }
}

TEST_CASE("First layer pattern: print settings", "[FirstLayerPattern]")
{
    DynamicPrintConfig      config = test_config();
    FirstLayerPatternParams params;
    config.set_deserialize_strict({
        { "first_layer_speed", "21" },
        { "first_layer_infill_speed", "17" },
        { "solid_infill_speed", "43" },
        { "perimeter_speed", "47" },
        { "first_layer_extrusion_width", "0.5" },
        { "default_acceleration", "1000" },
        { "first_layer_acceleration", "600" },
        { "first_layer_temperature", "215" },
        { "temperature", "210" },
        { "first_layer_bed_temperature", "65" },
    });

    SECTION("First layer speeds, width, acceleration and temperatures") {
        const std::string gcode = generate_first_layer_pattern(config, params);
        std::set<double> feedrates;
        for (const Extrusion &ex : parse_extrusions(gcode))
            feedrates.insert(ex.feedrate);
        CHECK(feedrates == std::set<double>{ 17. * 60., 21. * 60. });
        CHECK(gcode.find(";WIDTH:0.5\n") != std::string::npos);
        CHECK(gcode.find("M204 P600") != std::string::npos);
        CHECK(gcode.find("M204 P1000") == std::string::npos);
        CHECK(gcode.find("M109 S215") != std::string::npos);
        CHECK(gcode.find("M190 S65") != std::string::npos);
        // A single layer, the temperatures of the other layers are not set.
        CHECK(gcode.find("S210") == std::string::npos);
    }

    SECTION("Travels between the squares are retracted") {
        config.set_deserialize_strict({ { "retract_length", "0.8" }, { "retract_before_travel", "2" } });
        const std::string gcode = generate_first_layer_pattern(config, params);
        int retractions = 0;
        GCodeReader parser;
        parser.parse_buffer(gcode, [&retractions](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (line.cmd_is("G1") && line.has(Axis::E) && line.e() < 0 && ! line.has(Axis::X))
                ++ retractions;
        });
        // Before the travel to the pattern, between the squares and at the end.
        CHECK(retractions == 1 + 8 + 1);
    }

    SECTION("Custom G-code") {
        config.set_deserialize_strict({
            { "start_gcode", "START FILE=[input_filename_base] LAYERS=[total_layer_count] MIN={first_layer_print_min[0]}" },
            { "end_gcode", "END Z={max_layer_z}" },
        });
        const std::string gcode = generate_first_layer_pattern(config, params);
        CHECK(gcode.find("START FILE=first_layer_pattern LAYERS=1 MIN=10.") != std::string::npos);
        CHECK(gcode.find("END Z=0.25") != std::string::npos);
    }
}

TEST_CASE("First layer pattern: loadable by the G-code viewer", "[FirstLayerPattern]")
{
    const std::string gcode = generate_first_layer_pattern(test_config(), FirstLayerPatternParams());

    const boost::filesystem::path path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("first_layer_pattern_%%%%-%%%%.gcode");
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
