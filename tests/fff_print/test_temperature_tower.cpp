#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TemperatureTower.hpp"

#include "test_data.hpp"

using namespace Slic3r;
using namespace Catch;

constexpr bool debug_files = false;

namespace {

const std::string resources = std::string(TEST_DATA_DIR) + "/../../resources";

DynamicPrintConfig test_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor", "marlin2" },
        { "bed_shape", "0x0,250x0,250x210,0x210" },
        { "layer_height", 0.2 },
        { "first_layer_height", 0.2 },
        { "nozzle_diameter", "0.4" },
        { "first_layer_temperature", "215" },
        { "temperature", "210" },
        { "skirts", 0 },
    });
    return config;
}

TemperatureTowerParams test_params()
{
    TemperatureTowerParams params;
    params.temp_start = 220;
    params.temp_end   = 200;
    params.temp_step  = 10;
    return params;
}

struct Extrusion {
    Vec2d       from;
    Vec2d       to;
    float       z;
    // Last nozzle temperature set.
    int         temperature;
    std::string role;
};

std::vector<Extrusion> parse_extrusions(const std::string &gcode)
{
    std::vector<Extrusion> out;
    int         temperature = 0;
    std::string role;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        float s;
        if ((line.cmd_is("M104") || line.cmd_is("M109")) && line.has_value('S', s))
            temperature = int(s);
        else if (boost::starts_with(line.raw(), ";TYPE:"))
            role = line.raw().substr(6);
        else if (line.cmd_is("G1") && line.has(Axis::E) && line.e() > 0 && line.dist_XY(self) > 0)
            out.push_back({ Vec2d(self.x(), self.y()), Vec2d(line.new_X(self), line.new_Y(self)), self.z(), temperature, role });
    });
    return out;
}

} // namespace

TEST_CASE("Temperature tower: parameters", "[TemperatureTower]")
{
    TemperatureTowerParams params;
    params.temp_start = 230;
    params.temp_end   = 190;
    params.temp_step  = 5;
    CHECK(params.num_floors() == 9);
    CHECK(params.floor_temperature(0) == 230);
    CHECK(params.floor_temperature(8) == 190);
    std::swap(params.temp_start, params.temp_end);
    CHECK(params.floor_temperature(1) == 195);

    set_temperature_tower_range(params, 212);
    CHECK(params.temp_start == 230);
    CHECK(params.temp_end == 190);
    CHECK(params.temp_step == 5);
}

TEST_CASE("Temperature tower: model", "[TemperatureTower]")
{
    DynamicPrintConfig           config = test_config();
    const TemperatureTowerParams params = test_params();
    Model model;
    create_temperature_tower(model, config, params, resources);

    REQUIRE(model.objects.size() == 1);
    const ModelObject &object = *model.objects.front();
    REQUIRE(object.instances.size() == 1);
    // A part per floor and the labels.
    REQUIRE(object.volumes.size() == 4);
    CHECK(object.volumes[0]->name == "220 °C");
    CHECK(object.volumes[2]->name == "200 °C");
    CHECK(object.volumes[3]->name == "Labels");

    SECTION("Closed meshes") {
        for (const ModelVolume *volume : object.volumes) {
            CHECK(volume->mesh().stats().open_edges == 0);
            CHECK(its_volume(volume->mesh().its) > 0.f);
        }
    }

    SECTION("Floors of the model centered on the bed") {
        const BoundingBoxf3 bbox = object.instance_bounding_box(0);
        CHECK(bbox.min.z() == Approx(0.));
        // 3 floors of 10 mm, 44.5 x 10 mm and the labels sticking out by 0.6 mm.
        CHECK(bbox.max.z() == Approx(30.));
        CHECK(bbox.size().x() == Approx(44.49).margin(0.01));
        CHECK(bbox.size().y() == Approx(10.6).margin(0.01));
        const Vec3d size = temperature_tower_size(config, params, resources);
        CHECK(size.x() == Approx(bbox.size().x()).margin(0.01));
        CHECK(size.y() == Approx(bbox.size().y()).margin(0.01));
        CHECK(size.z() == Approx(bbox.size().z()).margin(0.01));
        CHECK(bbox.center().x() == Approx(125.).margin(0.01));
        CHECK(bbox.center().y() == Approx(105.).margin(0.01));
        // The labels on the front face, below the bridge, of each floor.
        auto volume_bbox = [&object](size_t idx) { return object.volumes[idx]->mesh().transformed_bounding_box(object.volumes[idx]->get_matrix()); };
        const BoundingBoxf3 labels = volume_bbox(3);
        const BoundingBoxf3 floors = volume_bbox(0);
        CHECK(labels.min.y() == Approx(floors.min.y() - 0.6).margin(0.01));
        CHECK(labels.min.z() > 1.);
        CHECK(labels.max.z() > 21.);
        CHECK(labels.max.z() < 24.7);
    }

    SECTION("Scaled with the nozzle") {
        config.set_deserialize_strict({ { "nozzle_diameter", "0.6" } });
        const Vec3d size = temperature_tower_size(config, params, resources);
        CHECK(size.x() == Approx(44.49 * 1.5).margin(0.02));
        CHECK(size.z() == Approx(45.));
        // Within 10 %, the floors are not scaled.
        config.set_deserialize_strict({ { "nozzle_diameter", "0.42" } });
        CHECK(temperature_tower_size(config, params, resources).z() == Approx(30.));
    }

    SECTION("Sliced at whole layers without raft and supports, with a brim") {
        const DynamicPrintConfig &object_config = object.config.get();
        CHECK(object_config.opt_float("layer_height") == Approx(0.2));
        CHECK(object_config.opt_int("raft_layers") == 0);
        CHECK(! object_config.opt_bool("support_material"));
        CHECK(object_config.opt_float("brim_width") == Approx(3.2));
        CHECK(object_config.option<ConfigOptionEnum<BrimType>>("brim_type")->value == btOuterOnly);
        // A wider brim of the print preset is kept.
        config.set_deserialize_strict({ { "brim_width", "5" } });
        create_temperature_tower(model, config, params, resources);
        CHECK(! model.objects.front()->config.has("brim_width"));
    }

    SECTION("Custom G-codes setting the temperatures") {
        const CustomGCode::Info &info = model.custom_gcode_per_print_z();
        REQUIRE(info.gcodes.size() == 3);
        const std::vector<double> print_z { 0.4, 10.2, 20.2 };
        const std::vector<int>    temps   { 220, 210, 200 };
        for (size_t i = 0; i < 3; ++ i) {
            CHECK(info.gcodes[i].type == CustomGCode::Custom);
            CHECK(info.gcodes[i].print_z == Approx(print_z[i]));
            CHECK(boost::starts_with(info.gcodes[i].extra, "M104 S" + std::to_string(temps[i]) + " "));
        }
        // With a layer height not dividing the floors, at the first layer starting in the floor.
        config.set_deserialize_strict({ { "layer_height", "0.3" }, { "first_layer_height", "0.25" } });
        create_temperature_tower(model, config, params, resources);
        CHECK(model.custom_gcode_per_print_z().gcodes[1].print_z == Approx(10.15 + 0.3));
    }

    SECTION("Without labels") {
        TemperatureTowerParams no_labels = params;
        no_labels.labels = false;
        create_temperature_tower(model, config, no_labels, resources);
        REQUIRE(model.objects.size() == 1);
        CHECK(model.objects.front()->volumes.size() == 3);
        CHECK(temperature_tower_size(config, no_labels, resources).y() == Approx(10.));
    }
}

TEST_CASE("Temperature tower: sliced", "[TemperatureTower]")
{
    DynamicPrintConfig     config = test_config();
    TemperatureTowerParams params = test_params();
    Model model;
    create_temperature_tower(model, config, params, resources);

    Print print;
    print.apply(model, config);
    print.validate();
    const std::string gcode = Test::gcode(print);
    if (debug_files) {
        std::ofstream file("temperature_tower.gcode");
        file << gcode;
    }
    const std::vector<Extrusion> extrusions = parse_extrusions(gcode);
    REQUIRE(! extrusions.empty());
    auto layer_bbox = [&extrusions](float z, bool brim = false) {
        BoundingBoxf bbox;
        for (const Extrusion &ex : extrusions)
            if (std::abs(ex.z - z) < EPSILON && (brim || ex.role != "Skirt/Brim"))
                bbox.merge(ex.from), bbox.merge(ex.to);
        return bbox;
    };

    SECTION("Temperature of each floor") {
        // The first layer at the filament's first layer temperature, the rest of the first floor at its temperature.
        std::set<int> temperatures;
        for (const Extrusion &ex : extrusions) {
            const int expected = ex.z < 0.3 ? 215 : ex.z < 10.1 ? 220 : ex.z < 20.1 ? 210 : 200;
            if (ex.temperature != expected)
                FAIL("Extrusion at z " << ex.z << " printed at " << ex.temperature << " instead of " << expected);
            temperatures.insert(ex.temperature);
        }
        CHECK(temperatures == std::set<int>{ 200, 210, 215, 220 });
        CHECK(extrusions.back().z == Approx(30.));
    }

    SECTION("Floors separated by a groove") {
        // The bottom of a floor is narrower than the full width slab at the top of the floor below.
        for (float z : { 10.f, 20.f }) {
            const BoundingBoxf slab   = layer_bbox(z - 0.6f);
            const BoundingBoxf bottom = layer_bbox(z + 0.2f);
            CHECK(bottom.min.x() > slab.min.x() + 1.);
            CHECK(bottom.max.x() < slab.max.x() - 1.);
            CHECK(bottom.min.y() > slab.min.y() + 0.1);
            CHECK(bottom.max.y() < slab.max.y() - 0.1);
        }
    }

    SECTION("Bridges") {
        // The top of each floor bridges the window of the floor, 15 mm wide.
        std::set<int> floors;
        for (const Extrusion &ex : extrusions)
            if (ex.role == "Bridge infill" && (ex.to - ex.from).norm() > 14.)
                floors.insert(int(ex.z / 10.));
        CHECK(floors == std::set<int>{ 0, 1, 2 });
    }

    SECTION("Brim") {
        const BoundingBoxf brim  = layer_bbox(0.2f, true);
        const BoundingBoxf tower = layer_bbox(0.2f);
        // 3.2 mm wide from the edge of the tower, measured between the extrusions' center lines.
        CHECK(brim.min.x() < tower.min.x() - 2.5);
    }

    SECTION("Labels in front of the floors") {
        // Extrusions of the labels stick out of the front face, which is the front of the floor at its top.
        const double front = layer_bbox(9.8f).min.y();
        std::set<int> label_floors;
        for (const Extrusion &ex : extrusions)
            if (ex.z > 0.3 && std::min(ex.from.y(), ex.to.y()) < front - 0.3 && ex.role != "Skirt/Brim")
                label_floors.insert(int(ex.z / 10.));
        CHECK(label_floors == std::set<int>{ 0, 1, 2 });
    }
}

TEST_CASE("Temperature tower: invalid parameters", "[TemperatureTower]")
{
    DynamicPrintConfig     config = test_config();
    TemperatureTowerParams params;
    Model model;
    auto create = [&]() { create_temperature_tower(model, config, params, resources); };

    SECTION("Range not a multiple of the step") {
        params.temp_step = 7;
        CHECK_THROWS_AS(create(), InvalidArgument);
    }
    SECTION("Invalid temperatures") {
        params.temp_end = 50;
        CHECK_THROWS_AS(create(), InvalidArgument);
        params.temp_end  = 190;
        params.temp_step = 0;
        CHECK_THROWS_AS(create(), InvalidArgument);
    }
    SECTION("Too many floors") {
        params.temp_start = 300;
        params.temp_end   = 150;
        params.temp_step  = 1;
        CHECK_THROWS_AS(create(), InvalidArgument);
    }
    SECTION("Higher than the printer") {
        config.set_deserialize_strict({ { "max_print_height", "50" } });
        CHECK_THROWS_AS(create(), InvalidArgument);
        params.temp_end = 215;
        CHECK_NOTHROW(create());
    }
    SECTION("Does not fit the bed") {
        config.set_deserialize_strict({ { "bed_shape", "0x0,40x0,40x40,0x40" } });
        CHECK_THROWS_AS(create(), InvalidArgument);
    }
    SECTION("Missing resources") {
        CHECK_THROWS_AS(create_temperature_tower(model, config, params, resources + "/missing"), InvalidArgument);
    }
}
