///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "TemperatureTower.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "BoundingBox.hpp"
#include "ClipperUtils.hpp"
#include "CustomGCode.hpp"
#include "Emboss.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "Model.hpp"
#include "PrintConfig.hpp"
#include "TextConfiguration.hpp"
#include "TriangleMesh.hpp"
#include "Format/AMF.hpp"
#include "format.hpp"

namespace Slic3r {

namespace {

// Model of a floor and font of the labels, relative to the resources directory.
constexpr const char *FLOOR_MODEL = "/calibration/temperature_tower/temperature_tower_floor.amf";
constexpr const char *LABEL_FONT  = "/fonts/NotoSans-Regular.ttf";
// The floor is designed for a 0.4 mm nozzle and scaled with the nozzle diameter if it differs by more than 10 %.
constexpr double DESIGN_NOZZLE_DIAMETER = 0.4;
constexpr double SCALE_TOLERANCE        = 0.1;
// Area of the front face of the floor for the label, below the bridge, in the coordinates of the floor model.
// The front face is at Y = -5, the floor narrows below Z = 1 to separate it from the floor below.
constexpr double LABEL_AREA_MIN_X  = -4.;
constexpr double LABEL_AREA_MAX_X  = 11.;
constexpr double LABEL_AREA_MIN_Z  = 1.;
constexpr double LABEL_AREA_MAX_Z  = 4.6;
// Distance of the label from the edges of the area.
constexpr double LABEL_MARGIN      = 0.3;
// How far the labels stick out of the front face, and how deep they are sunk into it to merge with the floor.
constexpr double LABEL_DEPTH       = 0.6;
constexpr double LABEL_SINK        = 0.2;
// Stroke width of the digits relative to their height, see make_label().
constexpr double LABEL_STROKE      = 0.13;
// Minimum stroke width relative to the nozzle diameter, for the strokes to be printed as distinct features.
constexpr double LABEL_MIN_STROKE  = 1.8;
// Minimum brim width relative to the nozzle diameter, the tower is high and its floors are narrow.
constexpr double MIN_BRIM_WIDTH    = 8.;
constexpr int    MAX_FLOORS        = 25;
constexpr int    MIN_TEMPERATURE   = 100;
constexpr int    MAX_TEMPERATURE   = 500;
// Default half range and step of the temperatures around the filament temperature.
constexpr int    DEFAULT_HALF_RANGE = 20;
constexpr int    DEFAULT_STEP       = 5;

double round_to(double value, int decimals)
{
    const double scale = std::pow(10., decimals);
    return std::round(value * scale) / scale;
}

class LabelFont
{
public:
    explicit LabelFont(const std::string &font_path) : m_font(Emboss::create_font_file(font_path.c_str()))
    {
        if (! m_font.has_value())
            throw InvalidArgument(format(_u8L("Failed to load the font of the labels %1%."), font_path));
    }

    // Outline of the text in font units, see text2shapes().
    ExPolygons shapes(const std::string &text) { return Emboss::text2shapes(m_font, text.c_str(), FontProp()); }

    // Scale of the font units to mm to get digits of the given height.
    double scale(double height)
    {
        const BoundingBox bbox = get_extents(this->shapes("0123456789"));
        return height / double(bbox.size().y());
    }

private:
    Emboss::FontFileWithCache m_font;
};

class TowerBuilder
{
public:
    TowerBuilder(const DynamicPrintConfig &config, const TemperatureTowerParams &params, const std::string &resources_dir);

    Vec3d size() const;
    void  build(Model &model);

private:
    void validate() const;
    void load_floor(const std::string &path);
    void compute_layout();
    // Label of the given floor, on its front face, in the coordinates of the tower.
    indexed_triangle_set make_label(int floor_idx);
    CustomGCode::Info    make_custom_gcodes() const;

    const DynamicPrintConfig     &m_config;
    const TemperatureTowerParams  m_params;
    LabelFont                     m_font;
    double                        m_nozzle_diameter;
    double                        m_layer_height;
    double                        m_first_layer_height;
    int                           m_num_floors;
    // Scale of the floor model.
    double                        m_scale { 1. };
    // Mesh of a floor, scaled, its bounding box starting at the origin. The front face is at Y = 0.
    indexed_triangle_set          m_floor;
    // Size of the scaled floor.
    Vec3d                         m_floor_size;
    // Position of the origin of the floor model in the scaled floor, to place the label.
    Vec3d                         m_floor_origin;
    // Font units to mm.
    double                        m_font_scale { 0. };
};

TowerBuilder::TowerBuilder(const DynamicPrintConfig &config, const TemperatureTowerParams &params, const std::string &resources_dir) :
    m_config(config), m_params(params), m_font(resources_dir + LABEL_FONT)
{
    this->validate();
    this->load_floor(resources_dir + FLOOR_MODEL);
    this->compute_layout();
}

void TowerBuilder::validate() const
{
    const TemperatureTowerParams &p = m_params;
    for (int temp : { p.temp_start, p.temp_end })
        if (temp < MIN_TEMPERATURE || temp > MAX_TEMPERATURE)
            throw InvalidArgument(format(_u8L("The temperatures have to be between %1% and %2% °C."), MIN_TEMPERATURE, MAX_TEMPERATURE));
    if (p.temp_step <= 0)
        throw InvalidArgument(_u8L("The temperature step has to be positive."));
    if (std::abs(p.temp_end - p.temp_start) % p.temp_step != 0)
        throw InvalidArgument(_u8L("The difference of the start and the end temperature has to be a multiple of the step."));
    if (p.num_floors() > MAX_FLOORS)
        throw InvalidArgument(format(_u8L("Too many floors to print, the maximum is %1%. Increase the step or narrow the range."), MAX_FLOORS));
}

void TowerBuilder::load_floor(const std::string &path)
{
    Model                     model;
    DynamicPrintConfig        config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    bool loaded = false;
    try {
        loaded = load_amf(path.c_str(), &config, &substitutions, &model, false);
    } catch (const std::exception &) {
    }
    if (! loaded || model.objects.empty())
        throw InvalidArgument(format(_u8L("Failed to load the model of the temperature tower %1%."), path));
    m_floor = model.objects.front()->raw_indexed_triangle_set();
    if (m_floor.indices.empty())
        throw InvalidArgument(format(_u8L("Failed to load the model of the temperature tower %1%."), path));

    // Scale with the nozzle.
    m_nozzle_diameter = m_config.option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    if (m_nozzle_diameter <= 0.)
        throw InvalidArgument(_u8L("Invalid nozzle diameter."));
    m_scale = m_nozzle_diameter / DESIGN_NOZZLE_DIAMETER;
    if (std::abs(m_scale - 1.) <= SCALE_TOLERANCE)
        m_scale = 1.;
    const BoundingBoxf3 bbox = bounding_box(m_floor);
    m_floor_origin = - m_scale * bbox.min;
    for (stl_vertex &v : m_floor.vertices)
        v = ((v.cast<double>() - bbox.min) * m_scale).cast<float>();
    m_floor_size = m_scale * bbox.size();
}

void TowerBuilder::compute_layout()
{
    const TemperatureTowerParams &p = m_params;
    m_num_floors         = p.num_floors();
    m_layer_height       = m_config.opt_float("layer_height");
    m_first_layer_height = m_config.get_abs_value("first_layer_height", m_layer_height);
    if (m_layer_height <= 0. || m_first_layer_height <= 0.)
        throw InvalidArgument(_u8L("Invalid layer height."));

    if (p.labels) {
        // The digits as high as the label area allows, narrowed down to the area's width for the longest label.
        double height = m_scale * (LABEL_AREA_MAX_Z - LABEL_AREA_MIN_Z - 2. * LABEL_MARGIN);
        m_font_scale = m_font.scale(height);
        double max_width = 0.;
        for (int i = 0; i < m_num_floors; ++ i)
            max_width = std::max(max_width, get_extents(m_font.shapes(std::to_string(p.floor_temperature(i)))).size().x() * m_font_scale);
        if (const double area_width = m_scale * (LABEL_AREA_MAX_X - LABEL_AREA_MIN_X - 2. * LABEL_MARGIN); max_width > area_width)
            m_font_scale *= area_width / max_width;
    }

    // Fits the bed and the printer?
    const Vec3d size = this->size();
    if (const double max_height = m_config.opt_float("max_print_height"); max_height > 0. && size.z() > max_height + EPSILON)
        throw InvalidArgument(format(_u8L("The tower (%1% mm) is higher than the maximum print height of the printer (%2% mm). "
                                          "Reduce the number of floors."),
                                     float_to_string_decimal_point(round_to(size.z(), 1)), float_to_string_decimal_point(max_height)));
    const Pointfs &bed_shape = m_config.option<ConfigOptionPoints>("bed_shape")->values;
    Polygon bed;
    for (const Vec2d &pt : bed_shape)
        bed.points.emplace_back(scaled(pt));
    const Vec2d center = BoundingBoxf(bed_shape).center();
    for (const Vec2d &corner : { Vec2d(-0.5, -0.5), Vec2d(0.5, -0.5), Vec2d(0.5, 0.5), Vec2d(-0.5, 0.5) })
        if (! bed.contains(scaled<coord_t>(Vec2d(center + corner.cwiseProduct(Vec2d(size.x(), size.y()))))))
            throw InvalidArgument(format(_u8L("The tower (%1% x %2% mm) does not fit the print bed."),
                                         float_to_string_decimal_point(round_to(size.x(), 1)), float_to_string_decimal_point(round_to(size.y(), 1))));
}

Vec3d TowerBuilder::size() const
{
    // The labels stick out of the front face.
    return { m_floor_size.x(), m_floor_size.y() + (m_params.labels ? LABEL_DEPTH : 0.), m_num_floors * m_floor_size.z() };
}

indexed_triangle_set TowerBuilder::make_label(int floor_idx)
{
    ExPolygons shapes = m_font.shapes(std::to_string(m_params.floor_temperature(floor_idx)));
    // Thicken the strokes too thin for the nozzle.
    const double height = get_extents(m_font.shapes("0123456789")).size().y() * m_font_scale;
    if (const double thicken = LABEL_MIN_STROKE * m_nozzle_diameter - LABEL_STROKE * height; thicken > 0.)
        shapes = offset_ex(shapes, float(thicken / 2. / m_font_scale));
    const BoundingBox bbox = get_extents(shapes);

    // Extrude the text along Z from 0 to the depth, in mm.
    Emboss::ProjectScale projection(std::make_unique<Emboss::ProjectZ>((LABEL_DEPTH + LABEL_SINK) / m_font_scale), m_font_scale);
    indexed_triangle_set its = Emboss::polygons2model(shapes, projection);

    // Stand the text on the front face, reading along X with the digits' tops up, sticking out towards -Y, centered
    // on the label area of the floor. The mapping (x, y, z) -> (x, -z, y) is a rotation, the triangles keep their
    // orientation.
    const Vec2d  text_min = bbox.min.cast<double>() * m_font_scale;
    const Vec2d  text_max = bbox.max.cast<double>() * m_font_scale;
    const Vec2d  area_center = m_floor_origin.x() * Vec2d::UnitX() + m_floor_origin.z() * Vec2d::UnitY() +
        m_scale * Vec2d(LABEL_AREA_MIN_X + LABEL_AREA_MAX_X, LABEL_AREA_MIN_Z + LABEL_AREA_MAX_Z) / 2.;
    const double x = area_center.x() - (text_min.x() + text_max.x()) / 2.;
    const double z = floor_idx * m_floor_size.z() + area_center.y() - (text_min.y() + text_max.y()) / 2.;
    for (stl_vertex &v : its.vertices)
        v = stl_vertex(float(x + v.x()), float(LABEL_SINK - v.z()), float(z + v.y()));
    return its;
}

CustomGCode::Info TowerBuilder::make_custom_gcodes() const
{
    CustomGCode::Info info;
    info.mode = m_config.option<ConfigOptionFloats>("nozzle_diameter")->size() > 1 ? CustomGCode::MultiAsSingle : CustomGCode::SingleExtruder;
    for (int i = 0; i < m_num_floors; ++ i) {
        // The first layer starting in the floor, the first floor from the second layer of the print, after
        // the switch from the first layer temperature. A layer across the bottom of a floor, in its groove, is printed
        // with the temperature of the floor below.
        const int layer = i == 0 ? 1 : int(std::ceil((i * m_floor_size.z() - m_first_layer_height) / m_layer_height - EPSILON)) + 1;
        CustomGCode::Item item;
        item.print_z  = m_first_layer_height + layer * m_layer_height;
        item.type     = CustomGCode::Custom;
        item.extruder = 1;
        item.extra    = format("M104 S%1% ; temperature tower floor %2%", m_params.floor_temperature(i), i + 1);
        info.gcodes.emplace_back(std::move(item));
    }
    return info;
}

void TowerBuilder::build(Model &model)
{
    model.clear_objects();
    model.clear_materials();

    ModelObject *object = model.add_object();
    object->name = format("Temperature tower %1%-%2%", m_params.floor_temperature(0), m_params.floor_temperature(m_num_floors - 1));
    // A part per floor, the floors touch each other.
    for (int i = 0; i < m_num_floors; ++ i) {
        indexed_triangle_set floor = m_floor;
        its_translate(floor, Vec3f(0.f, 0.f, float(i * m_floor_size.z())));
        ModelVolume *volume = object->add_volume(TriangleMesh(std::move(floor)));
        volume->name = format("%1% °C", m_params.floor_temperature(i));
    }
    if (m_params.labels) {
        indexed_triangle_set labels;
        for (int i = 0; i < m_num_floors; ++ i)
            its_merge(labels, this->make_label(i));
        ModelVolume *volume = object->add_volume(TriangleMesh(std::move(labels)));
        volume->name = "Labels";
    }
    for (ModelVolume *volume : object->volumes)
        volume->config.set_key_value("extruder", new ConfigOptionInt(0));

    // The floors have to start at the layers with the temperature changes.
    object->config.set_key_value("layer_height", new ConfigOptionFloat(m_layer_height));
    object->config.set_key_value("raft_layers", new ConfigOptionInt(0));
    object->config.set_key_value("support_material", new ConfigOptionBool(false));
    // A brim holding the high tower on the bed.
    const double min_brim_width = MIN_BRIM_WIDTH * m_nozzle_diameter;
    if (m_config.option<ConfigOptionEnum<BrimType>>("brim_type")->value == btNoBrim || m_config.opt_float("brim_width") < min_brim_width) {
        object->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
        object->config.set_key_value("brim_width", new ConfigOptionFloat(std::max(min_brim_width, m_config.opt_float("brim_width"))));
    }

    // Centered on the bed, the labels in front of the floors.
    const Vec2d center = BoundingBoxf(m_config.option<ConfigOptionPoints>("bed_shape")->values).center();
    const Vec3d size   = this->size();
    object->add_instance()->set_offset(to_3d(Vec2d(center - Vec2d(size.x(), size.y()) / 2. + Vec2d(0., size.y() - m_floor_size.y())), 0.));
    object->invalidate_bounding_box();

    model.custom_gcode_per_print_z() = this->make_custom_gcodes();
}

} // namespace

int TemperatureTowerParams::num_floors() const
{
    return temp_step > 0 ? std::abs(temp_end - temp_start) / temp_step + 1 : 0;
}

int TemperatureTowerParams::floor_temperature(int floor_idx) const
{
    return temp_start + (temp_end >= temp_start ? 1 : -1) * floor_idx * temp_step;
}

void set_temperature_tower_range(TemperatureTowerParams &params, int center)
{
    // Round to the step, keeping the range within the valid temperatures.
    center = std::clamp(int(std::round(double(center) / DEFAULT_STEP)) * DEFAULT_STEP, MIN_TEMPERATURE + DEFAULT_HALF_RANGE, MAX_TEMPERATURE - DEFAULT_HALF_RANGE);
    params.temp_start = center + DEFAULT_HALF_RANGE;
    params.temp_end   = center - DEFAULT_HALF_RANGE;
    params.temp_step  = DEFAULT_STEP;
}

void create_temperature_tower(Model &model, const DynamicPrintConfig &config, const TemperatureTowerParams &params, const std::string &resources_dir)
{
    TowerBuilder builder(config, params, resources_dir);
    builder.build(model);
}

Vec3d temperature_tower_size(const DynamicPrintConfig &config, const TemperatureTowerParams &params, const std::string &resources_dir)
{
    return TowerBuilder(config, params, resources_dir).size();
}

} // namespace Slic3r
