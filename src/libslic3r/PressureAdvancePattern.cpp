///|/ Pressure advance calibration pattern, ported from Ellis' Pressure Advance / Linear Advance Calibration Tool
///|/ Copyright (C) 2019 Sineos [https://github.com/Sineos]
///|/ Copyright (C) 2022 AndrewEllis93 [https://github.com/AndrewEllis93]
///|/ https://github.com/AndrewEllis93/Pressure_Linear_Advance_Tool, licensed under the GPLv3 or later.
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PressureAdvancePattern.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>

#include "BoundingBox.hpp"
#include "Exception.hpp"
#include "ExtrusionRole.hpp"
#include "GCode.hpp"
#include "GCode/GCodeProcessor.hpp"
#include "GCode/GCodeWriter.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "PlaceholderParser.hpp"
#include "Polygon.hpp"
#include "Utils.hpp"
#include "format.hpp"

namespace Slic3r {

namespace {

constexpr double GLYPH_PADDING_HORIZONTAL = 1.;
constexpr double GLYPH_PADDING_VERTICAL   = 1.;
// Overlap of the first layer of the chevrons with the anchor frame, in line widths.
constexpr double ENCROACHMENT             = 1. / 3.;
// Digits of the pressure advance labels, see draw_number().
constexpr double GLYPH_SEGMENT_LENGTH     = 2.;
constexpr double GLYPH_DOT_SIZE           = 0.75;
constexpr double GLYPH_SPACING            = 3.;
constexpr double GLYPH_SPACING_NARROW     = 1.;
// Z height to travel at from the end of the start G-code to the pattern.
constexpr double SAFE_TRAVEL_Z            = 5.;

double round_to(double value, int decimals)
{
    const double scale = std::pow(10., decimals);
    return std::round(value * scale) / scale;
}

double deg2rad(double deg) { return deg * PI / 180.; }

// Cross section of an extrusion with a rectangular shape and semicircular ends, see Flow::mm3_per_mm().
double extrusion_area(double width, double height) { return (width - height) * height + PI * 0.25 * height * height; }
// Distance of neighbor extrusions, see Flow::spacing().
double extrusion_spacing(double width, double height) { return width - height * (1. - 0.25 * PI); }

std::string pa_label(double value, bool no_leading_zero)
{
    std::string out = float_to_string_decimal_point(value);
    if (no_leading_zero && out.size() > 1 && out[0] == '0' && out[1] == '.')
        out.erase(0, 1);
    return out;
}

// Height (along Y, the digits are stacked) of the pressure advance label.
double label_height(const std::string &label)
{
    double height = 0.;
    for (char c : label)
        height += (c == '1' || c == '.') ? GLYPH_SPACING_NARROW : GLYPH_SPACING;
    return height;
}

// Pressure advance values and filament overrides of retraction parameters applied over the full print config,
// the same way Print::apply() does.
DynamicPrintConfig config_with_filament_overrides(const DynamicPrintConfig &config)
{
    DynamicPrintConfig out = config;
    for (const std::string &opt_key : print_config_def.extruder_retract_keys()) {
        const ConfigOption *opt          = config.option(opt_key);
        const ConfigOption *opt_filament = config.option("filament_" + opt_key);
        if (opt != nullptr && opt_filament != nullptr && ! opt_filament->is_nil()) {
            ConfigOption *opt_copy = opt->clone();
            opt_copy->apply_override(opt_filament);
            out.set_key_value(opt_key, opt_copy);
        }
    }
    return out;
}

class PatternGenerator
{
public:
    PatternGenerator(const DynamicPrintConfig &config, const PressureAdvancePatternParams &params) :
        m_full_config(config_with_filament_overrides(config)), m_params(params)
    {
        m_config.apply(m_full_config, true);
        m_flavor = m_config.gcode_flavor.value;

        this->validate();

        m_nozzle_diameter     = m_config.nozzle_diameter.get_at(0);
        m_layer_height        = m_full_config.opt_float("layer_height");
        m_first_layer_height  = m_config.first_layer_height.get_abs_value(m_layer_height);
        m_line_width          = m_nozzle_diameter * params.line_ratio / 100.;
        m_line_width_anchor   = m_nozzle_diameter * params.anchor_line_ratio / 100.;
        m_line_spacing        = extrusion_spacing(m_line_width, m_layer_height);
        m_line_spacing_anchor = extrusion_spacing(m_line_width_anchor, m_first_layer_height);
        m_half_angle          = deg2rad(params.corner_angle) / 2.;
        m_line_spacing_angle  = m_line_spacing / std::sin(m_half_angle);
        m_num_patterns        = params.num_patterns();

        const BoundingBoxf bed_bbox(m_config.bed_shape.values);
        m_center = bed_bbox.center();
        for (const Vec2d &pt : m_config.bed_shape.values)
            m_bed.points.emplace_back(scaled(pt));

        this->compute_layout();
    }

    std::string generate();
    // Size of the pattern after rotation, valid after generate().
    Vec2d size() const { return m_pattern_size; }

private:
    void validate() const;
    void compute_layout();
    void reset_state();
    Vec2d start_point() const;
    // Anchor, labels and chevrons of all layers. Expects the print head at the first layer, at start_point().
    void emit_pattern();

    void init_placeholder_parser();
    std::string process_template(const std::string &name, const std::string &templ, const DynamicConfig *config_override);

    std::string header() const;
    std::string set_pressure_advance(double value, const std::string_view comment) const;

    // Pattern coordinates (before rotation around the bed center by print_dir) to bed coordinates.
    Vec2d to_bed(const Vec2d &pt) const;

    void set_role(GCodeExtrusionRole role);
    void set_width(double width);
    void layer_change(double print_z, double height);
    void move_to(const Vec2d &pt, bool allow_retract = true);
    void draw_line(const Vec2d &pt, double width, double height, double speed);
    void draw_box(double min_x, double min_y, double size_x, double size_y, bool fill, int num_perimeters);
    void draw_number(double start_x, double start_y, const std::string &label, double height);

    DynamicPrintConfig                 m_full_config;
    PrintConfig                        m_config;
    const PressureAdvancePatternParams m_params;
    GCodeFlavor                        m_flavor;

    double  m_nozzle_diameter;
    double  m_layer_height;
    double  m_first_layer_height;
    double  m_line_width;
    double  m_line_width_anchor;
    double  m_line_spacing;
    double  m_line_spacing_anchor;
    double  m_half_angle;
    double  m_line_spacing_angle;
    int     m_num_patterns;
    Vec2d   m_center;
    Polygon m_bed;

    // Layout of the pattern, in pattern coordinates.
    double  m_frame_size_y;
    double  m_print_size_x;
    double  m_pattern_shift;
    double  m_pattern_start_x;
    double  m_pattern_start_y;
    double  m_tab_max_x;
    double  m_tab_size_y;
    std::vector<std::string> m_labels;
    Vec2d   m_pattern_size { Vec2d::Zero() };

    // Fan speed in percent, above the layers with the fan disabled.
    int     m_fan_speed { 0 };
    // Fan speed of the given layer, respecting disable_fan_first_layers.
    int     fan_speed(int layer) const { return layer < m_config.disable_fan_first_layers.get_at(0) ? 0 : m_fan_speed; }

    GCodeWriter        m_writer;
    PlaceholderParser  m_placeholder_parser;
    PlaceholderParser::ContextData m_placeholder_context;
    DynamicConfig      m_placeholder_outputs;

    std::string        m_gcode;
    // Current position in pattern coordinates.
    Vec2d              m_pos { Vec2d::Zero() };
    double             m_print_z { 0. };
    double             m_last_width { -1. };
    // Feed rate of the last extrusion, invalidated by travels, which set their own feed rate.
    double             m_last_speed { -1. };
    GCodeExtrusionRole m_role { GCodeExtrusionRole::None };
    BoundingBoxf       m_extrusion_bbox;
};

void PatternGenerator::validate() const
{
    const PressureAdvancePatternParams &p = m_params;
    if (! pressure_advance_supported(m_flavor))
        throw InvalidArgument(_u8L("The pressure advance calibration requires the Klipper, Marlin or RepRapFirmware G-code flavor. "
                                   "Change the G-code flavor in Printer Settings > General > Firmware."));
    if (p.pa_start < 0. || p.pa_step <= 0. || p.pa_end <= p.pa_start)
        throw InvalidArgument(_u8L("The pressure advance end value has to be larger than the start value and the step has to be positive."));
    if (p.num_patterns() > 100)
        throw InvalidArgument(_u8L("Too many patterns to print, increase the pressure advance step or narrow the range."));
    if (p.num_layers < 1 || (p.anchor == PressureAdvancePatternParams::Anchor::Layer && p.num_layers < 2))
        throw InvalidArgument(_u8L("The pattern needs at least one layer above the anchor layer."));
    if (p.wall_count < 1 || p.wall_side_length <= 0. || p.pattern_spacing < 0. || p.line_ratio <= 0. || p.anchor_line_ratio <= 0.)
        throw InvalidArgument(_u8L("Invalid pattern dimensions."));
    if (p.corner_angle <= 0. || p.corner_angle >= 180.)
        throw InvalidArgument(_u8L("The corner angle has to be between 0 and 180 degrees."));
    if (p.anchor != PressureAdvancePatternParams::Anchor::None && p.anchor_perimeters < 1)
        throw InvalidArgument(_u8L("The anchor needs at least one perimeter."));
    if (p.first_layer_speed <= 0. || p.perimeter_speed <= 0. || p.acceleration < 0.)
        throw InvalidArgument(_u8L("Invalid print speed or acceleration."));
    if (m_config.nozzle_diameter.empty() || m_config.nozzle_diameter.get_at(0) <= 0. || m_config.filament_diameter.get_at(0) <= 0.)
        throw InvalidArgument(_u8L("Invalid nozzle or filament diameter."));
}

void PatternGenerator::compute_layout()
{
    const PressureAdvancePatternParams &p = m_params;

    m_labels.clear();
    double max_label_height = 0.;
    for (int i = 0; i < m_num_patterns; ++ i) {
        m_labels.emplace_back(pa_label(p.pattern_value(i), p.no_leading_zero));
        // A label is printed below every other chevron.
        if (i % 2 == 0)
            max_label_height = std::max(max_label_height, label_height(m_labels.back()));
    }

    double object_size_x = m_num_patterns * ((p.wall_count - 1) * m_line_spacing_angle) +
                           (m_num_patterns - 1) * (p.pattern_spacing + m_line_width) +
                           std::cos(m_half_angle) * p.wall_side_length;
    // The frame is grown to the right, to prevent the last chevron from running over it.
    if (p.anchor == PressureAdvancePatternParams::Anchor::Frame)
        object_size_x += m_line_spacing_anchor * p.anchor_perimeters;

    m_frame_size_y = 2. * std::sin(m_half_angle) * p.wall_side_length;
    m_tab_size_y   = max_label_height + m_line_spacing_anchor + GLYPH_PADDING_VERTICAL * 2.;
    const double object_size_y = m_frame_size_y + (p.number_tab ? m_line_spacing_anchor + m_tab_size_y : 0.);

    // Shift the chevrons to the right if the label of the first one would stick out of the frame.
    m_pattern_shift = 0.;
    if (p.number_tab) {
        const double shift = GLYPH_PADDING_HORIZONTAL - (((p.wall_count - 1) / 2.) * m_line_spacing_angle - GLYPH_SEGMENT_LENGTH);
        if (shift > 0.)
            m_pattern_shift = shift + m_line_width_anchor / 2.;
    }
    m_print_size_x    = object_size_x + m_pattern_shift;
    m_pattern_start_x = m_center.x() - m_print_size_x / 2.;
    m_pattern_start_y = m_center.y() - object_size_y / 2.;

    // The tab below the labels spans from the frame start to the end of the last label.
    const int last_label = (m_num_patterns - 1) - (m_num_patterns - 1) % 2;
    m_tab_max_x = m_pattern_start_x + m_pattern_shift +
                  last_label * (p.pattern_spacing + m_line_width + (p.wall_count - 1) * m_line_spacing_angle) +
                  ((p.wall_count - 1) / 2.) * m_line_spacing_angle - GLYPH_SEGMENT_LENGTH +
                  2. * GLYPH_SEGMENT_LENGTH + GLYPH_PADDING_HORIZONTAL + m_line_width_anchor / 2.;
}

Vec2d PatternGenerator::to_bed(const Vec2d &pt) const
{
    // Clockwise rotation around the bed center.
    const double a = deg2rad(m_params.print_dir);
    const double c = std::cos(a);
    const double s = std::sin(a);
    const Vec2d  d = pt - m_center;
    return { c * d.x() + s * d.y() + m_center.x(), c * d.y() - s * d.x() + m_center.y() };
}

void PatternGenerator::set_role(GCodeExtrusionRole role)
{
    if (role != m_role) {
        m_role = role;
        m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) + gcode_extrusion_role_to_string(role) + "\n";
    }
}

void PatternGenerator::set_width(double width)
{
    if (std::abs(width - m_last_width) > EPSILON) {
        m_last_width = width;
        m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) + float_to_string_decimal_point(width) + "\n";
    }
}

void PatternGenerator::layer_change(double print_z, double height)
{
    m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) + "\n";
    m_gcode += ";Z:" + float_to_string_decimal_point(print_z) + "\n";
    m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + float_to_string_decimal_point(height) + "\n";
    m_print_z = print_z;
    m_gcode += m_writer.travel_to_z(print_z, "move to next layer");
    m_last_speed = -1.;
}

void PatternGenerator::move_to(const Vec2d &pt, bool allow_retract)
{
    if ((pt - m_pos).norm() < EPSILON)
        return;
    const bool   retract = allow_retract && (pt - m_pos).norm() > m_config.retract_before_travel.get_at(0);
    const double lift    = m_config.retract_lift.get_at(0);
    if (retract) {
        m_gcode += m_writer.retract();
        if (lift > 0.)
            m_gcode += m_writer.travel_to_z(m_print_z + lift, "lift Z");
    }
    m_gcode += m_writer.travel_to_xy(to_bed(pt));
    m_pos        = pt;
    m_last_speed = -1.;
    if (retract) {
        if (lift > 0.)
            m_gcode += m_writer.travel_to_z(m_print_z, "restore layer Z");
        m_gcode += m_writer.unretract();
    }
}

void PatternGenerator::draw_line(const Vec2d &pt, double width, double height, double speed)
{
    const double length = (pt - m_pos).norm();
    if (length < EPSILON)
        return;
    set_width(width);
    const Vec2d from = to_bed(m_pos);
    const Vec2d to   = to_bed(pt);
    m_extrusion_bbox.merge(from);
    m_extrusion_bbox.merge(to);
    if (speed != m_last_speed) {
        m_gcode += m_writer.set_speed(speed * 60.);
        m_last_speed = speed;
    }
    m_gcode += m_writer.extrude_to_xy(to, m_writer.extruder()->e_per_mm(extrusion_area(width, height)) * length);
    m_pos = pt;
}

// Draw perimeters from the outside in, optionally fill the inside with diagonal lines.
// The box is printed with the anchor extrusion width at the first layer.
void PatternGenerator::draw_box(double min_x, double min_y, double size_x, double size_y, bool fill, int num_perimeters)
{
    const double width   = m_line_width_anchor;
    const double height  = m_first_layer_height;
    const double speed   = m_params.first_layer_speed;
    const double spacing = extrusion_spacing(width, height);

    // Limit the number of perimeters to what fits into the box.
    num_perimeters = std::min<int>(num_perimeters, int(std::floor(std::min(size_x, size_y) * 0.5 / spacing)));
    num_perimeters = std::max(num_perimeters, 1);

    double x = min_x;
    double y = min_y;
    move_to({ x, y });
    for (int i = 0; i < num_perimeters; ++ i) {
        if (i != 0) {
            // Step inwards to print the next perimeter.
            x += spacing;
            y += spacing;
            move_to({ x, y });
        }
        const double sx = size_x - i * spacing * 2.;
        const double sy = size_y - i * spacing * 2.;
        draw_line({ x,      y + sy }, width, height, speed);
        draw_line({ x + sx, y + sy }, width, height, speed);
        draw_line({ x + sx, y      }, width, height, speed);
        draw_line({ x,      y      }, width, height, speed);
    }

    if (fill) {
        // 45 degrees lines x + y = c inside the innermost perimeter, overlapping it by the encroachment.
        const double inset      = spacing * (num_perimeters - 1) + width * (1. - ENCROACHMENT);
        const double x_min      = min_x + inset;
        const double x_max      = min_x + size_x - inset;
        const double y_min      = min_y + inset;
        const double y_max      = min_y + size_y - inset;
        const double spacing_45 = spacing / std::sin(deg2rad(45.));
        bool         reverse    = false;
        for (double c = x_min + y_min + spacing_45; c < x_max + y_max - EPSILON; c += spacing_45) {
            // Intersection of the line with the bottom or right edge and with the left or top edge.
            Vec2d a { 0., std::max(y_min, c - x_max) };
            a.x() = c - a.y();
            Vec2d b { std::max(x_min, c - y_max), 0. };
            b.y() = c - b.x();
            if (reverse)
                std::swap(a, b);
            // Short connecting travel, don't retract.
            move_to(a, false);
            draw_line(b, width, height, speed);
            reverse = ! reverse;
        }
    }
}

// Draw a seven-segment like label. The digits are stacked along the Y axis, to be read with the pattern rotated
// by 90 degrees. Each digit is a sequence of segments starting at one of the corners of the digit's cell.
void PatternGenerator::draw_number(double start_x, double start_y, const std::string &label, double height)
{
    static const std::map<char, std::vector<std::string>> glyphs {
        { '1', { "bl", "right", "right" } },
        { '2', { "bl", "up", "right", "down", "right", "up" } },
        { '3', { "bl", "up", "right", "down", "mup", "right", "down" } },
        { '4', { "ul", "right", "right", "mleft", "down", "left" } },
        { '5', { "ul", "down", "right", "up", "right", "down" } },
        { '6', { "ul", "down", "right", "right", "up", "left", "down" } },
        { '7', { "bl", "up", "right", "right" } },
        { '8', { "bl", "right", "right", "up", "left", "left", "down", "mright", "up" } },
        { '9', { "br", "up", "left", "left", "down", "right", "up" } },
        { '0', { "bl", "right", "right", "up", "left", "left", "down" } },
        { '.', { "br", "dot" } },
    };

    const double seg   = GLYPH_SEGMENT_LENGTH;
    const double width = m_line_width;
    const double speed = m_params.first_layer_speed;
    double       offset = 0.;
    for (char c : label) {
        auto it = glyphs.find(c);
        if (it == glyphs.end())
            continue;
        for (const std::string &s : it->second) {
            const double y = start_y + offset;
            if      (s == "bl")     move_to({ start_x,           y       });
            else if (s == "br")     move_to({ start_x + seg * 2, y       });
            else if (s == "ul")     move_to({ start_x,           y + seg });
            else if (s == "up")     draw_line(m_pos + Vec2d(0., seg),  width, height, speed);
            else if (s == "down")   draw_line(m_pos - Vec2d(0., seg),  width, height, speed);
            else if (s == "right")  draw_line(m_pos + Vec2d(seg, 0.),  width, height, speed);
            else if (s == "left")   draw_line(m_pos - Vec2d(seg, 0.),  width, height, speed);
            else if (s == "mup")    move_to(m_pos + Vec2d(0., seg));
            else if (s == "mright") move_to(m_pos + Vec2d(seg, 0.));
            else if (s == "mleft")  move_to(m_pos - Vec2d(seg, 0.));
            else if (s == "dot")    draw_line(m_pos - Vec2d(GLYPH_DOT_SIZE, 0.), width, height, speed);
        }
        offset += (c == '1' || c == '.') ? GLYPH_SPACING_NARROW : GLYPH_SPACING;
    }
}

std::string PatternGenerator::set_pressure_advance(double value, const std::string_view comment) const
{
    std::string out = set_pressure_advance_gcode(m_flavor, value);
    if (! comment.empty()) {
        out.pop_back();
        out += " ; ";
        out += comment;
        out += "\n";
    }
    if (m_params.show_on_display)
        out += (m_flavor == gcfMarlinLegacy || m_flavor == gcfMarlinFirmware ? "M117 LA " : "M117 PA ") + float_to_string_decimal_point(value) + "\n";
    return out;
}

void PatternGenerator::init_placeholder_parser()
{
    // Mirror the variables GCodeGenerator::_do_export() provides to the start / end G-code of a sliced print.
    PlaceholderParser &pp = m_placeholder_parser;
    pp.apply_config(m_full_config);
    pp.update_timestamp();
    for (const char *key : { "print_settings_id", "filament_settings_id", "printer_settings_id", "physical_printer_settings_id" })
        if (const ConfigOption *opt = m_full_config.option(key); opt != nullptr) {
            std::string name = key;
            pp.set(name.substr(0, name.size() - strlen("_settings_id")) + "_preset", opt->clone());
        }
    pp.set("input_filename_base", "pressure_advance_pattern");
    pp.set("num_objects", 1);
    pp.set("num_instances", 1);
    pp.set("scale", std::vector<std::string>{ "x:100% y:100% z:100%" });
    pp.set("initial_tool", 0);
    pp.set("initial_extruder", 0);
    pp.set("current_extruder", 0);
    pp.set("total_layer_count", m_params.num_layers);
    pp.set("current_object_idx", 0);
    pp.set("has_wipe_tower", false);
    pp.set("has_single_extruder_multi_material_priming", false);
    pp.set("total_toolchanges", 0);
    {
        const BoundingBoxf bbox(m_config.bed_shape.values);
        pp.set("print_bed_min",  new ConfigOptionFloats({ bbox.min.x(), bbox.min.y() }));
        pp.set("print_bed_max",  new ConfigOptionFloats({ bbox.max.x(), bbox.max.y() }));
        pp.set("print_bed_size", new ConfigOptionFloats({ bbox.size().x(), bbox.size().y() }));
    }
    {
        // The first layer is known only after the pattern was generated, see generate().
        const BoundingBoxf &bbox = m_extrusion_bbox;
        pp.set("first_layer_print_convex_hull", new ConfigOptionPoints({ bbox.min, { bbox.max.x(), bbox.min.y() }, bbox.max, { bbox.min.x(), bbox.max.y() } }));
        pp.set("first_layer_print_min",  new ConfigOptionFloats({ bbox.min.x(), bbox.min.y() }));
        pp.set("first_layer_print_max",  new ConfigOptionFloats({ bbox.max.x(), bbox.max.y() }));
        pp.set("first_layer_print_size", new ConfigOptionFloats({ bbox.size().x(), bbox.size().y() }));
    }
    const size_t num_extruders = m_config.nozzle_diameter.size();
    pp.set("num_extruders", int(num_extruders));
    std::vector<unsigned char> is_extruder_used(std::max(size_t(255), num_extruders), 0);
    is_extruder_used[0] = true;
    pp.set("is_extruder_used", new ConfigOptionBools(is_extruder_used));
    pp.set("extruded_volume", new ConfigOptionFloats(num_extruders, 0.));
    pp.set("extruded_weight", new ConfigOptionFloats(num_extruders, 0.));
    pp.set("extruded_volume_total", new ConfigOptionFloat(0.));
    pp.set("extruded_weight_total", new ConfigOptionFloat(0.));
    pp.set("zhop", new ConfigOptionFloat(0.));

    // Variables the custom G-code may write to.
    m_placeholder_outputs.set_key_value("position", new ConfigOptionFloats({ 0., 0., 0. }));
    m_placeholder_outputs.set_key_value("e_retracted", new ConfigOptionFloats(num_extruders, 0.));
    m_placeholder_outputs.set_key_value("e_restart_extra", new ConfigOptionFloats(num_extruders, 0.));
    if (! m_config.use_relative_e_distances)
        m_placeholder_outputs.set_key_value("e_position", new ConfigOptionFloats(num_extruders, 0.));

    m_placeholder_context.rng = std::mt19937(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    m_placeholder_context.global_config = std::make_unique<DynamicConfig>();
}

std::string PatternGenerator::process_template(const std::string &name, const std::string &templ, const DynamicConfig *config_override)
{
    if (templ.empty())
        return {};
    ConfigOptionFloats *opt_position = m_placeholder_outputs.option<ConfigOptionFloats>("position");
    const Vec3d pos = m_writer.get_position();
    opt_position->values = { pos.x(), pos.y(), pos.z() };
    std::string out;
    try {
        out = m_placeholder_parser.process(templ, 0, config_override, &m_placeholder_outputs, &m_placeholder_context);
    } catch (std::runtime_error &err) {
        throw InvalidArgument(format(_u8L("Failed to process the custom G-code template %1%:\n%2%"), name, err.what()));
    }
    // The custom G-code may have moved the print head.
    if (const std::vector<double> &new_pos = opt_position->values; new_pos.size() == 3 && Vec3d(new_pos[0], new_pos[1], new_pos[2]) != pos)
        m_writer.update_position({ new_pos[0], new_pos[1], new_pos[2] });
    if (! out.empty() && out.back() != '\n')
        out += '\n';
    return out;
}

std::string PatternGenerator::header() const
{
    const PressureAdvancePatternParams &p = m_params;
    auto on_off = [](bool v) { return v ? "true" : "false"; };
    const char *anchor = p.anchor == PressureAdvancePatternParams::Anchor::Frame ? "frame" :
                         p.anchor == PressureAdvancePatternParams::Anchor::Layer ? "layer" : "none";
    std::string values;
    for (int i = 0; i < m_num_patterns; ++ i)
        values += (i == 0 ? "" : ", ") + float_to_string_decimal_point(p.pattern_value(i));

    std::string out;
    out += "; " + header_slic3r_generated() + "\n";
    out += ";\n";
    out += "; Pressure advance calibration pattern\n";
    out += "; Based on Ellis' Pressure Advance / Linear Advance Calibration Tool\n";
    out += ";   Original Marlin linear advance calibration tool by Sineos [https://github.com/Sineos]\n";
    out += ";   Heavily modified/rewritten by Andrew Ellis [https://github.com/AndrewEllis93]\n";
    out += ";   https://github.com/AndrewEllis93/Pressure_Linear_Advance_Tool\n";
    out += ";\n";
    auto preset_name = [this](const char *key) -> std::string {
        if (const auto *opt = m_full_config.option<ConfigOptionString>(key); opt != nullptr)
            return opt->value;
        if (const auto *opt = m_full_config.option<ConfigOptionStrings>(key); opt != nullptr && ! opt->values.empty())
            return opt->values.front();
        return {};
    };
    out += "; Print preset: "    + preset_name("print_settings_id") + "\n";
    out += "; Filament preset: " + preset_name("filament_settings_id") + "\n";
    out += "; Printer preset: "  + preset_name("printer_settings_id") + "\n";
    out += ";\n";
    out += format(";  - Pressure advance start: %1%, end: %2%, step: %3%\n", float_to_string_decimal_point(p.pa_start), float_to_string_decimal_point(p.pa_end), float_to_string_decimal_point(p.pa_step));
    out += format(";  - Values: %1%\n", values);
    out += format(";  - Layers: %1%, first layer height: %2% mm, layer height: %3% mm\n", p.num_layers, float_to_string_decimal_point(m_first_layer_height), float_to_string_decimal_point(m_layer_height));
    out += format(";  - Walls: %1%, side length: %2% mm, spacing: %3% mm, corner angle: %4% deg, print direction: %5% deg\n",
                  p.wall_count, float_to_string_decimal_point(p.wall_side_length), float_to_string_decimal_point(p.pattern_spacing),
                  float_to_string_decimal_point(p.corner_angle), float_to_string_decimal_point(p.print_dir));
    out += format(";  - Line width: %1% %% (%2% mm), anchor: %3%, anchor perimeters: %4%, anchor line width: %5% %% (%6% mm)\n",
                  float_to_string_decimal_point(p.line_ratio), float_to_string_decimal_point(round_to(m_line_width, 4)), anchor, p.anchor_perimeters,
                  float_to_string_decimal_point(p.anchor_line_ratio), float_to_string_decimal_point(round_to(m_line_width_anchor, 4)));
    out += format(";  - First layer speed: %1% mm/s, print speed: %2% mm/s, acceleration: %3%\n",
                  float_to_string_decimal_point(p.first_layer_speed), float_to_string_decimal_point(p.perimeter_speed),
                  p.acceleration > 0. ? float_to_string_decimal_point(p.acceleration) + " mm/s^2" : std::string("firmware default"));
    out += format(";  - Number tab: %1%, no leading zero: %2%, show on display: %3%\n", on_off(p.number_tab), on_off(p.no_leading_zero), on_off(p.show_on_display));
    out += format(";  - Print size: %1% x %2% mm\n", float_to_string_decimal_point(round_to(size().x(), 2)), float_to_string_decimal_point(round_to(size().y(), 2)));
    out += "\n";
    return out;
}

void PatternGenerator::emit_pattern()
{
    const PressureAdvancePatternParams &p = m_params;
    using Anchor = PressureAdvancePatternParams::Anchor;

    const auto layer_height = [this](int layer) { return layer == 0 ? m_first_layer_height : m_layer_height; };
    const auto layer_speed  = [&p](int layer) { return layer == 0 ? p.first_layer_speed : p.perimeter_speed; };

    // Anchor.
    if (p.anchor != Anchor::None) {
        set_role(GCodeExtrusionRole::Skirt);
        draw_box(m_pattern_start_x, m_pattern_start_y, m_print_size_x, m_frame_size_y, p.anchor == Anchor::Layer, p.anchor_perimeters);
        if (p.number_tab)
            // Tab for the labels.
            draw_box(m_pattern_start_x, m_pattern_start_y + m_frame_size_y + m_line_spacing_anchor, m_tab_max_x - m_pattern_start_x, m_tab_size_y, true, p.anchor_perimeters);
    }

    for (int layer = (p.anchor == Anchor::Layer ? 1 : 0); layer < p.num_layers; ++ layer) {
        if (layer == 1) {
            // Switch from the first layer temperatures to the other layers ones.
            if (int temp = m_config.temperature.get_at(0); temp > 0 && temp != m_config.first_layer_temperature.get_at(0))
                m_gcode += m_writer.set_temperature(temp, false, 0);
            if (int temp = m_config.bed_temperature.get_at(0); temp > 0 && temp != m_config.first_layer_bed_temperature.get_at(0))
                m_gcode += m_writer.set_bed_temperature(temp, false);
        }
        if (layer > 0 && fan_speed(layer) != fan_speed(layer - 1))
            m_gcode += m_writer.set_fan(fan_speed(layer));
        if (layer > 0)
            layer_change(m_first_layer_height + layer * m_layer_height, layer_height(layer));

        // Labels below every other chevron, printed on the first layer above the anchor.
        if (p.number_tab && layer == (p.anchor == Anchor::None ? 0 : 1)) {
            set_role(GCodeExtrusionRole::Perimeter);
            m_gcode += set_pressure_advance(p.pattern_value(0), "set pressure advance to start value for numbering");
            for (int j = 0; j < m_num_patterns; j += 2) {
                const double x = m_pattern_start_x + m_pattern_shift +
                                 j * (p.pattern_spacing + m_line_width + (p.wall_count - 1) * m_line_spacing_angle) +
                                 // Center the label below the chevron walls.
                                 ((p.wall_count - 1) / 2.) * m_line_spacing_angle - GLYPH_SEGMENT_LENGTH;
                draw_number(x, m_pattern_start_y + m_frame_size_y + GLYPH_PADDING_VERTICAL + m_line_width, m_labels[j], layer_height(layer));
            }
        }

        // Chevrons.
        set_role(GCodeExtrusionRole::ExternalPerimeter);
        double x    = m_pattern_start_x + m_pattern_shift;
        double y    = m_pattern_start_y;
        double side = p.wall_side_length;
        if (layer == 0 && p.anchor == Anchor::Frame) {
            // Shrink the first layer of the chevrons to fit inside the frame.
            const double inset  = m_line_spacing_anchor * (p.anchor_perimeters - 1) + m_line_width_anchor * (1. - ENCROACHMENT);
            const double shrink = inset / std::sin(m_half_angle);
            side -= shrink;
            x    += shrink * std::cos(m_half_angle);
            y    += inset;
        }
        const double start_y = y;
        move_to({ x, y });
        for (int j = 0; j < m_num_patterns; ++ j) {
            m_gcode += set_pressure_advance(p.pattern_value(j), "set pressure advance");
            for (int k = 0; k < p.wall_count; ++ k) {
                x += std::cos(m_half_angle) * side;
                y += std::sin(m_half_angle) * side;
                draw_line({ x, y }, m_line_width, layer_height(layer), layer_speed(layer));
                x -= std::cos(m_half_angle) * side;
                y += std::sin(m_half_angle) * side;
                draw_line({ x, y }, m_line_width, layer_height(layer), layer_speed(layer));
                y = start_y;
                if (k != p.wall_count - 1)
                    // Next wall of this chevron.
                    x += m_line_spacing_angle;
                else if (j != m_num_patterns - 1)
                    // Next chevron.
                    x += p.pattern_spacing + m_line_width;
                else
                    // Done with this layer.
                    break;
                move_to({ x, y });
            }
        }
    }
}

void PatternGenerator::reset_state()
{
    m_writer = GCodeWriter();
    m_writer.apply_print_config(m_config);
    m_writer.set_extruders({ 0 });
    m_writer.multiple_extruders = m_config.nozzle_diameter.size() > 1;
    // Select the extruder without emitting a tool change.
    m_writer.set_extruder(0);
    m_gcode.clear();
    m_pos            = this->start_point();
    m_print_z        = m_first_layer_height;
    m_last_width     = -1.;
    m_last_speed     = -1.;
    m_role           = GCodeExtrusionRole::None;
    m_extrusion_bbox = BoundingBoxf();
}

Vec2d PatternGenerator::start_point() const
{
    return m_params.anchor != PressureAdvancePatternParams::Anchor::None ?
        Vec2d(m_pattern_start_x, m_pattern_start_y) : Vec2d(m_pattern_start_x + m_pattern_shift, m_pattern_start_y);
}

std::string PatternGenerator::generate()
{
    const PressureAdvancePatternParams &p = m_params;

    // Fan as for short layers of a sliced print, see fan_speed().
    const bool cooling = m_config.cooling.get_at(0);
    m_fan_speed = cooling ? m_config.max_fan_speed.get_at(0) : m_config.fan_always_on.get_at(0) ? m_config.min_fan_speed.get_at(0) : 0;

    // Dry run to find the extents of the pattern, to validate them against the bed and to pass them to the start G-code.
    this->reset_state();
    this->emit_pattern();
    m_pattern_size = m_extrusion_bbox.size();
    for (const Vec2d &pt : { m_extrusion_bbox.min, Vec2d(m_extrusion_bbox.max.x(), m_extrusion_bbox.min.y()),
                             m_extrusion_bbox.max, Vec2d(m_extrusion_bbox.min.x(), m_extrusion_bbox.max.y()) })
        if (! m_bed.contains(scaled(pt)))
            throw InvalidArgument(format(_u8L("The pattern (%1% x %2% mm) does not fit the print bed. "
                                              "Reduce the number of patterns, the side length or the spacing, or rotate the pattern."),
                                         float_to_string_decimal_point(round_to(size().x(), 1)), float_to_string_decimal_point(round_to(size().y(), 1))));
    this->init_placeholder_parser();

    this->reset_state();
    std::string out = this->header();

    // Start G-code, including the automatic temperature commands, the same way GCodeGenerator::_do_export() does.
    std::string start_gcode;
    {
        const int bed_temperature_extruder = m_config.bed_temperature_extruder;
        DynamicConfig config_override;
        if (0 < bed_temperature_extruder && bed_temperature_extruder <= int(m_config.nozzle_diameter.size()))
            config_override.set_key_value("first_layer_bed_temperature",
                new ConfigOptionInts(m_config.nozzle_diameter.size(), m_config.first_layer_bed_temperature.get_at(bed_temperature_extruder - 1)));
        start_gcode = this->process_template("start_gcode", m_config.start_gcode.value, config_override.empty() ? nullptr : &config_override);
    }
    const bool autoemit              = m_config.autoemit_temperature_commands;
    int        temp_by_gcode         = -1;
    const bool bed_temp_set_by_gcode = custom_gcode_sets_temperature(start_gcode, 140, 190, false, temp_by_gcode);
    const bool ext_temp_set_by_gcode = custom_gcode_sets_temperature(start_gcode, 104, 109, m_flavor == gcfRepRapFirmware, temp_by_gcode);
    const int  first_layer_bed_temp  = m_config.first_layer_bed_temperature.get_at(0);
    const int  first_layer_temp      = m_config.first_layer_temperature.get_at(0);
    if (autoemit && ! bed_temp_set_by_gcode && first_layer_bed_temp > 0)
        m_gcode += m_writer.set_bed_temperature(first_layer_bed_temp, true);
    if (autoemit && ! ext_temp_set_by_gcode && first_layer_temp > 0)
        m_gcode += m_writer.set_temperature(first_layer_temp, false, 0);
    this->set_role(GCodeExtrusionRole::Custom);
    m_gcode += start_gcode;
    if (autoemit && ! ext_temp_set_by_gcode && first_layer_temp > 0)
        m_gcode += m_writer.set_temperature(first_layer_temp, true, 0);

    m_gcode += m_writer.preamble();
    if (m_writer.multiple_extruders)
        m_gcode += m_writer.toolchange(0);
    DynamicConfig filament_gcode_config;
    filament_gcode_config.set_key_value("layer_num", new ConfigOptionInt(0));
    filament_gcode_config.set_key_value("layer_z", new ConfigOptionFloat(m_first_layer_height));
    filament_gcode_config.set_key_value("max_layer_z", new ConfigOptionFloat(0.));
    filament_gcode_config.set_key_value("filament_extruder_id", new ConfigOptionInt(0));
    m_gcode += this->process_template("start_filament_gcode", m_config.start_filament_gcode.get_at(0), &filament_gcode_config);
    if (p.acceleration > 0.)
        m_gcode += m_writer.set_print_acceleration(static_cast<unsigned int>(std::round(p.acceleration)));
    m_gcode += m_writer.set_fan(this->fan_speed(0));

    // Travel to the start of the pattern high above the bed, the start G-code may have left the nozzle anywhere.
    m_gcode += m_writer.retract();
    m_gcode += m_writer.travel_to_z(std::max(SAFE_TRAVEL_Z, m_writer.get_position().z()), "move to safe Z");
    m_gcode += m_writer.travel_to_xy(this->to_bed(this->start_point()), "move to pattern start");
    m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) + "\n";
    m_gcode += ";Z:" + float_to_string_decimal_point(m_first_layer_height) + "\n";
    m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + float_to_string_decimal_point(m_first_layer_height) + "\n";
    m_gcode += m_writer.travel_to_z(m_first_layer_height, "move to first layer height");
    m_gcode += m_writer.unretract();
    m_gcode += this->set_pressure_advance(p.pattern_value(0), "set pressure advance to start value");

    this->emit_pattern();

    // Restore the filament's pressure advance if known, otherwise leave the start value.
    double restore_pa = p.pattern_value(0);
    if (const auto *opt = m_full_config.option<ConfigOptionFloatsNullable>("filament_pressure_advance");
        m_flavor == gcfKlipper && opt != nullptr && ! opt->is_nil(0))
        restore_pa = opt->get_at(0);
    m_gcode += this->set_pressure_advance(restore_pa, "restore pressure advance");
    m_gcode += m_writer.retract();
    m_gcode += m_writer.set_fan(0);

    // End G-code, the same way GCodeGenerator::_do_export() does.
    this->set_role(GCodeExtrusionRole::Custom);
    const double max_layer_z = m_first_layer_height + (p.num_layers - 1) * m_layer_height;
    filament_gcode_config.set_key_value("layer_num", new ConfigOptionInt(p.num_layers - 1));
    filament_gcode_config.set_key_value("layer_z", new ConfigOptionFloat(max_layer_z));
    filament_gcode_config.set_key_value("max_layer_z", new ConfigOptionFloat(max_layer_z));
    m_gcode += this->process_template("end_filament_gcode", m_config.end_filament_gcode.get_at(0), &filament_gcode_config);
    m_gcode += this->process_template("end_gcode", m_config.end_gcode.value, &filament_gcode_config);
    m_gcode += m_writer.postamble();

    out += m_gcode;

    // Full config delimited the same way as GCodeGenerator does, so that GCodeProcessor accepts the file.
    // Keys excluded by GCodeGenerator::encode_full_config() and the print host credentials are not stored.
    static constexpr std::string_view banned_keys[] = {
        "compatible_printers", "compatible_prints", "print_host", "printhost_apikey", "printhost_cafile", "printhost_password", "printhost_user"
    };
    out += "\n; prusaslicer_config = begin\n";
    for (const std::string &key : m_full_config.keys())
        if (std::find(std::begin(banned_keys), std::end(banned_keys), key) == std::end(banned_keys))
            out += "; " + key + " = " + m_full_config.opt_serialize(key) + "\n";
    out += "; prusaslicer_config = end\n";
    return out;
}

} // namespace

int PressureAdvancePatternParams::num_patterns() const
{
    return pa_step > 0. ? int(std::round((pa_end - pa_start) / pa_step + 1.)) : 0;
}

double PressureAdvancePatternParams::pattern_value(int pattern_idx) const
{
    return round_to(pa_start + pattern_idx * pa_step, 4);
}

// Start, end and step of the pressure advance range for the extruder type.
static std::array<double, 3> pressure_advance_range(GCodeFlavor flavor, PressureAdvanceExtruderType extruder_type)
{
    const bool bowden = extruder_type == PressureAdvanceExtruderType::Bowden;
    if (flavor == gcfMarlinLegacy || flavor == gcfMarlinFirmware)
        // Linear advance K factor.
        return bowden ? std::array<double, 3>{ 0., 2., 0.1 } : std::array<double, 3>{ 0., 0.2, 0.01 };
    // Pressure advance in seconds.
    return bowden ? std::array<double, 3>{ 0., 1., 0.05 } : std::array<double, 3>{ 0., 0.08, 0.005 };
}

void set_pressure_advance_range(PressureAdvancePatternParams &params, GCodeFlavor flavor, PressureAdvanceExtruderType extruder_type)
{
    const std::array<double, 3> range = pressure_advance_range(flavor, extruder_type);
    params.pa_start = range[0];
    params.pa_end   = range[1];
    params.pa_step  = range[2];
}

bool is_pressure_advance_range(const PressureAdvancePatternParams &params, GCodeFlavor flavor, PressureAdvanceExtruderType extruder_type)
{
    const std::array<double, 3> range = pressure_advance_range(flavor, extruder_type);
    return is_approx(params.pa_start, range[0]) && is_approx(params.pa_end, range[1]) && is_approx(params.pa_step, range[2]);
}

bool pressure_advance_supported(GCodeFlavor flavor)
{
    return flavor == gcfKlipper || flavor == gcfMarlinLegacy || flavor == gcfMarlinFirmware || flavor == gcfRepRapFirmware;
}

std::string set_pressure_advance_gcode(GCodeFlavor flavor, double value)
{
    const std::string v = float_to_string_decimal_point(round_to(value, 4));
    switch (flavor) {
    case gcfKlipper:        return "SET_PRESSURE_ADVANCE ADVANCE=" + v + "\n";
    case gcfMarlinLegacy:
    case gcfMarlinFirmware: return "M900 K" + v + "\n";
    case gcfRepRapFirmware: return "M572 D0 S" + v + "\n";
    default:                return {};
    }
}

std::string generate_pressure_advance_pattern(const DynamicPrintConfig &config, const PressureAdvancePatternParams &params)
{
    PatternGenerator generator(config, params);
    return generator.generate();
}

Vec2d pressure_advance_pattern_size(const DynamicPrintConfig &config, const PressureAdvancePatternParams &params)
{
    PatternGenerator generator(config, params);
    generator.generate();
    return generator.size();
}

} // namespace Slic3r
