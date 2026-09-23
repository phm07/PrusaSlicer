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
#include <cmath>

#include "CalibrationPattern.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "Utils.hpp"
#include "format.hpp"

namespace Slic3r {

namespace {

using namespace calibration;

constexpr double GLYPH_PADDING_HORIZONTAL = 1.;
constexpr double GLYPH_PADDING_VERTICAL   = 1.;
// Overlap of the first layer of the chevrons with the anchor frame, in line widths.
constexpr double ENCROACHMENT             = 1. / 3.;

std::string pa_label(double value, bool no_leading_zero)
{
    std::string out = float_to_string_decimal_point(value);
    if (no_leading_zero && out.size() > 1 && out[0] == '0' && out[1] == '.')
        out.erase(0, 1);
    return out;
}

class PatternGenerator : public CalibrationPatternGenerator
{
public:
    PatternGenerator(const DynamicPrintConfig &config, const PressureAdvancePatternParams &params) :
        CalibrationPatternGenerator(config), m_params(params)
    {
        this->validate();

        m_rotation            = params.print_dir;
        m_line_width          = m_nozzle_diameter * params.line_ratio / 100.;
        m_line_width_anchor   = m_nozzle_diameter * params.anchor_line_ratio / 100.;
        m_line_spacing        = extrusion_spacing(m_line_width, m_layer_height);
        m_line_spacing_anchor = extrusion_spacing(m_line_width_anchor, m_first_layer_height);
        m_half_angle          = deg2rad(params.corner_angle) / 2.;
        m_line_spacing_angle  = m_line_spacing / std::sin(m_half_angle);
        m_num_patterns        = params.num_patterns();

        this->compute_layout();
    }

private:
    Vec2d       start_point() const override;
    void        emit_pattern() override;
    std::string header() const override;
    int         num_layers() const override { return m_params.num_layers; }
    std::string input_filename_base() const override { return "pressure_advance_pattern"; }
    std::string does_not_fit_message() const override;
    double      acceleration() const override { return m_params.acceleration; }
    std::string before_pattern_gcode() const override { return this->set_pressure_advance(m_params.pattern_value(0), "set pressure advance to start value"); }
    std::string after_pattern_gcode() const override;

    void validate() const;
    void compute_layout();

    std::string set_pressure_advance(double value, const std::string_view comment) const;

    void draw_box(double min_x, double min_y, double size_x, double size_y, bool fill, int num_perimeters);

    const PressureAdvancePatternParams m_params;

    double  m_line_width;
    double  m_line_width_anchor;
    double  m_line_spacing;
    double  m_line_spacing_anchor;
    double  m_half_angle;
    double  m_line_spacing_angle;
    int     m_num_patterns;

    // Layout of the pattern, in pattern coordinates.
    double  m_frame_size_y;
    double  m_print_size_x;
    double  m_pattern_shift;
    double  m_pattern_start_x;
    double  m_pattern_start_y;
    double  m_tab_max_x;
    double  m_tab_size_y;
    std::vector<std::string> m_labels;
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
            max_label_height = std::max(max_label_height, label_length(m_labels.back()));
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
        const double shift = GLYPH_PADDING_HORIZONTAL - (((p.wall_count - 1) / 2.) * m_line_spacing_angle - LABEL_SEGMENT_LENGTH);
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
                  ((p.wall_count - 1) / 2.) * m_line_spacing_angle - LABEL_SEGMENT_LENGTH +
                  2. * LABEL_SEGMENT_LENGTH + GLYPH_PADDING_HORIZONTAL + m_line_width_anchor / 2.;
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

std::string PatternGenerator::after_pattern_gcode() const
{
    // Restore the filament's pressure advance if known, otherwise leave the start value.
    double restore_pa = m_params.pattern_value(0);
    if (const auto *opt = m_full_config.option<ConfigOptionFloatsNullable>("filament_pressure_advance");
        m_flavor == gcfKlipper && opt != nullptr && ! opt->is_nil(0))
        restore_pa = opt->get_at(0);
    return this->set_pressure_advance(restore_pa, "restore pressure advance");
}

std::string PatternGenerator::does_not_fit_message() const
{
    return format(_u8L("The pattern (%1% x %2% mm) does not fit the print bed. "
                       "Reduce the number of patterns, the side length or the spacing, or rotate the pattern."),
                  float_to_string_decimal_point(round_to(size().x(), 1)), float_to_string_decimal_point(round_to(size().y(), 1)));
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
    out += this->preset_names_header();
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
        begin_layer(layer, m_first_layer_height + layer * m_layer_height, layer_height(layer));

        // Labels below every other chevron, printed on the first layer above the anchor.
        if (p.number_tab && layer == (p.anchor == Anchor::None ? 0 : 1)) {
            set_role(GCodeExtrusionRole::Perimeter);
            m_gcode += set_pressure_advance(p.pattern_value(0), "set pressure advance to start value for numbering");
            for (int j = 0; j < m_num_patterns; j += 2) {
                const double x = m_pattern_start_x + m_pattern_shift +
                                 j * (p.pattern_spacing + m_line_width + (p.wall_count - 1) * m_line_spacing_angle) +
                                 // Center the label below the chevron walls.
                                 ((p.wall_count - 1) / 2.) * m_line_spacing_angle - LABEL_SEGMENT_LENGTH;
                draw_number({ x, m_pattern_start_y + m_frame_size_y + GLYPH_PADDING_VERTICAL + m_line_width }, m_labels[j],
                            LabelDirection::Vertical, m_line_width, layer_height(layer), p.first_layer_speed);
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

Vec2d PatternGenerator::start_point() const
{
    return m_params.anchor != PressureAdvancePatternParams::Anchor::None ?
        Vec2d(m_pattern_start_x, m_pattern_start_y) : Vec2d(m_pattern_start_x + m_pattern_shift, m_pattern_start_y);
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
