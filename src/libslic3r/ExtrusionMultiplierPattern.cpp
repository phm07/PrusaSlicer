///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ExtrusionMultiplierPattern.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "CalibrationPattern.hpp"
#include "Exception.hpp"
#include "Flow.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "Utils.hpp"
#include "format.hpp"

namespace Slic3r {

namespace {

using namespace calibration;

// Layers of the embossed labels above the top surface.
constexpr int    LABEL_LAYERS    = 2;
// Maximum speed of the labels, they consist of short segments.
constexpr double LABEL_MAX_SPEED = 50.;
// Distance of the label from the perimeters.
constexpr double LABEL_MARGIN    = 1.5;
// Speed used if the print preset sets the speed to zero (automatic) and there is no volumetric speed limit.
constexpr double AUTO_SPEED      = 60.;

// Extrusion roles of a square, see PatternGenerator::acceleration() and speed().
enum class Role { ExternalPerimeter, Perimeter, SolidInfill, TopSolidInfill };

class PatternGenerator : public CalibrationPatternGenerator
{
public:
    PatternGenerator(const DynamicPrintConfig &config, const ExtrusionMultiplierPatternParams &params) :
        CalibrationPatternGenerator(config), m_params(params)
    {
        this->validate();
        m_filament_em = m_config.extrusion_multiplier.get_at(0);
        this->compute_layout();
    }

private:
    Vec2d       start_point() const override;
    void        emit_pattern() override;
    std::string header() const override;
    int         num_layers() const override { return m_params.num_layers + (m_params.labels ? LABEL_LAYERS : 0); }
    std::string input_filename_base() const override { return "extrusion_multiplier_pattern"; }
    std::string does_not_fit_message() const override;

    void validate() const;
    void compute_layout();

    // Flow parameters of a layer of the squares, see layer_flow().
    struct LayerFlow {
        double height;
        double ext_perimeter_width;
        double perimeter_width;
        double infill_width;
        // Distance of the perimeter centerlines from the edge of the square, from the outermost one.
        std::vector<double> perimeter_offsets;
        // Distance of the infill boundary from the edge of the square, including the infill / perimeters overlap.
        double infill_offset;
    };
    LayerFlow layer_flow(int layer) const;
    // Extrusion width from the print preset, see PrintRegion::flow().
    double    extrusion_width(const char *opt_key, FlowRole role, bool first_layer, double height) const;
    // Print speed from the print preset, see GCodeGenerator::_extrude().
    double    speed(Role role, bool first_layer, double width, double height) const;
    void      set_acceleration(Role role, bool first_layer);

    double    layer_z(int layer) const { return m_first_layer_height + layer * m_layer_height; }
    // Bottom left corner of the given square, in pattern coordinates.
    Vec2d     square_origin(int square_idx) const;

    void draw_perimeters(const Vec2d &origin, int layer, const LayerFlow &flow, double flow_ratio);
    void draw_infill(const Vec2d &origin, int layer, const LayerFlow &flow, double flow_ratio);
    void draw_label(const Vec2d &origin, const std::string &label, const LayerFlow &flow);

    const ExtrusionMultiplierPatternParams m_params;
    double m_filament_em;
    int    m_num_squares;
    int    m_columns;
    int    m_rows;
    Vec2d  m_origin;
    std::vector<std::string> m_labels;
};

void PatternGenerator::validate() const
{
    const ExtrusionMultiplierPatternParams &p = m_params;
    if (p.em_start <= 0. || p.em_step <= 0. || p.em_end <= p.em_start)
        throw InvalidArgument(_u8L("The extrusion multiplier end value has to be larger than the start value, "
                                   "the start value and the step have to be positive."));
    if (p.num_squares() > 49)
        throw InvalidArgument(_u8L("Too many squares to print, increase the step or narrow the range."));
    if (p.num_layers < 2)
        throw InvalidArgument(_u8L("The squares need at least two layers."));
    if (p.perimeters < 1)
        throw InvalidArgument(_u8L("The squares need at least one perimeter."));
    if (p.square_size <= 0. || p.spacing < 1.)
        throw InvalidArgument(_u8L("Invalid square size or spacing. The squares have to be at least 1 mm apart."));
    if (m_config.extrusion_multiplier.get_at(0) <= 0.)
        throw InvalidArgument(_u8L("Invalid extrusion multiplier of the filament."));
}

void PatternGenerator::compute_layout()
{
    const ExtrusionMultiplierPatternParams &p = m_params;
    m_num_squares = p.num_squares();
    // As square as possible, filled row by row.
    m_columns = int(std::ceil(std::sqrt(double(m_num_squares)) - EPSILON));
    m_rows    = (m_num_squares + m_columns - 1) / m_columns;
    const Vec2d size(m_columns * p.square_size + (m_columns - 1) * p.spacing, m_rows * p.square_size + (m_rows - 1) * p.spacing);
    m_origin = m_center - size / 2.;

    m_labels.clear();
    for (int i = 0; i < m_num_squares; ++ i)
        m_labels.emplace_back(p.square_label(i));

    // Validate that the perimeters leave space for the infill and the labels, on all layers.
    for (int layer : { 0, 1 }) {
        const LayerFlow flow = this->layer_flow(layer);
        if (2. * flow.infill_offset >= p.square_size - flow.infill_width)
            throw InvalidArgument(_u8L("The squares are too small for the number of perimeters. Increase the square size or reduce the number of perimeters."));
    }
    if (p.labels) {
        const LayerFlow flow   = this->layer_flow(p.num_layers);
        const double    margin = flow.perimeter_offsets.back() + flow.perimeter_width / 2. + LABEL_MARGIN;
        double max_label_length = 0.;
        for (const std::string &label : m_labels)
            max_label_length = std::max(max_label_length, label_length(label));
        if (margin + max_label_length > p.square_size - margin || margin + LABEL_HEIGHT > p.square_size - margin)
            throw InvalidArgument(_u8L("The squares are too small for the labels. Increase the square size or disable the labels."));
    }
}

Vec2d PatternGenerator::square_origin(int square_idx) const
{
    // The first square at the back left, to be read like a text when looking at the bed from the front.
    const int column = square_idx % m_columns;
    const int row    = square_idx / m_columns;
    return m_origin + Vec2d(column, m_rows - 1 - row) * (m_params.square_size + m_params.spacing);
}

double PatternGenerator::extrusion_width(const char *opt_key, FlowRole role, bool first_layer, double height) const
{
    auto option = [this](const char *key) {
        const auto *opt = m_full_config.option<ConfigOptionFloatOrPercent>(key);
        if (opt == nullptr)
            throw InvalidArgument(format("Missing configuration option %1%", key));
        return opt;
    };
    const ConfigOptionFloatOrPercent *opt = option(first_layer && option("first_layer_extrusion_width")->value > 0. ? "first_layer_extrusion_width" : opt_key);
    if (opt->value == 0.)
        opt = option("extrusion_width");
    return Flow::new_from_config_width(role, *opt, float(m_nozzle_diameter), float(height)).width();
}

PatternGenerator::LayerFlow PatternGenerator::layer_flow(int layer) const
{
    const bool first_layer = layer == 0;
    const bool top_layer   = layer >= m_params.num_layers - 1;
    LayerFlow  out;
    out.height              = first_layer ? m_first_layer_height : m_layer_height;
    out.ext_perimeter_width = this->extrusion_width("external_perimeter_extrusion_width", frExternalPerimeter, first_layer, out.height);
    out.perimeter_width     = this->extrusion_width("perimeter_extrusion_width", frPerimeter, first_layer, out.height);
    out.infill_width        = top_layer ?
        this->extrusion_width("top_infill_extrusion_width", frTopSolidInfill, first_layer, out.height) :
        this->extrusion_width("solid_infill_extrusion_width", frSolidInfill, first_layer, out.height);

    // Perimeter spacing the same way as PerimeterGenerator does.
    const double ext_spacing       = extrusion_spacing(out.ext_perimeter_width, out.height);
    const double perimeter_spacing = extrusion_spacing(out.perimeter_width, out.height);
    const double infill_spacing    = extrusion_spacing(out.infill_width, out.height);
    double offset = out.ext_perimeter_width / 2.;
    for (int i = 0; i < m_params.perimeters; ++ i) {
        if (i == 1)
            offset += (ext_spacing + perimeter_spacing) / 2.;
        else if (i > 1)
            offset += perimeter_spacing;
        out.perimeter_offsets.emplace_back(offset);
    }
    const double inset   = (m_params.perimeters == 1 ? ext_spacing : perimeter_spacing) / 2.;
    const double overlap = m_full_config.get_abs_value("infill_overlap", inset + infill_spacing / 2.);
    out.infill_offset    = offset + inset - std::max(0., overlap);
    return out;
}

double PatternGenerator::speed(Role role, bool first_layer, double width, double height) const
{
    const char *opt_key = role == Role::ExternalPerimeter ? "external_perimeter_speed" :
                          role == Role::Perimeter         ? "perimeter_speed" :
                          role == Role::SolidInfill       ? "solid_infill_speed" : "top_solid_infill_speed";
    const double area = extrusion_area(width, height);
    // Volumetric speed limit.
    double max_volumetric_speed = 0.;
    for (double limit : { m_config.max_volumetric_speed.value, m_config.filament_max_volumetric_speed.get_at(0) })
        if (limit > 0.)
            max_volumetric_speed = max_volumetric_speed > 0. ? std::min(max_volumetric_speed, limit) : limit;

    double speed = m_full_config.get_abs_value(opt_key);
    if (speed <= 0.)
        // Automatic speed.
        speed = max_volumetric_speed > 0. ? max_volumetric_speed / area : AUTO_SPEED;
    if (first_layer) {
        const double first_layer_infill_speed = role == Role::SolidInfill ? m_full_config.get_abs_value("first_layer_infill_speed", speed) : 0.;
        speed = first_layer_infill_speed > 0. ? first_layer_infill_speed : m_full_config.get_abs_value("first_layer_speed", speed);
    }
    if (max_volumetric_speed > 0.)
        speed = std::min(speed, max_volumetric_speed / area);
    return speed;
}

void PatternGenerator::set_acceleration(Role role, bool first_layer)
{
    const PrintConfig &c = m_config;
    if (c.default_acceleration.value <= 0.)
        return;
    double acceleration = c.default_acceleration.value;
    if (first_layer && c.first_layer_acceleration.value > 0.)
        acceleration = c.first_layer_acceleration.value;
    else if (role == Role::TopSolidInfill && c.top_solid_infill_acceleration.value > 0.)
        acceleration = c.top_solid_infill_acceleration.value;
    else if ((role == Role::SolidInfill || role == Role::TopSolidInfill) && c.solid_infill_acceleration.value > 0.)
        acceleration = c.solid_infill_acceleration.value;
    else if ((role == Role::SolidInfill || role == Role::TopSolidInfill) && c.infill_acceleration.value > 0.)
        acceleration = c.infill_acceleration.value;
    else if (role == Role::ExternalPerimeter && c.external_perimeter_acceleration.value > 0.)
        acceleration = c.external_perimeter_acceleration.value;
    else if ((role == Role::ExternalPerimeter || role == Role::Perimeter) && c.perimeter_acceleration.value > 0.)
        acceleration = c.perimeter_acceleration.value;
    m_gcode += m_writer.set_print_acceleration(static_cast<unsigned int>(std::floor(acceleration + 0.5)));
}

// Rectangular loops from the innermost one out, the external perimeter last, as PrusaSlicer does by default.
void PatternGenerator::draw_perimeters(const Vec2d &origin, int layer, const LayerFlow &flow, double flow_ratio)
{
    const bool   first_layer = layer == 0;
    const double size        = m_params.square_size;
    for (int i = int(flow.perimeter_offsets.size()) - 1; i >= 0; -- i) {
        const bool   external = i == 0;
        const Role   role     = external ? Role::ExternalPerimeter : Role::Perimeter;
        const double width    = external ? flow.ext_perimeter_width : flow.perimeter_width;
        const double speed    = this->speed(role, first_layer, width, flow.height);
        const double d        = flow.perimeter_offsets[i];
        const Vec2d  min      = origin + Vec2d(d, d);
        const Vec2d  max      = origin + Vec2d(size - d, size - d);
        // Travel to the next loop outwards without retracting, it is short and does not leave the square.
        move_to(min, i == int(flow.perimeter_offsets.size()) - 1);
        set_role(external ? GCodeExtrusionRole::ExternalPerimeter : GCodeExtrusionRole::Perimeter);
        set_acceleration(role, first_layer);
        draw_line({ max.x(), min.y() }, width, flow.height, speed, flow_ratio);
        draw_line(max,                  width, flow.height, speed, flow_ratio);
        draw_line({ min.x(), max.y() }, width, flow.height, speed, flow_ratio);
        draw_line(min,                  width, flow.height, speed, flow_ratio);
    }
}

// Solid infill of 45 degrees lines, alternating the direction layer by layer, connected by travels.
void PatternGenerator::draw_infill(const Vec2d &origin, int layer, const LayerFlow &flow, double flow_ratio)
{
    const bool first_layer = layer == 0;
    const bool top_layer   = layer == m_params.num_layers - 1;
    const Role role        = top_layer ? Role::TopSolidInfill : Role::SolidInfill;

    // Infill boundary in coordinates relative to the square.
    const double lo = flow.infill_offset;
    const double hi = m_params.square_size - flow.infill_offset;
    // Distribute the lines evenly over the boundary, adjusting the extrusion width to the resulting spacing
    // the same way as FillBase::_adjust_solid_spacing() does.
    const double spacing  = extrusion_spacing(flow.infill_width, flow.height);
    const double diagonal = 2. * (hi - lo) / std::sqrt(2.);
    const int    num_lines = std::max(1, int(std::round(diagonal / spacing)));
    const double new_spacing = diagonal / num_lines;
    const double width    = flow.infill_width + (new_spacing - spacing);
    const double speed    = this->speed(role, first_layer, width, flow.height);
    // Line ends are inset by half the spacing from the boundary, the boundary already includes the overlap
    // with the perimeters.
    const double inset    = new_spacing / 2.;
    const double min      = lo + inset;
    const double max      = hi - inset;

    set_role(top_layer ? GCodeExtrusionRole::TopSolidInfill : GCodeExtrusionRole::SolidInfill);
    set_acceleration(role, first_layer);
    // Odd layers are mirrored along Y.
    auto to_pattern = [&origin, layer, size = m_params.square_size](const Vec2d &pt) -> Vec2d {
        return origin + (layer % 2 == 0 ? pt : Vec2d(pt.x(), size - pt.y()));
    };
    bool reverse = false;
    for (int i = 0; i < num_lines; ++ i) {
        // Line x + y = c.
        const double c = 2. * lo + (i + 0.5) * new_spacing * std::sqrt(2.);
        // Intersections with the bottom or the right edge and with the left or the top edge.
        Vec2d a { 0., std::max(min, c - max) };
        a.x() = c - a.y();
        Vec2d b { std::max(min, c - max), 0. };
        b.y() = c - b.x();
        if (a.x() - b.x() < EPSILON)
            // The line misses the inset boundary in its corner.
            continue;
        if (reverse)
            std::swap(a, b);
        // Short connecting travel inside the square, don't retract.
        move_to(to_pattern(a), false);
        draw_line(to_pattern(b), width, flow.height, speed, flow_ratio);
        reverse = ! reverse;
    }
}

void PatternGenerator::draw_label(const Vec2d &origin, const std::string &label, const LayerFlow &flow)
{
    const double margin = flow.perimeter_offsets.back() + flow.perimeter_width / 2. + LABEL_MARGIN;
    const double speed  = std::min(this->speed(Role::ExternalPerimeter, false, flow.perimeter_width, flow.height), LABEL_MAX_SPEED);
    set_role(GCodeExtrusionRole::Perimeter);
    set_acceleration(Role::ExternalPerimeter, false);
    draw_number(origin + Vec2d(margin, margin), label, LabelDirection::Horizontal, flow.perimeter_width, flow.height, speed);
}

void PatternGenerator::emit_pattern()
{
    const ExtrusionMultiplierPatternParams &p = m_params;
    for (int layer = 0; layer < this->num_layers(); ++ layer) {
        const LayerFlow flow = this->layer_flow(layer);
        begin_layer(layer, this->layer_z(layer), flow.height);
        for (int i = 0; i < m_num_squares; ++ i) {
            const Vec2d origin = this->square_origin(i);
            if (layer < p.num_layers) {
                // The squares are printed with their extrusion multiplier instead of the filament's one.
                const double flow_ratio = p.square_value(i) / m_filament_em;
                draw_perimeters(origin, layer, flow, flow_ratio);
                draw_infill(origin, layer, flow, flow_ratio);
            } else
                draw_label(origin, m_labels[i], flow);
        }
    }
}

Vec2d PatternGenerator::start_point() const
{
    const double d = this->layer_flow(0).perimeter_offsets.back();
    return this->square_origin(0) + Vec2d(d, d);
}

std::string PatternGenerator::does_not_fit_message() const
{
    return format(_u8L("The pattern (%1% x %2% mm) does not fit the print bed. Reduce the number of squares, their size or the spacing."),
                  float_to_string_decimal_point(round_to(size().x(), 1)), float_to_string_decimal_point(round_to(size().y(), 1)));
}

std::string PatternGenerator::header() const
{
    const ExtrusionMultiplierPatternParams &p = m_params;
    std::string values;
    for (int i = 0; i < m_num_squares; ++ i)
        values += (i == 0 ? "" : ", ") + m_labels[i];
    const LayerFlow flow = this->layer_flow(1);

    std::string out;
    out += "; " + header_slic3r_generated() + "\n";
    out += ";\n";
    out += "; Extrusion multiplier calibration pattern\n";
    out += "; Squares printed with increasing extrusion multipliers, row by row from the back left one.\n";
    out += "; Pick the square with the smoothest top surface and set its value as the filament's extrusion multiplier.\n";
    out += ";\n";
    out += this->preset_names_header();
    out += ";\n";
    out += format(";  - Extrusion multiplier start: %1%, end: %2%, step: %3%, filament extrusion multiplier: %4%\n",
                  float_to_string_decimal_point(p.em_start), float_to_string_decimal_point(p.em_end), float_to_string_decimal_point(p.em_step),
                  float_to_string_decimal_point(m_filament_em));
    out += format(";  - Values: %1%\n", values);
    out += format(";  - Squares: %1% (%2% x %3%), size: %4% mm, spacing: %5% mm, perimeters: %6%, labels: %7%\n",
                  m_num_squares, m_columns, m_rows, float_to_string_decimal_point(p.square_size), float_to_string_decimal_point(p.spacing),
                  p.perimeters, p.labels ? "true" : "false");
    out += format(";  - Layers: %1%, first layer height: %2% mm, layer height: %3% mm\n",
                  p.num_layers, float_to_string_decimal_point(m_first_layer_height), float_to_string_decimal_point(m_layer_height));
    out += format(";  - Extrusion width: external perimeter %1% mm, perimeter %2% mm, solid infill %3% mm\n",
                  float_to_string_decimal_point(round_to(flow.ext_perimeter_width, 4)), float_to_string_decimal_point(round_to(flow.perimeter_width, 4)),
                  float_to_string_decimal_point(round_to(flow.infill_width, 4)));
    out += format(";  - Print size: %1% x %2% mm\n", float_to_string_decimal_point(round_to(size().x(), 2)), float_to_string_decimal_point(round_to(size().y(), 2)));
    out += "\n";
    return out;
}

// Half range and step of a calibration pass.
std::array<double, 2> extrusion_multiplier_range(ExtrusionMultiplierPass pass)
{
    return pass == ExtrusionMultiplierPass::Coarse ? std::array<double, 2>{ 0.06, 0.02 } : std::array<double, 2>{ 0.02, 0.005 };
}

} // namespace

int ExtrusionMultiplierPatternParams::num_squares() const
{
    return em_step > 0. ? int(std::round((em_end - em_start) / em_step + 1.)) : 0;
}

double ExtrusionMultiplierPatternParams::square_value(int square_idx) const
{
    return round_to(em_start + square_idx * em_step, 4);
}

std::string ExtrusionMultiplierPatternParams::square_label(int square_idx) const
{
    // Enough decimals to tell the neighbor squares apart, and to show the start value exactly.
    int decimals = 2;
    for (double v : { em_step, em_start })
        while (decimals < 4 && std::abs(round_to(v, decimals) - v) > 1e-6)
            ++ decimals;
    return float_to_string_decimal_point(this->square_value(square_idx), decimals);
}

void set_extrusion_multiplier_range(ExtrusionMultiplierPatternParams &params, double center, ExtrusionMultiplierPass pass)
{
    const std::array<double, 2> range = extrusion_multiplier_range(pass);
    params.em_step  = range[1];
    // Keep the start positive, shift the range up if needed.
    params.em_start = round_to(std::max(center - range[0], range[1]), 4);
    params.em_end   = round_to(params.em_start + 2. * range[0], 4);
}

bool is_extrusion_multiplier_range(const ExtrusionMultiplierPatternParams &params, double center, ExtrusionMultiplierPass pass)
{
    ExtrusionMultiplierPatternParams expected;
    set_extrusion_multiplier_range(expected, center, pass);
    return is_approx(params.em_start, expected.em_start) && is_approx(params.em_end, expected.em_end) && is_approx(params.em_step, expected.em_step);
}

std::string generate_extrusion_multiplier_pattern(const DynamicPrintConfig &config, const ExtrusionMultiplierPatternParams &params)
{
    PatternGenerator generator(config, params);
    return generator.generate();
}

Vec2d extrusion_multiplier_pattern_size(const DynamicPrintConfig &config, const ExtrusionMultiplierPatternParams &params)
{
    PatternGenerator generator(config, params);
    generator.generate();
    return generator.size();
}

} // namespace Slic3r
