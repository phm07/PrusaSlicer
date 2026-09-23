///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ExtrusionMultiplierPattern.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "CalibrationPattern.hpp"
#include "Exception.hpp"
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

    SquareFlow layer_flow(int layer) const { return this->square_flow(m_params.perimeters, layer == 0, layer >= m_params.num_layers - 1); }
    double     layer_z(int layer) const { return m_first_layer_height + layer * m_layer_height; }
    // Bottom left corner of the given square, in pattern coordinates.
    Vec2d      square_origin(int square_idx) const;

    void draw_label(const Vec2d &origin, const std::string &label, const SquareFlow &flow);

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
        const SquareFlow flow = this->layer_flow(layer);
        if (2. * flow.infill_offset >= p.square_size - flow.infill_width)
            throw InvalidArgument(_u8L("The squares are too small for the number of perimeters. Increase the square size or reduce the number of perimeters."));
    }
    if (p.labels) {
        const SquareFlow flow   = this->layer_flow(p.num_layers);
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

void PatternGenerator::draw_label(const Vec2d &origin, const std::string &label, const SquareFlow &flow)
{
    const double margin = flow.perimeter_offsets.back() + flow.perimeter_width / 2. + LABEL_MARGIN;
    const double speed  = std::min(this->print_speed(SquareRole::ExternalPerimeter, false, flow.perimeter_width, flow.height), LABEL_MAX_SPEED);
    set_role(GCodeExtrusionRole::Perimeter);
    set_acceleration(SquareRole::ExternalPerimeter, false);
    draw_number(origin + Vec2d(margin, margin), label, LabelDirection::Horizontal, flow.perimeter_width, flow.height, speed);
}

void PatternGenerator::emit_pattern()
{
    const ExtrusionMultiplierPatternParams &p = m_params;
    for (int layer = 0; layer < this->num_layers(); ++ layer) {
        const SquareFlow flow = this->layer_flow(layer);
        begin_layer(layer, this->layer_z(layer), flow.height);
        for (int i = 0; i < m_num_squares; ++ i) {
            const Vec2d origin = this->square_origin(i);
            if (layer < p.num_layers) {
                // The squares are printed with their extrusion multiplier instead of the filament's one.
                const double flow_ratio = p.square_value(i) / m_filament_em;
                draw_square_perimeters(origin, p.square_size, flow, layer == 0, flow_ratio);
                // Alternate the direction of the infill lines layer by layer.
                draw_square_infill(origin, p.square_size, flow, layer == 0, layer == p.num_layers - 1, layer % 2 == 1, flow_ratio);
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
    const SquareFlow flow = this->layer_flow(1);

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
