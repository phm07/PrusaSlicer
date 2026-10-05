///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "MaxFlowPattern.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "CalibrationPattern.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "Slicing.hpp"
#include "Utils.hpp"
#include "format.hpp"

namespace Slic3r {

namespace {

using namespace calibration;

// Radius of the semicircles connecting the straight walls, at the wall's centerline.
constexpr double RADIUS            = 10.;
// Segments of a quarter circle, 10 degrees each.
constexpr int    QUARTER_SEGMENTS  = 9;
// Loops of the first layer base on either side of the wall.
constexpr int    BASE_LOOPS        = 4;
// Maximum number of bands.
constexpr int    MAX_BANDS         = 100;

class PatternGenerator : public CalibrationPatternGenerator
{
public:
    PatternGenerator(const DynamicPrintConfig &config, const MaxFlowPatternParams &params) :
        CalibrationPatternGenerator(config), m_params(params)
    {
        this->validate();
        this->compute_layout();
    }

    const MaxFlowPatternLayout& layout() const { return m_layout; }
    // Throws if the printer's machine limits do not allow printing the last band at its flow.
    void check_machine_limits() const;

private:
    Vec2d       start_point() const override { return this->loop(this->base_radius(BASE_LOOPS)).front(); }
    void        emit_pattern() override;
    std::string header() const override;
    int         num_layers() const override { return 1 + m_params.num_bands() * m_layout.layers_per_band; }
    std::string input_filename_base() const override { return "max_flow_pattern"; }
    std::string does_not_fit_message() const override;

    void validate() const;
    void compute_layout();

    // Print speed of a band of the wall.
    double band_speed(int band_idx) const { return m_params.band_flow(band_idx) / extrusion_area(m_layout.extrusion_width, m_layout.layer_height); }
    // Radius of a loop of the base, loop_idx from -BASE_LOOPS (innermost) to BASE_LOOPS (outermost), zero at the wall.
    double base_radius(int loop_idx) const { return RADIUS + loop_idx * m_base_spacing; }
    // Closed stadium around the bed center with the given radius of the semicircles, counterclockwise from its
    // leftmost point, so that the layer changes are on the semicircle and not on the straight walls.
    std::vector<Vec2d> loop(double radius) const;
    void draw_loop(double radius, double width, double height, double speed);
    // Acceleration of the wall, see print_acceleration(), limited by the machine limits. Zero if unknown.
    double wall_acceleration() const;
    bool   machine_limits_used() const { return m_config.machine_limits_usage.value != MachineLimitsUsage::Ignore; }

    const MaxFlowPatternParams m_params;
    MaxFlowPatternLayout       m_layout;
    double                     m_base_width;
    double                     m_base_spacing;
};

void PatternGenerator::validate() const
{
    const MaxFlowPatternParams &p = m_params;
    if (p.flow_start <= 0. || p.flow_step <= 0. || p.flow_end <= p.flow_start)
        throw InvalidArgument(_u8L("The flow end value has to be larger than the start value, the start value and the step have to be positive."));
    if (p.num_bands() > MAX_BANDS)
        throw InvalidArgument(_u8L("Too many bands to print, increase the step or narrow the range."));
    if (p.band_height <= 0.)
        throw InvalidArgument(_u8L("Invalid band height."));
    if (p.layer_height < 0. || p.extrusion_width < 0.)
        throw InvalidArgument(_u8L("Invalid layer height or extrusion width."));
    if (p.length < 2. * RADIUS)
        throw InvalidArgument(format(_u8L("The straight walls have to be at least %1% mm long."), int(2. * RADIUS)));
}

void PatternGenerator::compute_layout()
{
    const MaxFlowPatternParams &p = m_params;
    MaxFlowPatternLayout       &l = m_layout;
    l.first_layer_height = m_first_layer_height;
    l.layer_height       = p.layer_height > 0. ? p.layer_height : m_layer_height;
    const double max_layer_height = Slicing::max_layer_height_from_nozzle(m_full_config, 1);
    if (l.layer_height > max_layer_height + EPSILON)
        throw InvalidArgument(format(_u8L("The layer height %1% mm exceeds the maximum layer height %2% mm of the printer."),
                                     float_to_string_decimal_point(l.layer_height), float_to_string_decimal_point(round_to(max_layer_height, 4))));
    l.extrusion_width    = p.extrusion_width > 0. ? p.extrusion_width :
        this->extrusion_width("external_perimeter_extrusion_width", frExternalPerimeter, false, l.layer_height);
    if (l.extrusion_width < l.layer_height || l.extrusion_width > 3. * m_nozzle_diameter)
        throw InvalidArgument(_u8L("The extrusion width has to be larger than the layer height and at most three times the nozzle diameter."));
    l.layers_per_band = std::max(1, int(std::round(p.band_height / l.layer_height)));
    l.band_height     = l.layers_per_band * l.layer_height;
    l.total_height    = l.first_layer_height + p.num_bands() * l.band_height;
    if (const double max_height = m_config.max_print_height.value; max_height > 0. && l.total_height > max_height)
        throw InvalidArgument(format(_u8L("The tower (%1% mm) is taller than the maximum print height of the printer. "
                                          "Reduce the number of bands or their height."), float_to_string_decimal_point(round_to(l.total_height, 1))));

    m_base_width   = this->extrusion_width("external_perimeter_extrusion_width", frExternalPerimeter, true, m_first_layer_height);
    m_base_spacing = extrusion_spacing(m_base_width, m_first_layer_height);
}

double PatternGenerator::wall_acceleration() const
{
    // The highest flows of a sliced print are reached on the solid infill.
    double acceleration = this->print_acceleration(SquareRole::SolidInfill, false);
    if (this->machine_limits_used())
        if (const double limit = m_config.machine_max_acceleration_extruding.get_at(0); limit > 0.)
            acceleration = acceleration > 0. ? std::min(acceleration, limit) : limit;
    return acceleration;
}

void PatternGenerator::check_machine_limits() const
{
    const int    last  = m_params.num_bands() - 1;
    const double flow  = m_params.band_flow(last);
    const double speed = this->band_speed(last);
    auto to_string = [](double v) { return float_to_string_decimal_point(round_to(v, 1)); };
    if (this->machine_limits_used()) {
        if (const double max_feedrate = m_config.machine_max_feedrate_x.get_at(0); max_feedrate > 0. && speed > max_feedrate + EPSILON)
            throw InvalidArgument(format(_u8L("The flow of the last band (%1% mm³/s) needs a print speed of %2% mm/s, above the maximum "
                                              "feedrate of the X axis (%3% mm/s). Reduce the flow, or increase the layer height or the extrusion width."),
                                         to_string(flow), to_string(speed), to_string(max_feedrate)));
        const double filament_area = 0.25 * PI * sqr(m_config.filament_diameter.get_at(0));
        const double e_feedrate    = flow * m_config.extrusion_multiplier.get_at(0) / filament_area;
        if (const double max_feedrate = m_config.machine_max_feedrate_e.get_at(0); max_feedrate > 0. && e_feedrate > max_feedrate + EPSILON)
            throw InvalidArgument(format(_u8L("The flow of the last band (%1% mm³/s) needs a feedrate of the extruder of %2% mm/s, above its "
                                              "maximum feedrate (%3% mm/s). Reduce the flow."), to_string(flow), to_string(e_feedrate), to_string(max_feedrate)));
    }
    // At least half of each straight wall is to be printed at the full speed: accelerating to it and decelerating
    // from it takes speed^2 / acceleration.
    if (const double acceleration = this->wall_acceleration(); acceleration > 0.) {
        const double min_length = 2. * sqr(speed) / acceleration;
        if (m_params.length < min_length - EPSILON)
            throw InvalidArgument(format(_u8L("With the acceleration of %1% mm/s², the print head does not reach the speed of the last band "
                                              "(%2% mm/s) on the straight walls for long enough. Increase their length to %3% mm, reduce the flow, "
                                              "or increase the layer height or the extrusion width."),
                                         int(std::round(acceleration)), to_string(speed), int(std::ceil(min_length))));
    }
}

std::vector<Vec2d> PatternGenerator::loop(double radius) const
{
    const double half = m_params.length / 2.;
    std::vector<Vec2d> out;
    auto arc = [this, &out, radius](double center_x, double angle_from, int quarters) {
        for (int i = 0; i <= quarters * QUARTER_SEGMENTS; ++ i) {
            const double a = angle_from + 0.5 * PI * i / QUARTER_SEGMENTS;
            out.emplace_back(m_center + Vec2d(center_x + radius * std::cos(a), radius * std::sin(a)));
        }
    };
    // Lower half of the left semicircle, bottom wall, right semicircle, top wall, upper half of the left semicircle.
    arc(- half, PI, 1);
    arc(half, 1.5 * PI, 2);
    arc(- half, 0.5 * PI, 1);
    return out;
}

void PatternGenerator::draw_loop(double radius, double width, double height, double speed)
{
    const std::vector<Vec2d> pts = this->loop(radius);
    move_to(pts.front(), false);
    for (size_t i = 1; i < pts.size(); ++ i)
        draw_line(pts[i], width, height, speed);
}

void PatternGenerator::emit_pattern()
{
    const MaxFlowPatternParams &p = m_params;
    const MaxFlowPatternLayout &l = m_layout;

    // First layer base centered below the wall, from the outermost loop in. The loops are next to each other,
    // don't retract between them.
    {
        const double speed = this->print_speed(SquareRole::ExternalPerimeter, true, m_base_width, l.first_layer_height);
        set_acceleration(SquareRole::ExternalPerimeter, true);
        for (int i = BASE_LOOPS; i >= - BASE_LOOPS; -- i) {
            set_role(i == 0 ? GCodeExtrusionRole::ExternalPerimeter : GCodeExtrusionRole::Skirt);
            this->draw_loop(this->base_radius(i), m_base_width, l.first_layer_height, speed);
        }
    }

    // The wall, band by band, ignoring the volumetric speed limits.
    set_role(GCodeExtrusionRole::ExternalPerimeter);
    set_acceleration(SquareRole::SolidInfill, false);
    int layer = 1;
    for (int band = 0; band < p.num_bands(); ++ band)
        for (int i = 0; i < l.layers_per_band; ++ i, ++ layer) {
            begin_layer(layer, l.first_layer_height + layer * l.layer_height, l.layer_height);
            if (i == 0) {
                m_gcode += format(";Max flow band %1%: %2% mm3/s\n", band + 1, float_to_string_decimal_point(p.band_flow(band)));
                if (p.show_on_display)
                    m_gcode += format("M117 Flow %1% mm3/s\n", float_to_string_decimal_point(p.band_flow(band)));
            }
            this->draw_loop(RADIUS, l.extrusion_width, l.layer_height, this->band_speed(band));
        }
}

std::string PatternGenerator::does_not_fit_message() const
{
    return format(_u8L("The pattern (%1% x %2% mm) does not fit the print bed. Reduce the length of the straight walls."),
                  float_to_string_decimal_point(round_to(size().x(), 1)), float_to_string_decimal_point(round_to(size().y(), 1)));
}

std::string PatternGenerator::header() const
{
    const MaxFlowPatternParams &p = m_params;
    const MaxFlowPatternLayout &l = m_layout;
    auto to_string = [](double v, int decimals) { return float_to_string_decimal_point(round_to(v, decimals)); };

    std::string out;
    out += "; " + header_slic3r_generated() + "\n";
    out += ";\n";
    out += "; Maximum volumetric flow calibration pattern\n";
    out += "; A single wall tower printed in bands of increasing volumetric flow, from the bottom up.\n";
    out += "; Find the height at which the straight walls start to show gaps, a rough or matte surface or layers not bonding.\n";
    out += "; The flow of the band below it is the maximum volumetric flow, set it as the filament's max volumetric speed.\n";
    out += ";\n";
    out += this->preset_names_header();
    out += ";\n";
    out += format(";  - Flow start: %1% mm3/s, end: %2% mm3/s, step: %3% mm3/s, bands: %4%\n",
                  float_to_string_decimal_point(p.flow_start), float_to_string_decimal_point(p.flow_end), float_to_string_decimal_point(p.flow_step), p.num_bands());
    out += format(";  - Band height: %1% mm (%2% layers), first layer height: %3% mm, layer height: %4% mm\n",
                  to_string(l.band_height, 4), l.layers_per_band, to_string(l.first_layer_height, 4), to_string(l.layer_height, 4));
    out += format(";  - Extrusion width: %1% mm, straight walls: %2% mm, show on display: %3%\n",
                  to_string(l.extrusion_width, 4), float_to_string_decimal_point(p.length), p.show_on_display ? "true" : "false");
    if (const double acceleration = this->wall_acceleration(); acceleration > 0.)
        out += format(";  - Acceleration: %1% mm/s^2\n", int(std::round(acceleration)));
    out += format(";  - Print size: %1% x %2% mm, height: %3% mm\n",
                  to_string(size().x(), 2), to_string(size().y(), 2), to_string(l.total_height, 2));
    out += ";\n";
    out += ";  Band  Height from - to [mm]  Flow [mm3/s]  Speed [mm/s]\n";
    for (int i = 0; i < p.num_bands(); ++ i) {
        const double from = l.first_layer_height + i * l.band_height;
        out += format(";  %1$4d  %2$8s - %3$-8s      %4$10s  %5$12s\n", i + 1, to_string(from, 2), to_string(from + l.band_height, 2),
                      float_to_string_decimal_point(p.band_flow(i)), to_string(this->band_speed(i), 1));
    }
    out += "\n";
    return out;
}

} // namespace

int MaxFlowPatternParams::num_bands() const
{
    return flow_step > 0. ? int(std::round((flow_end - flow_start) / flow_step + 1.)) : 0;
}

double MaxFlowPatternParams::band_flow(int band_idx) const
{
    return round_to(flow_start + band_idx * flow_step, 4);
}

std::string generate_max_flow_pattern(const DynamicPrintConfig &config, const MaxFlowPatternParams &params)
{
    PatternGenerator generator(config, params);
    generator.check_machine_limits();
    return generator.generate();
}

Vec2d max_flow_pattern_size(const DynamicPrintConfig &config, const MaxFlowPatternParams &params)
{
    PatternGenerator generator(config, params);
    generator.check_machine_limits();
    generator.generate();
    return generator.size();
}

MaxFlowPatternLayout max_flow_pattern_layout(const DynamicPrintConfig &config, const MaxFlowPatternParams &params)
{
    return PatternGenerator(config, params).layout();
}

double max_flow_at_height(const MaxFlowPatternParams &params, const MaxFlowPatternLayout &layout, double height)
{
    const int band = int(std::floor((height - layout.first_layer_height) / layout.band_height + EPSILON));
    return params.band_flow(std::clamp(band, 0, std::max(0, params.num_bands() - 1)));
}

} // namespace Slic3r
