///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "FirstLayerPattern.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "CalibrationPattern.hpp"
#include "ClipperUtils.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "Utils.hpp"
#include "format.hpp"

namespace Slic3r {

namespace {

using namespace calibration;

// Minimum gap between neighbor squares, in mm.
constexpr double MIN_SPACING = 1.;
// Maximum number of squares along an axis.
constexpr int    MAX_SQUARES = 10;

class PatternGenerator : public CalibrationPatternGenerator
{
public:
    PatternGenerator(const DynamicPrintConfig &config, const FirstLayerPatternParams &params) :
        CalibrationPatternGenerator(config), m_params(params)
    {
        this->validate();
        m_flow = this->square_flow(m_params.perimeters, true, false);
        this->compute_layout();
    }

private:
    Vec2d       start_point() const override;
    void        emit_pattern() override;
    std::string header() const override;
    int         num_layers() const override { return 1; }
    std::string input_filename_base() const override { return "first_layer_pattern"; }
    std::string does_not_fit_message() const override;

    void validate() const;
    void compute_layout();

    const FirstLayerPatternParams m_params;
    SquareFlow                    m_flow;
    // Bottom left corners of the squares in the print order.
    std::vector<Vec2d>            m_squares;
    // Spread of the squares relative to the full spread between the margins, less than one if the bed is not
    // rectangular.
    double                        m_scale { 1. };
};

void PatternGenerator::validate() const
{
    const FirstLayerPatternParams &p = m_params;
    if (p.columns < 1 || p.rows < 1 || p.columns > MAX_SQUARES || p.rows > MAX_SQUARES)
        throw InvalidArgument(format(_u8L("The number of squares along X and Y has to be between 1 and %1%."), MAX_SQUARES));
    if (p.square_size <= 0. || p.margin < 0.)
        throw InvalidArgument(_u8L("Invalid square size or margin."));
    if (p.perimeters < 1)
        throw InvalidArgument(_u8L("The squares need at least one perimeter."));
}

void PatternGenerator::compute_layout()
{
    const FirstLayerPatternParams &p = m_params;

    if (2. * m_flow.infill_offset >= p.square_size - m_flow.infill_width)
        throw InvalidArgument(_u8L("The squares are too small for the number of perimeters. Increase the square size or reduce the number of perimeters."));

    // Spread the squares evenly between the margins of the bed's bounding box, a single column / row is centered.
    const BoundingBoxf bed_bbox(m_config.bed_shape.values);
    const Vec2d        area = bed_bbox.size() - 2. * Vec2d(p.margin, p.margin);
    const Vec2i        counts(p.columns, p.rows);
    // Largest spread of the square centers from the bed center, and the smallest one keeping the squares apart.
    Vec2d  half_spread = Vec2d::Zero();
    double min_scale   = 0.;
    for (int axis = 0; axis < 2; ++ axis) {
        if (area[axis] < p.square_size)
            throw InvalidArgument(this->does_not_fit_message());
        if (counts[axis] > 1) {
            half_spread[axis] = (area[axis] - p.square_size) / 2.;
            min_scale = std::max(min_scale, (p.square_size + MIN_SPACING) * (counts[axis] - 1) / (2. * half_spread[axis]));
        }
    }
    if (min_scale > 1.)
        throw InvalidArgument(this->does_not_fit_message());

    // Row by row from the front left one, alternating the direction of the rows to keep the travels short.
    auto layout = [this, &p, &counts, &half_spread](double scale) {
        std::vector<Vec2d> out;
        for (int row = 0; row < p.rows; ++ row)
            for (int i = 0; i < p.columns; ++ i) {
                const Vec2i idx(row % 2 == 0 ? i : p.columns - 1 - i, row);
                Vec2d center = m_center;
                for (int axis = 0; axis < 2; ++ axis)
                    if (counts[axis] > 1)
                        center[axis] += scale * half_spread[axis] * (2. * idx[axis] / (counts[axis] - 1) - 1.);
                out.emplace_back(center - Vec2d(p.square_size, p.square_size) / 2.);
            }
        return out;
    };
    // Area of the bed the squares have to be in. The squares of a rectangular bed touch its edges with their margins,
    // shrink the bed slightly less than the margin.
    Polygon bed;
    for (const Vec2d &pt : m_config.bed_shape.values)
        bed.points.emplace_back(scaled(pt));
    const Polygons allowed = offset(bed, - scaled<float>(p.margin - 0.01));
    auto on_bed = [&p, &allowed](const std::vector<Vec2d> &squares) {
        for (const Vec2d &origin : squares)
            for (const Vec2d &corner : { Vec2d(0., 0.), Vec2d(p.square_size, 0.), Vec2d(p.square_size, p.square_size), Vec2d(0., p.square_size) }) {
                const Point pt = scaled<coord_t>(Vec2d(origin + corner));
                if (std::none_of(allowed.begin(), allowed.end(), [&pt](const Polygon &poly) { return poly.contains(pt); }))
                    return false;
            }
        return true;
    };

    // On a bed which is not rectangular, e.g. a round one, pull the squares towards the bed center until they fit.
    m_scale = 1.;
    if (! on_bed(layout(1.))) {
        if (! on_bed(layout(min_scale)))
            throw InvalidArgument(this->does_not_fit_message());
        double lo = min_scale;
        double hi = 1.;
        for (int i = 0; i < 30; ++ i) {
            const double mid = (lo + hi) / 2.;
            (on_bed(layout(mid)) ? lo : hi) = mid;
        }
        m_scale = lo;
    }
    m_squares = layout(m_scale);
}

Vec2d PatternGenerator::start_point() const
{
    const double d = m_flow.perimeter_offsets.back();
    return m_squares.front() + Vec2d(d, d);
}

void PatternGenerator::emit_pattern()
{
    // All the squares printed the same way, with the same direction of the infill lines, to be comparable.
    for (const Vec2d &origin : m_squares) {
        draw_square_perimeters(origin, m_params.square_size, m_flow, true);
        draw_square_infill(origin, m_params.square_size, m_flow, true, false, false);
    }
}

std::string PatternGenerator::does_not_fit_message() const
{
    return _u8L("The squares do not fit the print bed. Reduce the number of squares, their size or the margin.");
}

std::string PatternGenerator::header() const
{
    const FirstLayerPatternParams &p = m_params;
    std::string out;
    out += "; " + header_slic3r_generated() + "\n";
    out += ";\n";
    out += "; First layer calibration pattern\n";
    out += "; Single layer squares spread over the print bed, printed as the first layer of a sliced print.\n";
    out += "; All the squares should look the same: smooth, with the lines merged together, without gaps or ridges.\n";
    out += ";\n";
    out += this->preset_names_header();
    out += ";\n";
    out += format(";  - Squares: %1% (%2% x %3% grid), size: %4% mm, margin: %5% mm, perimeters: %6%\n",
                  p.columns * p.rows, p.columns, p.rows, float_to_string_decimal_point(p.square_size), float_to_string_decimal_point(p.margin), p.perimeters);
    if (m_scale < 1.)
        out += format(";  - Pulled towards the bed center to fit the bed, spread: %1%%%\n", int(std::floor(m_scale * 100.)));
    out += format(";  - First layer height: %1% mm\n", float_to_string_decimal_point(m_first_layer_height));
    out += format(";  - Extrusion width: external perimeter %1% mm, perimeter %2% mm, solid infill %3% mm\n",
                  float_to_string_decimal_point(round_to(m_flow.ext_perimeter_width, 4)), float_to_string_decimal_point(round_to(m_flow.perimeter_width, 4)),
                  float_to_string_decimal_point(round_to(m_flow.infill_width, 4)));
    out += format(";  - Print size: %1% x %2% mm\n", float_to_string_decimal_point(round_to(size().x(), 2)), float_to_string_decimal_point(round_to(size().y(), 2)));
    out += "\n";
    return out;
}

} // namespace

std::string generate_first_layer_pattern(const DynamicPrintConfig &config, const FirstLayerPatternParams &params)
{
    PatternGenerator generator(config, params);
    return generator.generate();
}

Vec2d first_layer_pattern_size(const DynamicPrintConfig &config, const FirstLayerPatternParams &params)
{
    PatternGenerator generator(config, params);
    generator.generate();
    return generator.size();
}

} // namespace Slic3r
