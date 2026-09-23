///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_CalibrationPattern_hpp_
#define slic3r_CalibrationPattern_hpp_

#include <string>
#include <string_view>

#include "BoundingBox.hpp"
#include "ExtrusionRole.hpp"
#include "Flow.hpp"
#include "PlaceholderParser.hpp"
#include "Polygon.hpp"
#include "PrintConfig.hpp"
#include "GCode/GCodeWriter.hpp"

namespace Slic3r {

// Base of the calibration patterns, which are generated directly as G-code from the active presets instead of being
// sliced: start / end G-code processed by the placeholder parser, temperatures, retraction including filament
// overrides, fan and bed shape are handled the same way as for a sliced print. The pattern is printed with the first
// extruder. The full config is appended at the end of the file, as for a sliced print, so that the G-code viewer can
// load the file.
class CalibrationPatternGenerator
{
public:
    virtual ~CalibrationPatternGenerator() = default;

    // Generates a complete, ready-to-print G-code file.
    // Throws Slic3r::InvalidArgument (with a translated message) if the pattern does not fit the bed
    // or if the custom G-code cannot be processed.
    std::string generate();
    // Size of the pattern on the bed, in mm, valid after generate().
    Vec2d size() const { return m_pattern_size; }

protected:
    // config is the full print config, i.e. the active print, filament and printer presets merged.
    // Throws Slic3r::InvalidArgument if the nozzle or the filament diameter is invalid.
    CalibrationPatternGenerator(const DynamicPrintConfig &config);

    // Point of the first layer the print head travels to before emit_pattern(), in pattern coordinates.
    virtual Vec2d       start_point() const = 0;
    // All the layers of the pattern. Expects the print head at the first layer, at start_point().
    // Called twice: a dry run to find the extents of the pattern, then to emit the G-code.
    virtual void        emit_pattern() = 0;
    // Comment block at the start of the file.
    virtual std::string header() const = 0;
    virtual int         num_layers() const = 0;
    // Value of the input_filename_base placeholder.
    virtual std::string input_filename_base() const = 0;
    virtual std::string does_not_fit_message() const = 0;
    // Printing acceleration set after the start G-code, zero to keep the acceleration configured in the firmware.
    virtual double      acceleration() const { return 0.; }
    // G-code emitted right before and right after the pattern.
    virtual std::string before_pattern_gcode() const { return {}; }
    virtual std::string after_pattern_gcode() const { return {}; }

    // "; Print preset: ..." lines of the header.
    std::string preset_names_header() const;

    // Pattern coordinates (before rotation around the bed center by m_rotation) to bed coordinates.
    Vec2d to_bed(const Vec2d &pt) const;

    void set_role(GCodeExtrusionRole role);
    // Temperature and fan changes of a new layer above the first one, and the layer change itself.
    void begin_layer(int layer, double print_z, double height);
    // Travel, retracting and lifting Z if the travel is longer than retract_before_travel.
    void move_to(const Vec2d &pt, bool allow_retract = true);
    // Extrude from the current position. The extruded volume is multiplied by flow_ratio on top of the filament's
    // extrusion multiplier.
    void draw_line(const Vec2d &pt, double width, double height, double speed, double flow_ratio = 1.);

    enum class LabelDirection {
        // The digits are stacked along the Y axis, to be read with the pattern rotated by 90 degrees clockwise.
        // start is the top left corner of the label as read.
        Vertical,
        // The digits are read along the X axis. start is the bottom left corner of the label.
        Horizontal,
    };
    // Draw a seven-segment like label consisting of digits and dots, see label_length() and LABEL_HEIGHT.
    void draw_number(const Vec2d &start, const std::string &label, LabelDirection direction, double width, double height, double speed);
    // Length of a label in the reading direction, as advanced by draw_number().
    static double label_length(const std::string &label);
    // Height of the digits.
    static constexpr double LABEL_HEIGHT = 4.;
    // Length of a digit segment.
    static constexpr double LABEL_SEGMENT_LENGTH = 2.;

    // Extrusion roles of the solid squares, see print_speed() and set_acceleration().
    enum class SquareRole { ExternalPerimeter, Perimeter, SolidInfill, TopSolidInfill };
    // Flow parameters of a layer of a solid square, see square_flow().
    struct SquareFlow {
        double height;
        double ext_perimeter_width;
        double perimeter_width;
        double infill_width;
        // Distance of the perimeter centerlines from the edge of the square, from the outermost one.
        std::vector<double> perimeter_offsets;
        // Distance of the infill boundary from the edge of the square, including the infill / perimeters overlap.
        double infill_offset;
    };
    // Extrusion widths and perimeter spacing from the print preset, the same way as for a sliced print.
    SquareFlow square_flow(int perimeters, bool first_layer, bool top_layer) const;
    // Extrusion width from the print preset, see PrintRegion::flow().
    double     extrusion_width(const char *opt_key, FlowRole role, bool first_layer, double height) const;
    // Print speed from the print preset including the volumetric speed limits, see GCodeGenerator::_extrude().
    double     print_speed(SquareRole role, bool first_layer, double width, double height) const;
    // Acceleration from the print preset, nothing is emitted if default_acceleration is zero.
    void       set_acceleration(SquareRole role, bool first_layer);
    // Rectangular loops of a square with the given bottom left corner, from the innermost one out,
    // the external perimeter last, as PrusaSlicer does by default.
    void       draw_square_perimeters(const Vec2d &origin, double size, const SquareFlow &flow, bool first_layer, double flow_ratio = 1.);
    // Solid infill of a square of 45 degrees lines, connected by travels. mirror flips the lines along Y,
    // to alternate the direction layer by layer.
    void       draw_square_infill(const Vec2d &origin, double size, const SquareFlow &flow, bool first_layer, bool top_layer, bool mirror, double flow_ratio = 1.);

    // Fan speed of the given layer, respecting disable_fan_first_layers.
    int fan_speed(int layer) const { return layer < m_config.disable_fan_first_layers.get_at(0) ? 0 : m_fan_speed; }

    // Full config with the filament overrides of the retraction parameters applied.
    DynamicPrintConfig m_full_config;
    PrintConfig        m_config;
    GCodeFlavor        m_flavor;

    double  m_nozzle_diameter;
    double  m_layer_height;
    double  m_first_layer_height;
    Vec2d   m_center;
    // Rotation of the pattern around the bed center, clockwise in degrees.
    double  m_rotation { 0. };

    GCodeWriter        m_writer;
    std::string        m_gcode;
    // Current position in pattern coordinates.
    Vec2d              m_pos { Vec2d::Zero() };
    double             m_print_z { 0. };

private:
    void reset_state();
    void set_width(double width);
    void init_placeholder_parser();
    std::string process_template(const std::string &name, const std::string &templ, const DynamicConfig *config_override);

    Polygon m_bed;
    // Fan speed in percent, above the layers with the fan disabled.
    int     m_fan_speed { 0 };
    Vec2d   m_pattern_size { Vec2d::Zero() };

    PlaceholderParser              m_placeholder_parser;
    PlaceholderParser::ContextData m_placeholder_context;
    DynamicConfig                  m_placeholder_outputs;

    double             m_last_width { -1. };
    // Feed rate of the last extrusion, invalidated by travels, which set their own feed rate.
    double             m_last_speed { -1. };
    GCodeExtrusionRole m_role { GCodeExtrusionRole::None };
    BoundingBoxf       m_extrusion_bbox;
};

namespace calibration {

double round_to(double value, int decimals);
double deg2rad(double deg);
// Cross section of an extrusion with a rectangular shape and semicircular ends, see Flow::mm3_per_mm().
double extrusion_area(double width, double height);
// Distance of neighbor extrusions, see Flow::spacing().
double extrusion_spacing(double width, double height);

} // namespace calibration

} // namespace Slic3r

#endif /* slic3r_CalibrationPattern_hpp_ */
