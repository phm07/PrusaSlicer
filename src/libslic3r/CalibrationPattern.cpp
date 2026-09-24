///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "CalibrationPattern.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <vector>

#include "Exception.hpp"
#include "GCode.hpp"
#include "GCode/GCodeProcessor.hpp"
#include "I18N.hpp"
#include "LocalesUtils.hpp"
#include "Utils.hpp"
#include "format.hpp"

namespace Slic3r {

namespace calibration {

double round_to(double value, int decimals)
{
    const double scale = std::pow(10., decimals);
    return std::round(value * scale) / scale;
}

double deg2rad(double deg) { return deg * PI / 180.; }

double extrusion_area(double width, double height) { return (width - height) * height + PI * 0.25 * height * height; }

double extrusion_spacing(double width, double height) { return width - height * (1. - 0.25 * PI); }

} // namespace calibration

namespace {

// Digits of the labels, see draw_number().
constexpr double GLYPH_DOT_SIZE       = 0.75;
constexpr double GLYPH_SPACING        = 3.;
constexpr double GLYPH_SPACING_NARROW = 1.;
// Z height to travel at from the end of the start G-code to the pattern.
constexpr double SAFE_TRAVEL_Z        = 5.;
// Speed used if the print preset sets the speed to zero (automatic) and there is no volumetric speed limit.
constexpr double AUTO_SPEED           = 60.;

double glyph_advance(char c) { return (c == '1' || c == '.') ? GLYPH_SPACING_NARROW : GLYPH_SPACING; }

// Filament overrides of retraction parameters applied over the full print config, the same way Print::apply() does.
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

} // namespace

CalibrationPatternGenerator::CalibrationPatternGenerator(const DynamicPrintConfig &config) :
    m_full_config(config_with_filament_overrides(config))
{
    m_config.apply(m_full_config, true);
    m_flavor = m_config.gcode_flavor.value;

    if (m_config.nozzle_diameter.empty() || m_config.nozzle_diameter.get_at(0) <= 0. || m_config.filament_diameter.get_at(0) <= 0.)
        throw InvalidArgument(_u8L("Invalid nozzle or filament diameter."));

    m_nozzle_diameter    = m_config.nozzle_diameter.get_at(0);
    m_layer_height       = m_full_config.opt_float("layer_height");
    m_first_layer_height = m_config.first_layer_height.get_abs_value(m_layer_height);

    const BoundingBoxf bed_bbox(m_config.bed_shape.values);
    m_center = bed_bbox.center();
    for (const Vec2d &pt : m_config.bed_shape.values)
        m_bed.points.emplace_back(scaled(pt));
}

Vec2d CalibrationPatternGenerator::to_bed(const Vec2d &pt) const
{
    if (m_rotation == 0.)
        return pt;
    // Clockwise rotation around the bed center.
    const double a = calibration::deg2rad(m_rotation);
    const double c = std::cos(a);
    const double s = std::sin(a);
    const Vec2d  d = pt - m_center;
    return { c * d.x() + s * d.y() + m_center.x(), c * d.y() - s * d.x() + m_center.y() };
}

double CalibrationPatternGenerator::extrusion_width(const char *opt_key, FlowRole role, bool first_layer, double height) const
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

CalibrationPatternGenerator::SquareFlow CalibrationPatternGenerator::square_flow(int perimeters, bool first_layer, bool top_layer) const
{
    SquareFlow out;
    out.height              = first_layer ? m_first_layer_height : m_layer_height;
    out.ext_perimeter_width = this->extrusion_width("external_perimeter_extrusion_width", frExternalPerimeter, first_layer, out.height);
    out.perimeter_width     = this->extrusion_width("perimeter_extrusion_width", frPerimeter, first_layer, out.height);
    out.infill_width        = top_layer ?
        this->extrusion_width("top_infill_extrusion_width", frTopSolidInfill, first_layer, out.height) :
        this->extrusion_width("solid_infill_extrusion_width", frSolidInfill, first_layer, out.height);

    // Perimeter spacing the same way as PerimeterGenerator does.
    const double ext_spacing       = calibration::extrusion_spacing(out.ext_perimeter_width, out.height);
    const double perimeter_spacing = calibration::extrusion_spacing(out.perimeter_width, out.height);
    const double infill_spacing    = calibration::extrusion_spacing(out.infill_width, out.height);
    double offset = out.ext_perimeter_width / 2.;
    for (int i = 0; i < perimeters; ++ i) {
        if (i == 1)
            offset += (ext_spacing + perimeter_spacing) / 2.;
        else if (i > 1)
            offset += perimeter_spacing;
        out.perimeter_offsets.emplace_back(offset);
    }
    const double inset   = (perimeters == 1 ? ext_spacing : perimeter_spacing) / 2.;
    const double overlap = m_full_config.get_abs_value("infill_overlap", inset + infill_spacing / 2.);
    out.infill_offset    = offset + inset - std::max(0., overlap);
    return out;
}

double CalibrationPatternGenerator::print_speed(SquareRole role, bool first_layer, double width, double height) const
{
    const char *opt_key = role == SquareRole::ExternalPerimeter ? "external_perimeter_speed" :
                          role == SquareRole::Perimeter         ? "perimeter_speed" :
                          role == SquareRole::SolidInfill       ? "solid_infill_speed" : "top_solid_infill_speed";
    const double area = calibration::extrusion_area(width, height);
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
        const double first_layer_infill_speed = role == SquareRole::SolidInfill ? m_full_config.get_abs_value("first_layer_infill_speed", speed) : 0.;
        speed = first_layer_infill_speed > 0. ? first_layer_infill_speed : m_full_config.get_abs_value("first_layer_speed", speed);
    }
    if (max_volumetric_speed > 0.)
        speed = std::min(speed, max_volumetric_speed / area);
    return speed;
}

void CalibrationPatternGenerator::set_acceleration(SquareRole role, bool first_layer)
{
    const PrintConfig &c = m_config;
    if (c.default_acceleration.value <= 0.)
        return;
    const bool infill    = role == SquareRole::SolidInfill || role == SquareRole::TopSolidInfill;
    const bool perimeter = role == SquareRole::ExternalPerimeter || role == SquareRole::Perimeter;
    double acceleration = c.default_acceleration.value;
    if (first_layer && c.first_layer_acceleration.value > 0.)
        acceleration = c.first_layer_acceleration.value;
    else if (role == SquareRole::TopSolidInfill && c.top_solid_infill_acceleration.value > 0.)
        acceleration = c.top_solid_infill_acceleration.value;
    else if (infill && c.solid_infill_acceleration.value > 0.)
        acceleration = c.solid_infill_acceleration.value;
    else if (infill && c.infill_acceleration.value > 0.)
        acceleration = c.infill_acceleration.value;
    else if (role == SquareRole::ExternalPerimeter && c.external_perimeter_acceleration.value > 0.)
        acceleration = c.external_perimeter_acceleration.value;
    else if (perimeter && c.perimeter_acceleration.value > 0.)
        acceleration = c.perimeter_acceleration.value;
    m_gcode += m_writer.set_print_acceleration(static_cast<unsigned int>(std::floor(acceleration + 0.5)));
}

void CalibrationPatternGenerator::draw_square_perimeters(const Vec2d &origin, double size, const SquareFlow &flow, bool first_layer, double flow_ratio)
{
    for (int i = int(flow.perimeter_offsets.size()) - 1; i >= 0; -- i) {
        const bool       external = i == 0;
        const SquareRole role     = external ? SquareRole::ExternalPerimeter : SquareRole::Perimeter;
        const double     width    = external ? flow.ext_perimeter_width : flow.perimeter_width;
        const double     speed    = this->print_speed(role, first_layer, width, flow.height);
        const double     d        = flow.perimeter_offsets[i];
        const Vec2d      min      = origin + Vec2d(d, d);
        const Vec2d      max      = origin + Vec2d(size - d, size - d);
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

void CalibrationPatternGenerator::draw_square_infill(const Vec2d &origin, double size, const SquareFlow &flow, bool first_layer, bool top_layer, bool mirror, double flow_ratio)
{
    const SquareRole role = top_layer ? SquareRole::TopSolidInfill : SquareRole::SolidInfill;

    // Infill boundary in coordinates relative to the square.
    const double lo = flow.infill_offset;
    const double hi = size - flow.infill_offset;
    // Distribute the lines evenly over the boundary, adjusting the extrusion width to the resulting spacing
    // the same way as FillBase::_adjust_solid_spacing() does.
    const double spacing     = calibration::extrusion_spacing(flow.infill_width, flow.height);
    const double diagonal    = 2. * (hi - lo) / std::sqrt(2.);
    const int    num_lines   = std::max(1, int(std::round(diagonal / spacing)));
    const double new_spacing = diagonal / num_lines;
    const double width       = flow.infill_width + (new_spacing - spacing);
    const double speed       = this->print_speed(role, first_layer, width, flow.height);
    // Line ends are inset by half the spacing from the boundary, the boundary already includes the overlap
    // with the perimeters.
    const double inset       = new_spacing / 2.;
    const double min         = lo + inset;
    const double max         = hi - inset;

    set_role(top_layer ? GCodeExtrusionRole::TopSolidInfill : GCodeExtrusionRole::SolidInfill);
    set_acceleration(role, first_layer);
    auto to_pattern = [&origin, mirror, size](const Vec2d &pt) -> Vec2d {
        return origin + (mirror ? Vec2d(pt.x(), size - pt.y()) : pt);
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

void CalibrationPatternGenerator::set_role(GCodeExtrusionRole role)
{
    if (role != m_role) {
        m_role = role;
        m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) + gcode_extrusion_role_to_string(role) + "\n";
    }
}

void CalibrationPatternGenerator::set_width(double width)
{
    if (std::abs(width - m_last_width) > EPSILON) {
        m_last_width = width;
        m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) + float_to_string_decimal_point(width) + "\n";
    }
}

void CalibrationPatternGenerator::begin_layer(int layer, double print_z, double height)
{
    if (layer == 0)
        return;
    if (layer == 1) {
        // Switch from the first layer temperatures to the other layers ones.
        if (int temp = m_config.temperature.get_at(0); temp > 0 && temp != m_config.first_layer_temperature.get_at(0))
            m_gcode += m_writer.set_temperature(temp, false, 0);
        if (int temp = m_config.bed_temperature.get_at(0); temp > 0 && temp != m_config.first_layer_bed_temperature.get_at(0))
            m_gcode += m_writer.set_bed_temperature(temp, false);
    }
    if (fan_speed(layer) != fan_speed(layer - 1))
        m_gcode += m_writer.set_fan(fan_speed(layer));
    m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) + "\n";
    m_gcode += ";Z:" + float_to_string_decimal_point(print_z) + "\n";
    m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + float_to_string_decimal_point(height) + "\n";
    m_print_z = print_z;
    m_gcode += m_writer.travel_to_z(print_z, "move to next layer");
    m_last_speed = -1.;
}

void CalibrationPatternGenerator::move_to(const Vec2d &pt, bool allow_retract)
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

void CalibrationPatternGenerator::draw_line(const Vec2d &pt, double width, double height, double speed, double flow_ratio)
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
    m_gcode += m_writer.extrude_to_xy(to, m_writer.extruder()->e_per_mm(calibration::extrusion_area(width, height)) * length * flow_ratio);
    m_pos = pt;
}

double CalibrationPatternGenerator::label_length(const std::string &label)
{
    double length = 0.;
    for (char c : label)
        length += glyph_advance(c);
    return length;
}

// Each digit is a sequence of segments starting at one of the corners of the digit's cell. The glyphs are defined
// for LabelDirection::Vertical: "up" advances in the reading direction (+Y), "right" goes down the digit (+X).
void CalibrationPatternGenerator::draw_number(const Vec2d &start, const std::string &label, LabelDirection direction, double width, double height, double speed)
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

    // Glyph coordinates: u down the digit, v in the reading direction.
    auto to_pattern = [&start, direction](double u, double v) -> Vec2d {
        return direction == LabelDirection::Vertical ? Vec2d(start.x() + u, start.y() + v) : Vec2d(start.x() + v, start.y() + LABEL_HEIGHT - u);
    };
    const double seg    = LABEL_SEGMENT_LENGTH;
    double       offset = 0.;
    for (char c : label) {
        auto it = glyphs.find(c);
        if (it == glyphs.end())
            continue;
        double u = 0.;
        double v = offset;
        for (const std::string &s : it->second) {
            const bool draw = s == "up" || s == "down" || s == "right" || s == "left" || s == "dot";
            if      (s == "bl")                   { u = 0.;      v = offset; }
            else if (s == "br")                   { u = seg * 2; v = offset; }
            else if (s == "ul")                   { u = 0.;      v = offset + seg; }
            else if (s == "up"    || s == "mup")    v += seg;
            else if (s == "down")                   v -= seg;
            else if (s == "right" || s == "mright") u += seg;
            else if (s == "left"  || s == "mleft")  u -= seg;
            else if (s == "dot")                    u -= GLYPH_DOT_SIZE;
            if (draw)
                draw_line(to_pattern(u, v), width, height, speed);
            else
                move_to(to_pattern(u, v));
        }
        offset += glyph_advance(c);
    }
}

void CalibrationPatternGenerator::init_placeholder_parser()
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
    pp.set("input_filename_base", this->input_filename_base());
    pp.set("num_objects", 1);
    pp.set("num_instances", 1);
    pp.set("scale", std::vector<std::string>{ "x:100% y:100% z:100%" });
    pp.set("initial_tool", 0);
    pp.set("initial_extruder", 0);
    pp.set("current_extruder", 0);
    pp.set("total_layer_count", this->num_layers());
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

std::string CalibrationPatternGenerator::process_template(const std::string &name, const std::string &templ, const DynamicConfig *config_override)
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

std::string CalibrationPatternGenerator::preset_names_header() const
{
    auto preset_name = [this](const char *key) -> std::string {
        if (const auto *opt = m_full_config.option<ConfigOptionString>(key); opt != nullptr)
            return opt->value;
        if (const auto *opt = m_full_config.option<ConfigOptionStrings>(key); opt != nullptr && ! opt->values.empty())
            return opt->values.front();
        return {};
    };
    std::string out;
    out += "; Print preset: "    + preset_name("print_settings_id") + "\n";
    out += "; Filament preset: " + preset_name("filament_settings_id") + "\n";
    out += "; Printer preset: "  + preset_name("printer_settings_id") + "\n";
    return out;
}

void CalibrationPatternGenerator::reset_state()
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

std::string CalibrationPatternGenerator::generate()
{
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
            throw InvalidArgument(this->does_not_fit_message());
    this->init_placeholder_parser();

    this->reset_state();
    std::string out = this->header();
    // Placeholders of the statistics, filled in by GCodeProcessor::post_process_file() the same way as for a sliced print.
    if (m_config.remaining_times)
        out += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::First_Line_M73_Placeholder) + "\n";

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
    if (const double acceleration = this->acceleration(); acceleration > 0.)
        m_gcode += m_writer.set_print_acceleration(static_cast<unsigned int>(std::round(acceleration)));
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
    m_gcode += this->before_pattern_gcode();

    this->emit_pattern();

    m_gcode += this->after_pattern_gcode();
    m_gcode += m_writer.retract();
    m_gcode += m_writer.set_fan(0);

    // End G-code, the same way GCodeGenerator::_do_export() does.
    this->set_role(GCodeExtrusionRole::Custom);
    const double max_layer_z = m_print_z;
    filament_gcode_config.set_key_value("layer_num", new ConfigOptionInt(this->num_layers() - 1));
    filament_gcode_config.set_key_value("layer_z", new ConfigOptionFloat(max_layer_z));
    filament_gcode_config.set_key_value("max_layer_z", new ConfigOptionFloat(max_layer_z));
    m_gcode += this->process_template("end_filament_gcode", m_config.end_filament_gcode.get_at(0), &filament_gcode_config);
    m_gcode += this->process_template("end_gcode", m_config.end_gcode.value, &filament_gcode_config);
    m_gcode += m_writer.postamble();

    out += m_gcode;
    if (m_config.remaining_times)
        out += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Last_Line_M73_Placeholder) + "\n";

    // Filament statistics, see GCodeGenerator::_do_export(). Only the first extruder prints.
    const bool has_weight = m_config.filament_density.get_at(0) > 0.;
    const bool has_cost   = has_weight && m_config.filament_cost.get_at(0) > 0.;
    out += "\n";
    out += PrintStatistics::FilamentUsedMmMask + " 0\n";
    out += PrintStatistics::FilamentUsedCm3Mask + " 0\n";
    if (has_weight)
        out += PrintStatistics::FilamentUsedGMask + " 0\n";
    if (has_cost)
        out += PrintStatistics::FilamentCostMask + " 0\n";
    out += "\n";
    out += PrintStatistics::TotalFilamentUsedGMask + " 0\n";
    out += PrintStatistics::TotalFilamentCostMask + " 0\n";
    out += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Estimated_Printing_Time_Placeholder) + "\n";

    // Full config delimited the same way as GCodeGenerator does, so that GCodeProcessor accepts the file.
    // Keys excluded by GCodeGenerator::encode_full_config() and the print host credentials are not stored.
    static constexpr std::string_view banned_keys[] = {
        "compatible_printers", "compatible_prints", "klipper_estimator_limits", "print_host", "printhost_apikey", "printhost_cafile", "printhost_password", "printhost_user"
    };
    out += "\n; prusaslicer_config = begin\n";
    for (const std::string &key : m_full_config.keys())
        if (std::find(std::begin(banned_keys), std::end(banned_keys), key) == std::end(banned_keys))
            out += "; " + key + " = " + m_full_config.opt_serialize(key) + "\n";
    out += "; prusaslicer_config = end\n";
    return out;
}

} // namespace Slic3r
