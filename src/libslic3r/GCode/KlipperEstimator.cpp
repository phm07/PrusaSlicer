///|/ Copyright (c) 2022 Lasse Dalegaard
///|/
///|/ Ported from klipper_estimator (https://github.com/Annex-Engineering/klipper_estimator),
///|/ released under the MIT license.
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "KlipperEstimator.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <sstream>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <fast_float.h>
#include <LocalesUtils.hpp>

namespace Slic3r {
namespace KlipperEstimator {

namespace pt = boost::property_tree;

// ---------------------------------------------------------------------------------------------------------------------
// PrinterLimits
// ---------------------------------------------------------------------------------------------------------------------

static double scv_to_jd(double scv, double acceleration)
{
    return scv * scv * (std::sqrt(2.) - 1.) / acceleration;
}

void PrinterLimits::recalculate()
{
    junction_deviation = scv_to_jd(square_corner_velocity, max_acceleration);
    if (minimum_cruise_ratio)
        accel_to_decel = max_acceleration * (1. - std::clamp(*minimum_cruise_ratio, 0., 1.));
    else if (max_accel_to_decel)
        accel_to_decel = std::min(*max_accel_to_decel, max_acceleration);
    else
        accel_to_decel = std::min(50., max_acceleration);
}

void PrinterLimits::set_max_acceleration(double v)
{
    max_acceleration = v;
    this->recalculate();
}

void PrinterLimits::set_max_accel_to_decel(double v)
{
    max_accel_to_decel   = v;
    minimum_cruise_ratio = std::nullopt;
    this->recalculate();
}

void PrinterLimits::set_minimum_cruise_ratio(double v)
{
    max_accel_to_decel   = std::nullopt;
    minimum_cruise_ratio = std::clamp(v, 0., 1.);
    this->recalculate();
}

void PrinterLimits::set_square_corner_velocity(double scv)
{
    square_corner_velocity = scv;
    this->recalculate();
}

static std::optional<double> parse_double(std::string_view s)
{
    if (!s.empty() && s.front() == '+')
        s.remove_prefix(1);
    double v = 0.;
    auto [ptr, ec] = fast_float::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc() || ptr != s.data() + s.size() || s.empty())
        return std::nullopt;
    return v;
}

static std::optional<double> get_double(const pt::ptree &tree, const char *key)
{
    if (auto child = tree.get_child_optional(pt::ptree::path_type(key, '/')); child && child->empty())
        return parse_double(child->data());
    return std::nullopt;
}

static void put_double(pt::ptree &tree, const char *key, double value)
{
    tree.put(pt::ptree::path_type(key, '/'), float_to_string_decimal_point(value));
}

std::string PrinterLimits::serialize() const
{
    pt::ptree tree;
    put_double(tree, "max_velocity", max_velocity);
    put_double(tree, "max_acceleration", max_acceleration);
    if (max_accel_to_decel)
        put_double(tree, "max_accel_to_decel", *max_accel_to_decel);
    if (minimum_cruise_ratio)
        put_double(tree, "minimum_cruise_ratio", *minimum_cruise_ratio);
    put_double(tree, "square_corner_velocity", square_corner_velocity);
    put_double(tree, "instant_corner_velocity", instant_corner_velocity);
    if (mm_per_arc_segment)
        put_double(tree, "mm_per_arc_segment", *mm_per_arc_segment);
    if (firmware_retraction) {
        pt::ptree fr;
        put_double(fr, "retract_length", firmware_retraction->retract_length);
        put_double(fr, "unretract_extra_length", firmware_retraction->unretract_extra_length);
        put_double(fr, "unretract_speed", firmware_retraction->unretract_speed);
        put_double(fr, "retract_speed", firmware_retraction->retract_speed);
        put_double(fr, "lift_z", firmware_retraction->lift_z);
        tree.add_child("firmware_retraction", fr);
    }
    pt::ptree checkers;
    for (const MoveChecker &mc : move_checkers) {
        pt::ptree c;
        c.put("type", mc.type == MoveChecker::Type::Axis ? "axis" : "extruder");
        if (mc.type == MoveChecker::Type::Axis) {
            put_double(c, "axis_x", mc.axis.x());
            put_double(c, "axis_y", mc.axis.y());
            put_double(c, "axis_z", mc.axis.z());
        }
        put_double(c, "max_velocity", mc.max_velocity);
        put_double(c, "max_accel", mc.max_accel);
        checkers.push_back(std::make_pair("", c));
    }
    tree.add_child("move_checkers", checkers);

    std::ostringstream ss;
    pt::write_json(ss, tree, false);
    std::string out = ss.str();
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
        out.pop_back();
    return out;
}

std::optional<PrinterLimits> PrinterLimits::deserialize(const std::string &str)
{
    if (str.empty())
        return std::nullopt;
    try {
        std::istringstream ss(str);
        pt::ptree tree;
        pt::read_json(ss, tree);

        PrinterLimits limits;
        std::optional<double> max_velocity           = get_double(tree, "max_velocity");
        std::optional<double> max_acceleration       = get_double(tree, "max_acceleration");
        std::optional<double> square_corner_velocity = get_double(tree, "square_corner_velocity");
        std::optional<double> instant_corner_velocity = get_double(tree, "instant_corner_velocity");
        if (!max_velocity || !max_acceleration || !square_corner_velocity || !instant_corner_velocity)
            return std::nullopt;
        limits.max_velocity            = *max_velocity;
        limits.max_acceleration        = *max_acceleration;
        limits.square_corner_velocity  = *square_corner_velocity;
        limits.instant_corner_velocity = *instant_corner_velocity;
        limits.max_accel_to_decel      = get_double(tree, "max_accel_to_decel");
        limits.minimum_cruise_ratio    = get_double(tree, "minimum_cruise_ratio");
        limits.mm_per_arc_segment      = get_double(tree, "mm_per_arc_segment");
        if (auto fr = tree.get_child_optional("firmware_retraction"); fr) {
            FirmwareRetractionOptions opts;
            opts.retract_length         = get_double(*fr, "retract_length").value_or(0.);
            opts.unretract_extra_length = get_double(*fr, "unretract_extra_length").value_or(0.);
            opts.unretract_speed        = get_double(*fr, "unretract_speed").value_or(0.);
            opts.retract_speed          = get_double(*fr, "retract_speed").value_or(0.);
            opts.lift_z                 = get_double(*fr, "lift_z").value_or(0.);
            limits.firmware_retraction  = opts;
        }
        if (auto checkers = tree.get_child_optional("move_checkers"); checkers) {
            for (const auto &[key, c] : *checkers) {
                MoveChecker mc;
                mc.type = c.get<std::string>("type", "") == "axis" ? MoveChecker::Type::Axis : MoveChecker::Type::Extruder;
                if (mc.type == MoveChecker::Type::Axis)
                    mc.axis = Vec3d(get_double(c, "axis_x").value_or(0.), get_double(c, "axis_y").value_or(0.), get_double(c, "axis_z").value_or(0.));
                std::optional<double> max_velocity = get_double(c, "max_velocity");
                std::optional<double> max_accel    = get_double(c, "max_accel");
                if (!max_velocity || !max_accel)
                    return std::nullopt;
                mc.max_velocity = *max_velocity;
                mc.max_accel    = *max_accel;
                limits.move_checkers.emplace_back(mc);
            }
        }
        if (limits.max_velocity <= 0. || limits.max_acceleration <= 0.)
            return std::nullopt;
        limits.recalculate();
        return limits;
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

std::optional<PrinterLimits> PrinterLimits::from_moonraker_settings(const std::string &json, std::string *error)
{
    auto fail = [error](const std::string &msg) -> std::optional<PrinterLimits> {
        if (error != nullptr)
            *error = msg;
        return std::nullopt;
    };

    pt::ptree root;
    try {
        std::istringstream ss(json);
        pt::read_json(ss, root);
    } catch (const std::exception &ex) {
        return fail(std::string("Invalid JSON: ") + ex.what());
    }

    auto settings = root.get_child_optional(pt::ptree::path_type("result/status/configfile/settings", '/'));
    if (!settings)
        return fail("Missing configfile settings in the response");
    auto printer  = settings->get_child_optional("printer");
    auto extruder = settings->get_child_optional("extruder");
    if (!printer)
        return fail("Missing [printer] section");
    if (!extruder)
        return fail("Missing [extruder] section");

    auto require = [&fail](const pt::ptree &tree, const char *section, const char *key, double &out) {
        if (std::optional<double> v = get_double(tree, key); v) {
            out = *v;
            return true;
        }
        fail(std::string("Missing ") + section + "." + key);
        return false;
    };

    double max_velocity, max_accel, square_corner_velocity, max_extrude_only_velocity, max_extrude_only_accel, instantaneous_corner_velocity;
    if (!require(*printer, "printer", "max_velocity", max_velocity) ||
        !require(*printer, "printer", "max_accel", max_accel) ||
        !require(*printer, "printer", "square_corner_velocity", square_corner_velocity) ||
        !require(*extruder, "extruder", "max_extrude_only_velocity", max_extrude_only_velocity) ||
        !require(*extruder, "extruder", "max_extrude_only_accel", max_extrude_only_accel) ||
        !require(*extruder, "extruder", "instantaneous_corner_velocity", instantaneous_corner_velocity))
        return std::nullopt;
    if (max_velocity <= 0. || max_accel <= 0.)
        return fail("Invalid printer velocity or acceleration limits");

    PrinterLimits limits;
    limits.set_max_velocity(max_velocity);
    limits.set_max_acceleration(max_accel);
    if (std::optional<double> v = get_double(*printer, "minimum_cruise_ratio"); v)
        limits.set_minimum_cruise_ratio(*v);
    else if (std::optional<double> v = get_double(*printer, "max_accel_to_decel"); v)
        limits.set_max_accel_to_decel(*v);
    limits.set_square_corner_velocity(square_corner_velocity);
    limits.instant_corner_velocity = instantaneous_corner_velocity;

    if (auto arcs = settings->get_child_optional("gcode_arcs"); arcs)
        limits.mm_per_arc_segment = get_double(*arcs, "resolution");

    if (auto fr = settings->get_child_optional("firmware_retraction"); fr) {
        FirmwareRetractionOptions opts;
        if (!require(*fr, "firmware_retraction", "retract_length", opts.retract_length) ||
            !require(*fr, "firmware_retraction", "unretract_extra_length", opts.unretract_extra_length) ||
            !require(*fr, "firmware_retraction", "unretract_speed", opts.unretract_speed) ||
            !require(*fr, "firmware_retraction", "retract_speed", opts.retract_speed))
            return std::nullopt;
        opts.lift_z = get_double(*fr, "lift_z").value_or(0.);
        limits.firmware_retraction = opts;
    }

    const std::array<std::pair<const char*, Vec3d>, 3> axes{ {
        { "x", Vec3d::UnitX() }, { "y", Vec3d::UnitY() }, { "z", Vec3d::UnitZ() }
    } };
    for (const auto &[name, axis] : axes) {
        std::optional<double> v = get_double(*printer, (std::string("max_") + name + "_velocity").c_str());
        std::optional<double> a = get_double(*printer, (std::string("max_") + name + "_accel").c_str());
        if (v && a)
            limits.move_checkers.push_back({ MoveChecker::Type::Axis, axis, *v, *a });
    }
    limits.move_checkers.push_back({ MoveChecker::Type::Extruder, Vec3d::Zero(), max_extrude_only_velocity, max_extrude_only_accel });

    return limits;
}

// ---------------------------------------------------------------------------------------------------------------------
// G-code parsing
// ---------------------------------------------------------------------------------------------------------------------

namespace {

struct Command
{
    enum class Op { Nop, Move, Traditional, Extended };

    Op                                             op{ Op::Nop };
    // Op::Move
    std::array<std::optional<double>, 4>           axes;
    std::optional<double>                          feedrate;
    // Op::Traditional
    char                                           letter{ 0 };
    unsigned int                                   code{ 0 };
    std::vector<std::pair<char, std::string_view>> params;
    // Op::Extended, the command name and the parameter names are lower case.
    std::string                                    command;
    std::vector<std::pair<std::string, std::string>> ext_params;
    // Comment without the leading ';', only valid while the parsed line is alive.
    std::optional<std::string_view>                comment;

    bool is(char l, unsigned int c) const { return op == Op::Traditional && letter == l && code == c; }

    std::optional<double> param(char key) const {
        for (const auto &[c, v] : params)
            if (c == key)
                return parse_double(v);
        return std::nullopt;
    }

    std::optional<double> ext_param(const char *key) const {
        for (const auto &[k, v] : ext_params)
            if (k == key)
                return parse_double(v);
        return std::nullopt;
    }
};

inline bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f'; }
inline bool is_alpha(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }
inline bool is_digit(char c) { return c >= '0' && c <= '9'; }

std::string_view trim(std::string_view s)
{
    while (!s.empty() && is_space(s.front()))
        s.remove_prefix(1);
    while (!s.empty() && is_space(s.back()))
        s.remove_suffix(1);
    return s;
}

std::string_view skip_spaces(std::string_view s)
{
    while (!s.empty() && is_space(s.front()))
        s.remove_prefix(1);
    return s;
}

std::string to_lower(std::string_view s)
{
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return out;
}

// Parses an optional trailing comment, anything else is ignored.
void parse_comment(std::string_view s, Command &cmd)
{
    s = skip_spaces(s);
    if (s.empty() || s.front() != ';')
        return;
    s.remove_prefix(1);
    while (!s.empty() && is_space(s.back()))
        s.remove_suffix(1);
    cmd.comment = s;
}

bool parse_traditional(std::string_view s, Command &cmd)
{
    if (s.empty() || !is_alpha(s.front()))
        return false;
    const char letter = s.front();
    s.remove_prefix(1);
    size_t n = 0;
    unsigned long code = 0;
    while (n < s.size() && is_digit(s[n])) {
        code = code * 10 + (s[n] - '0');
        if (code > 65535)
            return false;
        ++n;
    }
    if (n == 0)
        return false;
    s.remove_prefix(n);

    std::vector<std::pair<char, std::string_view>> params;
    std::string_view rest = skip_spaces(s);
    bool first = true;
    for (;;) {
        std::string_view t = first ? rest : skip_spaces(rest);
        if (!first && t.size() == rest.size())
            break; // parameters have to be separated by white space
        if (t.empty() || !is_alpha(t.front()))
            break;
        size_t end = 1;
        while (end < t.size() && !is_space(t[end]) && t[end] != ';')
            ++end;
        params.emplace_back(char(std::toupper((unsigned char)t.front())), t.substr(1, end - 1));
        rest  = t.substr(end);
        first = false;
    }

    if ((letter == 'G') && (code == 0 || code == 1)) {
        cmd.op = Command::Op::Move;
        for (const auto &[c, v] : params) {
            std::optional<double> value = parse_double(v);
            if (!value)
                continue;
            switch (c) {
            case 'X': cmd.axes[0] = value; break;
            case 'Y': cmd.axes[1] = value; break;
            case 'Z': cmd.axes[2] = value; break;
            case 'E': cmd.axes[3] = value; break;
            case 'F': cmd.feedrate = value; break;
            default: break;
            }
        }
    } else {
        cmd.op     = Command::Op::Traditional;
        cmd.letter = letter;
        cmd.code   = (unsigned int)code;
        cmd.params = std::move(params);
    }
    parse_comment(rest, cmd);
    return true;
}

bool parse_extended(std::string_view s, Command &cmd)
{
    size_t n = 0;
    while (n < s.size() && (is_alpha(s[n]) || s[n] == '_'))
        ++n;
    if (n == 0)
        return false;
    cmd.op      = Command::Op::Extended;
    cmd.command = to_lower(s.substr(0, n));
    s.remove_prefix(n);

    // Parameters in the KEY=VALUE form, values may be quoted as in Klipper (shlex, non-posix).
    for (;;) {
        s = skip_spaces(s);
        if (s.empty() || s.front() == ';')
            break;
        size_t eq = 0;
        while (eq < s.size() && s[eq] != '=' && !is_space(s[eq]) && s[eq] != ';')
            ++eq;
        if (eq == s.size() || s[eq] != '=') {
            // Not a parameter, skip the token.
            s.remove_prefix(eq);
            continue;
        }
        std::string key = to_lower(s.substr(0, eq));
        s.remove_prefix(eq + 1);
        std::string_view value;
        if (!s.empty() && s.front() == '"') {
            size_t close = s.find('"', 1);
            value = close == std::string_view::npos ? s.substr(1) : s.substr(1, close - 1);
            s.remove_prefix(close == std::string_view::npos ? s.size() : close + 1);
        } else {
            size_t end = 0;
            while (end < s.size() && !is_space(s[end]) && s[end] != ';')
                ++end;
            value = s.substr(0, end);
            s.remove_prefix(end);
        }
        cmd.ext_params.emplace_back(std::move(key), std::string(value));
    }
    parse_comment(s, cmd);
    return true;
}

Command parse_line(std::string_view line)
{
    Command cmd;
    std::string_view s = trim(line);
    // Optional line number.
    if (s.size() > 1 && (s.front() == 'N' || s.front() == 'n') && is_digit(s[1])) {
        s.remove_prefix(1);
        while (!s.empty() && is_digit(s.front()))
            s.remove_prefix(1);
        s = skip_spaces(s);
    }
    if (s.empty())
        return cmd;
    if (s.front() == ';') {
        parse_comment(s, cmd);
        return cmd;
    }
    if (!parse_traditional(s, cmd) && !parse_extended(s, cmd)) {
        // Unparseable line, ignore it.
        cmd = Command();
    }
    return cmd;
}

// ---------------------------------------------------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------------------------------------------------

using V4 = std::array<double, 4>;

inline bool xyz_equal(const V4 &a, const V4 &b) { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]; }
inline double xyz_distance(const V4 &a, const V4 &b)
{
    return std::sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]) + (b[2] - a[2]) * (b[2] - a[2]));
}

enum class PositionMode { Absolute, Relative };

struct ToolheadState;

struct PlanningMove
{
    V4     start{};
    V4     end{};
    double distance{ 0. };
    V4     rate{};
    double acceleration{ 0. };
    double junction_deviation{ 0. };
    double max_start_v2{ 0. };
    double max_cruise_v2{ 0. };
    double max_dv2{ 0. };
    double max_smoothed_v2{ 0. };
    double smoothed_dv2{ 0. };

    double start_v{ 0. };
    double cruise_v{ 0. };
    double end_v{ 0. };

    // Index of the G-code line, which generated this move.
    size_t line{ 0 };

    bool is_kinematic_move() const { return !xyz_equal(start, end); }
    bool is_extrude_move() const { return std::abs(end[3] - start[3]) >= DBL_EPSILON; }
    bool is_extrude_only_move() const { return !is_kinematic_move() && is_extrude_move(); }
    bool is_zero_distance() const { return std::abs(distance) < DBL_EPSILON; }

    void limit_speed(double velocity, double accel)
    {
        const double v2 = velocity * velocity;
        if (v2 < max_cruise_v2)
            max_cruise_v2 = v2;
        acceleration = std::min(acceleration, accel);
        max_dv2      = 2. * distance * acceleration;
        smoothed_dv2 = std::min(smoothed_dv2, max_dv2);
    }

    void set_junction(double start_v2, double cruise_v2, double end_v2)
    {
        start_v  = std::sqrt(start_v2);
        cruise_v = std::sqrt(cruise_v2);
        end_v    = std::sqrt(end_v2);
    }

    double accel_distance() const { return (cruise_v * cruise_v - start_v * start_v) * 0.5 / acceleration; }
    double decel_distance() const { return (cruise_v * cruise_v - end_v * end_v) * 0.5 / acceleration; }
    double cruise_distance() const { return std::max(distance - accel_distance() - decel_distance(), 0.); }

    double accel_time() const { return safe_div(accel_distance(), (start_v + cruise_v) * 0.5); }
    double cruise_time() const { return safe_div(cruise_distance(), cruise_v); }
    double decel_time() const { return safe_div(decel_distance(), (end_v + cruise_v) * 0.5); }
    double total_time() const { return accel_time() + cruise_time() + decel_time(); }

    static double safe_div(double a, double b) { return b > 0. ? a / b : 0.; }

    inline void apply_junction(const PlanningMove &prev, const ToolheadState &toolhead);
};

struct ToolheadState
{
    V4                          position{ 0., 0., 0., 0. };
    std::array<PositionMode, 4> position_modes{ PositionMode::Absolute, PositionMode::Absolute, PositionMode::Absolute, PositionMode::Relative };
    PrinterLimits               limits;
    double                      velocity;

    explicit ToolheadState(const PrinterLimits &limits) : limits(limits), velocity(limits.max_velocity) {}

    static double new_element(double v, double old, PositionMode mode) { return mode == PositionMode::Relative ? old + v : v; }

    void set_speed(double v)
    {
        // Klipper refuses non-positive feed rates.
        if (v > 0.)
            velocity = v;
    }

    PlanningMove make_move(const V4 &start, const V4 &end) const
    {
        PlanningMove m;
        m.start              = start;
        m.end                = end;
        m.junction_deviation = limits.junction_deviation;
        if (xyz_equal(start, end)) {
            // Extrude only move.
            const double de       = end[3] - start[3];
            const double move_d   = std::abs(de);
            const double inv_move = move_d > 0. ? 1. / move_d : 0.;
            m.distance      = move_d;
            m.rate          = { 0., 0., 0., de * inv_move };
            m.acceleration  = DBL_MAX;
            m.max_cruise_v2 = velocity * velocity;
            m.max_dv2       = DBL_MAX;
            m.smoothed_dv2  = DBL_MAX;
        } else {
            const double distance = xyz_distance(start, end);
            const double v        = std::min(velocity, limits.max_velocity);
            m.distance      = distance;
            for (size_t i = 0; i < 4; ++i)
                m.rate[i] = (end[i] - start[i]) / distance;
            m.acceleration  = limits.max_acceleration;
            m.max_cruise_v2 = v * v;
            m.max_dv2       = 2. * distance * limits.max_acceleration;
            m.smoothed_dv2  = 2. * distance * limits.accel_to_decel;
        }
        return m;
    }

    void check_move(PlanningMove &m) const
    {
        for (const MoveChecker &c : limits.move_checkers) {
            if (c.type == MoveChecker::Type::Axis) {
                if (m.is_zero_distance())
                    continue;
                const double d = std::abs((m.end[0] - m.start[0]) * c.axis.x() + (m.end[1] - m.start[1]) * c.axis.y() + (m.end[2] - m.start[2]) * c.axis.z());
                if (d == 0.)
                    continue; // The move does not travel along this axis.
                const double ratio = m.distance / d;
                m.limit_speed(c.max_velocity * ratio, c.max_accel * ratio);
            } else {
                if (!m.is_extrude_only_move())
                    continue;
                const double e_rate = m.rate[3];
                if ((m.rate[0] == 0. && m.rate[1] == 0.) || e_rate < 0.) {
                    const double inv_extrude_r = 1. / std::abs(e_rate);
                    m.limit_speed(c.max_velocity * inv_extrude_r, c.max_accel * inv_extrude_r);
                }
            }
        }
    }

    PlanningMove perform_move(const std::array<std::optional<double>, 4> &axes)
    {
        V4 new_pos = position;
        for (size_t i = 0; i < 4; ++i)
            if (axes[i])
                new_pos[i] = new_element(*axes[i], new_pos[i], position_modes[i]);
        PlanningMove m = make_move(position, new_pos);
        check_move(m);
        position = new_pos;
        return m;
    }

    PlanningMove perform_relative_move(const std::array<std::optional<double>, 4> &axes)
    {
        const auto old_modes = position_modes;
        position_modes.fill(PositionMode::Relative);
        PlanningMove m = perform_move(axes);
        position_modes = old_modes;
        return m;
    }

    double extruder_junction_speed_v2(const PlanningMove &cur, const PlanningMove &prev) const
    {
        const double diff_r = std::abs(cur.rate[3] - prev.rate[3]);
        if (diff_r > 0.) {
            const double v = limits.instant_corner_velocity / diff_r;
            return v * v;
        }
        return cur.max_cruise_v2;
    }
};

inline void PlanningMove::apply_junction(const PlanningMove &prev, const ToolheadState &toolhead)
{
    if (!this->is_kinematic_move() || !prev.is_kinematic_move())
        return;

    double junction_cos_theta = -(rate[0] * prev.rate[0] + rate[1] * prev.rate[1] + rate[2] * prev.rate[2]);
    if (junction_cos_theta > 0.999999)
        // Move was not at an angle, skip all this.
        return;
    junction_cos_theta          = std::max(junction_cos_theta, -0.999999);
    const double sin_theta_d2   = std::sqrt(0.5 * (1. - junction_cos_theta));
    const double r              = sin_theta_d2 / (1. - sin_theta_d2);
    const double tan_theta_d2   = sin_theta_d2 / std::sqrt(0.5 * (1. + junction_cos_theta));
    const double move_centripetal_v2      = 0.5 * distance * tan_theta_d2 * acceleration;
    const double prev_move_centripetal_v2 = 0.5 * prev.distance * tan_theta_d2 * prev.acceleration;
    const double extruder_v2              = toolhead.extruder_junction_speed_v2(*this, prev);

    max_start_v2 = std::min({ extruder_v2,
                              r * junction_deviation * acceleration,
                              r * prev.junction_deviation * prev.acceleration,
                              move_centripetal_v2,
                              prev_move_centripetal_v2,
                              max_cruise_v2,
                              prev.max_cruise_v2,
                              prev.max_start_v2 + prev.max_dv2 });
    max_smoothed_v2 = std::min(max_start_v2, prev.max_smoothed_v2 + prev.smoothed_dv2);
}

// A sequence of moves planned together by the lookahead, uninterrupted by a delay.
class MoveSequence
{
public:
    void add_move(PlanningMove m, const ToolheadState &toolhead)
    {
        if (m.distance == 0.)
            return;
        if (!m_moves.empty())
            m.apply_junction(m_moves.back(), toolhead);
        m_moves.emplace_back(m);
    }

    bool empty() const { return m_moves.empty(); }

    // Plans the whole sequence, no more moves will be added.
    void flush() { this->process(false); }

    // Returns the next move with a final velocity profile, if there is one.
    std::optional<PlanningMove> next_move()
    {
        this->process(true);
        if (m_flush_count == 0)
            return std::nullopt;
        PlanningMove m = m_moves.front();
        m_moves.pop_front();
        --m_flush_count;
        return m;
    }

private:
    // Port of Klipper's LookAheadQueue.flush().
    void process(bool partial)
    {
        if (m_flush_count == m_moves.size())
            // Nothing to flush.
            return;

        struct Delayed { PlanningMove *move; double start_v2; double end_v2; };
        std::vector<Delayed> delayed;

        double next_end_v2      = 0.;
        double next_smoothed_v2 = 0.;
        double peak_cruise_v2   = 0.;

        bool update_flush_count = partial;
        const size_t skip = partial ? m_flush_count : 0;
        if (!partial)
            m_flush_count = m_moves.size();

        for (size_t idx = m_moves.size(); idx-- > skip;) {
            PlanningMove &m = m_moves[idx];
            const double reachable_start_v2    = next_end_v2 + m.max_dv2;
            const double start_v2              = std::min(m.max_start_v2, reachable_start_v2);
            const double reachable_smoothed_v2 = next_smoothed_v2 + m.smoothed_dv2;
            const double smoothed_v2           = std::min(m.max_smoothed_v2, reachable_smoothed_v2);
            if (smoothed_v2 < reachable_smoothed_v2) {
                // It's possible for this move to accelerate.
                if (smoothed_v2 + m.smoothed_dv2 > next_smoothed_v2 || !delayed.empty()) {
                    // This move can decelerate or this is a full accel move after a full decel move.
                    if (update_flush_count && peak_cruise_v2 != 0.) {
                        m_flush_count      = idx;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = std::min(m.max_cruise_v2, (smoothed_v2 + reachable_smoothed_v2) * 0.5);

                    if (!delayed.empty()) {
                        // Propagate peak_cruise_v2 to any delayed moves.
                        if (!update_flush_count && idx < m_flush_count) {
                            double mc_v2 = peak_cruise_v2;
                            for (auto it = delayed.rbegin(); it != delayed.rend(); ++it) {
                                mc_v2 = std::min(mc_v2, it->start_v2);
                                it->move->set_junction(std::min(it->start_v2, mc_v2), mc_v2, std::min(it->end_v2, mc_v2));
                            }
                        }
                        delayed.clear();
                    }
                }
                if (!update_flush_count && idx < m_flush_count) {
                    const double cruise_v2 = std::min({ (start_v2 + reachable_start_v2) * 0.5, m.max_cruise_v2, peak_cruise_v2 });
                    m.set_junction(std::min(start_v2, cruise_v2), cruise_v2, std::min(next_end_v2, cruise_v2));
                }
            } else {
                // Delay calculating this move until peak_cruise_v2 is known.
                delayed.push_back({ &m, start_v2, next_end_v2 });
            }
            next_end_v2      = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }

        if (update_flush_count)
            m_flush_count = 0;
    }

    std::deque<PlanningMove> m_moves;
    size_t                   m_flush_count{ 0 };
};

struct Delay
{
    double duration{ 0. };
    size_t line{ 0 };
};

// Sequence of move sequences separated by delays (dwells, heating, homing).
class OperationSequence
{
public:
    void add_delay(const Delay &delay)
    {
        // A delay flushes the lookahead in Klipper, the preceding moves are final.
        if (!m_ops.empty() && !m_ops.back().is_delay)
            m_ops.back().moves.flush();
        m_ops.push_back({ true, delay, {} });
    }

    void add_move(const PlanningMove &m, const ToolheadState &toolhead)
    {
        if (m_ops.empty() || m_ops.back().is_delay)
            m_ops.push_back({ false, {}, {} });
        m_ops.back().moves.add_move(m, toolhead);
    }

    void flush()
    {
        for (Op &op : m_ops)
            if (!op.is_delay)
                op.moves.flush();
    }

    // Calls the callback with (line, time) for each finalized operation.
    template<typename Fn> void drain(Fn &&fn)
    {
        while (!m_ops.empty()) {
            Op &op = m_ops.front();
            if (op.is_delay) {
                fn(op.delay.line, op.delay.duration);
                m_ops.pop_front();
                continue;
            }
            std::optional<PlanningMove> m = op.moves.next_move();
            const bool exhausted = op.moves.empty();
            if (exhausted)
                m_ops.pop_front();
            if (m)
                fn(m->line, m->total_time());
            else if (!exhausted)
                // The remaining moves of this sequence depend on moves, which were not seen yet.
                break;
        }
    }

private:
    struct Op
    {
        bool         is_delay{ false };
        Delay        delay;
        MoveSequence moves;
    };
    std::deque<Op> m_ops;
};

enum class Plane { XY, XZ, YZ };

} // anonymous namespace

// ---------------------------------------------------------------------------------------------------------------------
// Estimator
// ---------------------------------------------------------------------------------------------------------------------

class Estimator::Impl
{
public:
    explicit Impl(const PrinterLimits &limits) : m_toolhead(limits)
    {
        m_result.line_times.emplace_back(0.f);
    }

    void process_line(std::string_view line)
    {
        const size_t line_id = m_result.line_times.size();
        m_result.line_times.emplace_back(0.f);
        this->process_cmd(parse_line(line), line_id);
        // Drain the finalized moves regularly to keep the memory usage low.
        if (line_id % 1000 == 0)
            this->drain();
    }

    Result finalize()
    {
        m_operations.flush();
        this->drain();
        return std::move(m_result);
    }

private:
    void drain()
    {
        m_operations.drain([this](size_t line, double time) {
            m_result.line_times[line] += float(time);
            m_result.total_time += time;
        });
    }

    void add_move(PlanningMove m, size_t line)
    {
        m.line = line;
        m_operations.add_move(m, m_toolhead);
    }

    void process_cmd(const Command &cmd, size_t line)
    {
        if (std::optional<double> delay = dwell_time(cmd); delay) {
            m_operations.add_delay({ *delay, line });
            return;
        }

        switch (cmd.op) {
        case Command::Op::Move:
            if (cmd.feedrate)
                m_toolhead.set_speed(*cmd.feedrate / 60.);
            if (cmd.axes[0] || cmd.axes[1] || cmd.axes[2] || cmd.axes[3])
                this->add_move(m_toolhead.perform_move(cmd.axes), line);
            break;
        case Command::Op::Traditional:
            this->process_traditional(cmd, line);
            break;
        case Command::Op::Extended:
            this->process_extended(cmd);
            break;
        case Command::Op::Nop:
            if (cmd.comment) {
                std::string_view comment = skip_spaces(*cmd.comment);
                static constexpr std::string_view add_time = "ESTIMATOR_ADD_TIME ";
                if (comment.substr(0, add_time.size()) == add_time) {
                    std::string_view duration = comment.substr(add_time.size());
                    duration = duration.substr(0, duration.find(' '));
                    if (std::optional<double> d = parse_double(duration); d)
                        m_operations.add_delay({ *d, line });
                }
            }
            break;
        }
    }

    static std::optional<double> dwell_time(const Command &cmd)
    {
        // Time assumed for operations of unknown duration (homing, heating, filament change).
        static constexpr double indeterminate = 0.1;
        if (cmd.is('G', 4))
            return cmd.param('P').has_value() ? *cmd.param('P') / 1000. : 0.25;
        if (cmd.is('G', 28) || cmd.is('M', 109) || cmd.is('M', 190) || cmd.is('M', 600))
            return indeterminate;
        if (cmd.op == Command::Op::Extended && cmd.command == "temperature_wait")
            return indeterminate;
        return std::nullopt;
    }

    void process_traditional(const Command &cmd, size_t line)
    {
        ToolheadState &th = m_toolhead;
        if (cmd.letter == 'G') {
            switch (cmd.code) {
            case 2:
            case 3: this->generate_arc(cmd, cmd.code == 2, line); break;
            case 10: this->firmware_retract(line); break;
            case 11: this->firmware_unretract(line); break;
            case 17: m_plane = Plane::XY; break;
            case 18: m_plane = Plane::XZ; break;
            case 19: m_plane = Plane::YZ; break;
            case 90:
                // Not handled by the original klipper_estimator. Klipper uses relative extrusion in G91 mode.
                m_relative_xyz = false;
                this->update_position_modes();
                break;
            case 91:
                m_relative_xyz = true;
                this->update_position_modes();
                break;
            case 92:
                for (size_t i = 0; i < 4; ++i)
                    if (std::optional<double> v = cmd.param("XYZE"[i]); v)
                        th.position[i] = *v;
                break;
            default: break;
            }
        } else if (cmd.letter == 'M') {
            switch (cmd.code) {
            case 82:
                m_relative_e = false;
                this->update_position_modes();
                break;
            case 83:
                m_relative_e = true;
                this->update_position_modes();
                break;
            case 204: {
                std::optional<double> s = cmd.param('S'), p = cmd.param('P'), t = cmd.param('T');
                if (s)
                    th.limits.set_max_acceleration(*s);
                else if (p && t)
                    th.limits.set_max_acceleration(std::min(*p, *t));
                break;
            }
            default: break;
            }
        }
    }

    void process_extended(const Command &cmd)
    {
        PrinterLimits &limits = m_toolhead.limits;
        if (cmd.command == "set_velocity_limit") {
            if (std::optional<double> v = cmd.ext_param("velocity"); v)
                limits.set_max_velocity(*v);
            if (std::optional<double> v = cmd.ext_param("accel"); v)
                limits.set_max_acceleration(*v);
            if (std::optional<double> v = cmd.ext_param("accel_to_decel"); v)
                limits.set_max_accel_to_decel(*v);
            if (std::optional<double> v = cmd.ext_param("minimum_cruise_ratio"); v)
                limits.set_minimum_cruise_ratio(*v);
            if (std::optional<double> v = cmd.ext_param("square_corner_velocity"); v)
                limits.set_square_corner_velocity(*v);
        } else if (cmd.command == "set_retraction" && limits.firmware_retraction) {
            FirmwareRetractionOptions &fr = *limits.firmware_retraction;
            if (std::optional<double> v = cmd.ext_param("retract_length"); v)
                fr.retract_length = std::max(*v, 0.);
            if (std::optional<double> v = cmd.ext_param("retract_speed"); v)
                fr.retract_speed = std::max(*v, 0.);
            if (std::optional<double> v = cmd.ext_param("unretract_extra_length"); v)
                fr.unretract_extra_length = std::max(*v, 0.);
            if (std::optional<double> v = cmd.ext_param("unretract_speed"); v)
                fr.unretract_speed = std::max(*v, 0.);
            if (std::optional<double> v = cmd.ext_param("lift_z"); v)
                fr.lift_z = std::max(*v, 0.);
        }
    }

    void update_position_modes()
    {
        const PositionMode xyz = m_relative_xyz ? PositionMode::Relative : PositionMode::Absolute;
        m_toolhead.position_modes = { xyz, xyz, xyz, (m_relative_xyz || m_relative_e) ? PositionMode::Relative : PositionMode::Absolute };
    }

    // Emulates a move at the given speed, keeping the current toolhead speed.
    void add_relative_move_at_speed(const std::array<std::optional<double>, 4> &axes, double speed, size_t line)
    {
        const double v = m_toolhead.velocity;
        m_toolhead.velocity = speed;
        this->add_move(m_toolhead.perform_relative_move(axes), line);
        m_toolhead.velocity = v;
    }

    void firmware_retract(size_t line)
    {
        if (!m_toolhead.limits.firmware_retraction || m_retracted)
            return;
        const FirmwareRetractionOptions fr = *m_toolhead.limits.firmware_retraction;
        if (fr.retract_length > 0.)
            this->add_relative_move_at_speed({ std::nullopt, std::nullopt, std::nullopt, -fr.retract_length }, fr.retract_speed, line);
        if (fr.lift_z > 0.)
            this->add_move(m_toolhead.perform_relative_move({ std::nullopt, std::nullopt, fr.lift_z, std::nullopt }), line);
        m_retracted         = true;
        m_lifted_z          = fr.lift_z;
        m_unretract_length  = fr.retract_length + fr.unretract_extra_length;
    }

    void firmware_unretract(size_t line)
    {
        if (!m_toolhead.limits.firmware_retraction || !m_retracted)
            return;
        const FirmwareRetractionOptions &fr = *m_toolhead.limits.firmware_retraction;
        if (m_unretract_length > 0.)
            this->add_relative_move_at_speed({ std::nullopt, std::nullopt, std::nullopt, m_unretract_length }, fr.unretract_speed, line);
        if (m_lifted_z > 0.)
            this->add_move(m_toolhead.perform_relative_move({ std::nullopt, std::nullopt, -m_lifted_z, std::nullopt }), line);
        m_retracted = false;
    }

    // Port of Klipper's gcode_arcs, originally from Marlin's plan_arc().
    void generate_arc(const Command &cmd, bool clockwise, size_t line)
    {
        ToolheadState &th = m_toolhead;
        if (!th.limits.mm_per_arc_segment || *th.limits.mm_per_arc_segment <= 0.)
            return;
        const double mm_per_arc_segment = *th.limits.mm_per_arc_segment;

        size_t alpha_axis, beta_axis, helical_axis;
        double offset_p, offset_q;
        switch (m_plane) {
        case Plane::XY: alpha_axis = 0; beta_axis = 1; helical_axis = 2; offset_p = cmd.param('I').value_or(0.); offset_q = cmd.param('J').value_or(0.); break;
        case Plane::XZ: alpha_axis = 0; beta_axis = 2; helical_axis = 1; offset_p = cmd.param('I').value_or(0.); offset_q = cmd.param('K').value_or(0.); break;
        default:        alpha_axis = 1; beta_axis = 2; helical_axis = 0; offset_p = cmd.param('J').value_or(0.); offset_q = cmd.param('K').value_or(0.); break;
        }
        if (offset_p == 0. && offset_q == 0.)
            // We need at least one coordinate to work with.
            return;

        auto map_coord = [&th](std::optional<double> c, size_t axis) {
            return c ? ToolheadState::new_element(*c, th.position[axis], th.position_modes[axis]) : th.position[axis];
        };
        const std::array<double, 3> target{ map_coord(cmd.param('X'), 0), map_coord(cmd.param('Y'), 1), map_coord(cmd.param('Z'), 2) };
        const std::optional<double> e = cmd.param('E') ? std::optional<double>(map_coord(cmd.param('E'), 3)) : std::nullopt;
        if (std::optional<double> f = cmd.param('F'); f)
            th.set_speed(*f / 60.);

        const std::array<double, 3> current{ th.position[0], th.position[1], th.position[2] };
        const double r_p      = -offset_p;
        const double r_q      = -offset_q;
        const double center_p = current[alpha_axis] - r_p;
        const double center_q = current[beta_axis] - r_q;
        const double rt_alpha = target[alpha_axis] - center_p;
        const double rt_beta  = target[beta_axis] - center_q;
        double angular_travel = std::atan2(r_p * rt_beta - r_q * rt_alpha, r_p * rt_alpha + r_q * rt_beta);
        if (angular_travel < 0.)
            angular_travel += 2. * PI;
        if (clockwise)
            angular_travel -= 2. * PI;
        if (angular_travel == 0. && current[alpha_axis] == target[alpha_axis] && current[beta_axis] == target[beta_axis])
            // Make a circle if the angular rotation is 0 and the target is the current position.
            angular_travel = 2. * PI;

        const double linear_travel = target[helical_axis] - current[helical_axis];
        const double radius        = std::hypot(r_p, r_q);
        const double flat_mm       = radius * angular_travel;
        const double mm_of_travel  = linear_travel != 0. ? std::hypot(flat_mm, linear_travel) : std::abs(flat_mm);
        const size_t segments      = std::max<size_t>(1, size_t(std::floor(mm_of_travel / mm_per_arc_segment)));

        const double theta_per_segment  = angular_travel / double(segments);
        const double linear_per_segment = linear_travel / double(segments);
        double       e_base             = th.position[3];
        const double e_per_move         = e ? (*e - e_base) / double(segments) : 0.;

        const auto old_modes = th.position_modes;
        th.position_modes.fill(PositionMode::Absolute);
        for (size_t i = 1; i <= segments; ++i) {
            std::array<double, 3> coord = target;
            if (i < segments) {
                const double ti     = double(i);
                const double cos_ti = std::cos(ti * theta_per_segment);
                const double sin_ti = std::sin(ti * theta_per_segment);
                coord[alpha_axis]   = center_p + (-offset_p * cos_ti + offset_q * sin_ti);
                coord[beta_axis]    = center_q + (-offset_p * sin_ti - offset_q * cos_ti);
                coord[helical_axis] = current[helical_axis] + ti * linear_per_segment;
            }
            e_base += e_per_move;
            this->add_move(th.perform_move({ coord[0], coord[1], coord[2], e_base }), line);
        }
        th.position_modes = old_modes;
    }

    ToolheadState     m_toolhead;
    OperationSequence m_operations;
    Plane             m_plane{ Plane::XY };
    bool              m_relative_xyz{ false };
    // klipper_estimator assumes relative extrusion until M82 is seen, because the M83 is often hidden in a start macro.
    bool              m_relative_e{ true };

    // Firmware retraction state.
    bool              m_retracted{ false };
    double            m_lifted_z{ 0. };
    double            m_unretract_length{ 0. };

    Result            m_result;
};

Estimator::Estimator(const PrinterLimits &limits) : m_impl(std::make_unique<Impl>(limits)) {}
Estimator::~Estimator() = default;

void Estimator::process_line(std::string_view line) { m_impl->process_line(line); }

Estimator::Result Estimator::finalize() { return m_impl->finalize(); }

Estimator::Result estimate(const PrinterLimits &limits, std::string_view gcode)
{
    Estimator estimator(limits);
    while (!gcode.empty()) {
        const size_t eol = gcode.find('\n');
        estimator.process_line(gcode.substr(0, eol));
        gcode.remove_prefix(eol == std::string_view::npos ? gcode.size() : eol + 1);
    }
    return estimator.finalize();
}

} // namespace KlipperEstimator
} // namespace Slic3r
