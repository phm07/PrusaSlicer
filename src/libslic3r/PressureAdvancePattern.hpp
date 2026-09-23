///|/ Pressure advance calibration pattern, ported from Ellis' Pressure Advance / Linear Advance Calibration Tool
///|/ Copyright (C) 2019 Sineos [https://github.com/Sineos]
///|/ Copyright (C) 2022 AndrewEllis93 [https://github.com/AndrewEllis93]
///|/ https://github.com/AndrewEllis93/Pressure_Linear_Advance_Tool, licensed under the GPLv3 or later.
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_PressureAdvancePattern_hpp_
#define slic3r_PressureAdvancePattern_hpp_

#include <string>
#include <vector>

#include "PrintConfig.hpp"

namespace Slic3r {

// Parameters of the pressure advance calibration pattern: a row of chevrons ("<" shapes) printed side by side,
// each with the pressure advance increased by a fixed step. The chevron with the sharpest corner and no
// gaps or bulges at the line ends tells the pressure advance value to use for the filament.
// Everything not listed here (temperatures, retraction, travel speed, fan, start / end G-code...)
// is taken from the active printer, filament and print presets.
struct PressureAdvancePatternParams
{
    enum class Anchor {
        // No anchor, the chevrons are printed directly on the bed.
        None,
        // A frame is printed around the chevrons on the first layer.
        Frame,
        // A solid layer is printed below the chevrons.
        Layer,
    };

    double pa_start          { 0. };
    double pa_end            { 0.08 };
    double pa_step           { 0.005 };

    // Number of layers of the chevrons, including the first layer.
    int    num_layers        { 4 };
    // Number of nested walls of a single chevron.
    int    wall_count        { 3 };
    // Length of a single leg of a chevron, in mm.
    double wall_side_length  { 30. };
    // Gap between neighbor chevrons, in mm.
    double pattern_spacing   { 2. };
    // Angle of the chevron tip, in degrees.
    double corner_angle      { 90. };
    // Rotation of the whole pattern around the bed center, in degrees.
    double print_dir         { 0. };
    // Extrusion width of the chevrons, in percent of the nozzle diameter.
    double line_ratio        { 112.5 };

    Anchor anchor            { Anchor::Frame };
    int    anchor_perimeters { 4 };
    // Extrusion width of the anchor, in percent of the nozzle diameter.
    double anchor_line_ratio { 140. };

    // Print the pressure advance values next to the chevrons.
    bool   number_tab        { true };
    // Print ".005" instead of "0.005".
    bool   no_leading_zero   { false };
    // Show the current pressure advance value on the printer display (M117).
    bool   show_on_display   { true };

    // Print speeds in mm/s.
    double first_layer_speed { 30. };
    double perimeter_speed   { 100. };
    // Printing acceleration in mm/s^2, zero to keep the acceleration configured in the firmware.
    double acceleration      { 0. };

    int    num_patterns() const;
    // Pressure advance value of the given chevron, rounded to 4 decimal places.
    double pattern_value(int pattern_idx) const;
};

enum class PressureAdvanceExtruderType {
    DirectDrive,
    Bowden,
};

// Set a pressure advance range (start, end, step) covering the typical values of the given extruder type.
// Marlin's linear advance K factor has a different scale than Klipper's / RepRapFirmware's pressure advance.
void set_pressure_advance_range(PressureAdvancePatternParams &params, GCodeFlavor flavor, PressureAdvanceExtruderType extruder_type);
// Returns true if the range of params matches set_pressure_advance_range() for the given extruder type.
bool is_pressure_advance_range(const PressureAdvancePatternParams &params, GCodeFlavor flavor, PressureAdvanceExtruderType extruder_type);

// Generates a complete, ready-to-print G-code file with the pressure advance calibration pattern centered on the bed.
// config is the full print config, i.e. the active print, filament and printer presets merged. The pattern is
// printed with the first extruder. The printer's start / end G-code is processed by the placeholder parser the same
// way as for a sliced print. The full config is appended at the end of the file, as for a sliced print, so that the
// G-code viewer can load the file.
// Throws Slic3r::InvalidArgument (with a translated message) if the parameters are invalid,
// if the pattern does not fit the bed or if the G-code flavor does not support pressure advance.
std::string generate_pressure_advance_pattern(const DynamicPrintConfig &config, const PressureAdvancePatternParams &params);

// Size of the pattern on the bed (including the rotation by print_dir), in mm.
Vec2d pressure_advance_pattern_size(const DynamicPrintConfig &config, const PressureAdvancePatternParams &params);

// Returns true if the G-code flavor has a command to set the pressure advance / linear advance.
bool pressure_advance_supported(GCodeFlavor flavor);
// Command setting the pressure advance of the active extruder for the given G-code flavor.
std::string set_pressure_advance_gcode(GCodeFlavor flavor, double value);

} // namespace Slic3r

#endif /* slic3r_PressureAdvancePattern_hpp_ */
