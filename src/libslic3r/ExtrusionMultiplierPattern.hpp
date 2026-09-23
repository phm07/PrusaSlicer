///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_ExtrusionMultiplierPattern_hpp_
#define slic3r_ExtrusionMultiplierPattern_hpp_

#include <string>

#include "PrintConfig.hpp"

namespace Slic3r {

// Parameters of the extrusion multiplier calibration pattern: a grid of solid squares, each printed with a different
// extrusion multiplier, with the value embossed on its top surface. The square with the smoothest top surface, where
// the lines just merge without gaps between them and without ridges or a rough, shiny surface from overextrusion,
// tells the extrusion multiplier to use for the filament.
// The extrusion widths, speeds, accelerations, temperatures, retraction, fan and start / end G-code are taken from
// the active print, filament and printer presets, so that the squares are printed the same way as a sliced print.
struct ExtrusionMultiplierPatternParams
{
    // Absolute extrusion multipliers, replacing the filament's extrusion multiplier.
    double em_start    { 0.90 };
    double em_end      { 1.02 };
    double em_step     { 0.02 };

    // Side length of a square, in mm.
    double square_size { 30. };
    // Gap between neighbor squares, in mm.
    double spacing     { 5. };
    // Number of solid layers of a square, including the first and the top layer.
    int    num_layers  { 5 };
    int    perimeters  { 2 };
    // Emboss the extrusion multiplier on the top surface of each square.
    bool   labels      { true };

    int    num_squares() const;
    // Extrusion multiplier of the given square, rounded to 4 decimal places.
    double square_value(int square_idx) const;
    // Label of the given square, with as many decimals as the step needs, at least two: "0.95", "0.975".
    std::string square_label(int square_idx) const;
};

enum class ExtrusionMultiplierPass {
    // A wide range with a large step, to find the neighborhood of the right value.
    Coarse,
    // A narrow range with a small step, around the result of the coarse pass.
    Fine,
};

// Set a range (start, end, step) around center for the given calibration pass.
void set_extrusion_multiplier_range(ExtrusionMultiplierPatternParams &params, double center, ExtrusionMultiplierPass pass);
// Returns true if the range of params matches set_extrusion_multiplier_range() for the given center and pass.
bool is_extrusion_multiplier_range(const ExtrusionMultiplierPatternParams &params, double center, ExtrusionMultiplierPass pass);

// Generates a complete, ready-to-print G-code file with the extrusion multiplier calibration pattern centered on the
// bed. config is the full print config, i.e. the active print, filament and printer presets merged.
// Throws Slic3r::InvalidArgument (with a translated message) if the parameters are invalid or if the pattern does not
// fit the bed.
std::string generate_extrusion_multiplier_pattern(const DynamicPrintConfig &config, const ExtrusionMultiplierPatternParams &params);

// Size of the pattern on the bed, in mm.
Vec2d extrusion_multiplier_pattern_size(const DynamicPrintConfig &config, const ExtrusionMultiplierPatternParams &params);

} // namespace Slic3r

#endif /* slic3r_ExtrusionMultiplierPattern_hpp_ */
