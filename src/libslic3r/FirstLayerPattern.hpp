///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_FirstLayerPattern_hpp_
#define slic3r_FirstLayerPattern_hpp_

#include <string>

#include "PrintConfig.hpp"

namespace Slic3r {

// Parameters of the first layer calibration pattern: a grid of single layer squares spread evenly over the print bed,
// from margin away from its edges. Comparing the squares shows whether the first layer is squished the same over the
// whole bed, i.e. whether the bed is level and the mesh bed leveling works, and whether the Z offset is right.
// The extrusion widths, speeds, accelerations, temperatures, retraction, fan and start / end G-code of the first layer
// are taken from the active print, filament and printer presets, so that the squares are printed the same way as
// the first layer of a sliced print.
struct FirstLayerPatternParams
{
    // Number of squares along X and Y.
    int    columns     { 3 };
    int    rows        { 3 };
    // Side length of a square, in mm.
    double square_size { 30. };
    // Distance of the outer squares from the edges of the bed, in mm.
    double margin      { 10. };
    int    perimeters  { 1 };
};

// Generates a complete, ready-to-print G-code file with the first layer calibration pattern.
// config is the full print config, i.e. the active print, filament and printer presets merged.
// On a bed which is not rectangular, the squares are pulled towards the bed center until they fit the bed.
// Throws Slic3r::InvalidArgument (with a translated message) if the parameters are invalid or if the squares
// do not fit the bed.
std::string generate_first_layer_pattern(const DynamicPrintConfig &config, const FirstLayerPatternParams &params);

// Size of the pattern on the bed, in mm.
Vec2d first_layer_pattern_size(const DynamicPrintConfig &config, const FirstLayerPatternParams &params);

} // namespace Slic3r

#endif /* slic3r_FirstLayerPattern_hpp_ */
