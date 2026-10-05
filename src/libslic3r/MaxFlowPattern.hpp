///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_MaxFlowPattern_hpp_
#define slic3r_MaxFlowPattern_hpp_

#include <string>

#include "PrintConfig.hpp"

namespace Slic3r {

// Parameters of the maximum volumetric flow calibration pattern: a single wall tower of a stadium shape (two straight
// walls along X connected by semicircles) standing on a first layer base. The wall is printed in bands of increasing
// volumetric flow, the print speed of a band is its flow divided by the cross section of the extrusion. The height
// at which the straight walls start to show gaps, a rough or matte surface or layers not bonding tells the maximum
// volumetric flow the hotend can melt, to be set as the filament's max volumetric speed.
// The volumetric speed limits of the print and filament presets are ignored, they are what is being calibrated.
// Temperatures, fan, retraction and start / end G-code are taken from the active presets.
struct MaxFlowPatternParams
{
    // Volumetric flow of the first and the last band, and the increment between neighbor bands, in mm^3/s.
    double flow_start      { 5. };
    double flow_end        { 20. };
    double flow_step       { 1. };
    // Height of a band, in mm, rounded to whole layers.
    double band_height     { 2. };

    // Layer height of the wall, in mm, zero for the layer height of the print preset.
    double layer_height    { 0. };
    // Extrusion width of the wall, in mm, zero for the external perimeter extrusion width of the print preset.
    double extrusion_width { 0. };
    // Length of the straight walls along X, in mm. Longer walls leave more room to accelerate to the band's speed.
    double length          { 100. };

    // Show the flow of the current band on the printer display (M117).
    bool   show_on_display { true };

    int    num_bands() const;
    // Volumetric flow of the given band, rounded to 4 decimal places.
    double band_flow(int band_idx) const;
};

// Generates a complete, ready-to-print G-code file with the maximum volumetric flow calibration pattern centered on
// the bed. config is the full print config, i.e. the active print, filament and printer presets merged.
// Throws Slic3r::InvalidArgument (with a translated message) if the parameters are invalid, if the pattern does not
// fit the bed or if the printer's machine limits do not allow reaching the flow of the last band.
std::string generate_max_flow_pattern(const DynamicPrintConfig &config, const MaxFlowPatternParams &params);

// Size of the pattern on the bed, in mm.
Vec2d max_flow_pattern_size(const DynamicPrintConfig &config, const MaxFlowPatternParams &params);

// Geometry of the bands, as printed by generate_max_flow_pattern().
struct MaxFlowPatternLayout
{
    double first_layer_height;
    double layer_height;
    double extrusion_width;
    int    layers_per_band;
    // Height of a band, a multiple of the layer height.
    double band_height;
    // Height of the whole tower above the bed.
    double total_height;
};
// Throws Slic3r::InvalidArgument (with a translated message) if the parameters are invalid.
MaxFlowPatternLayout max_flow_pattern_layout(const DynamicPrintConfig &config, const MaxFlowPatternParams &params);

// Volumetric flow of the band at the given height above the bed, as measured on the printed tower.
// Returns the flow of the first band for heights within the base, and of the last band above the tower.
double max_flow_at_height(const MaxFlowPatternParams &params, const MaxFlowPatternLayout &layout, double height);

} // namespace Slic3r

#endif /* slic3r_MaxFlowPattern_hpp_ */
