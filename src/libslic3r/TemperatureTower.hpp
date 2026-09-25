///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_TemperatureTower_hpp_
#define slic3r_TemperatureTower_hpp_

#include <string>

#include "Point.hpp"

namespace Slic3r {

class DynamicPrintConfig;
class Model;

// Parameters of the temperature tower: floors of the "Smart compact temperature calibration tower" by gaaZolee
// (resources/calibration/temperature_tower) stacked on each other, each printed with a different nozzle temperature,
// from the bottom one up. A floor has a cone in a round window for stringing, a bridge, a 45 degrees overhang and
// a groove separating it from the floor below, and the temperature embossed on its front face. Comparing the floors
// tells the temperature to use for the filament.
// Unlike the other calibration patterns, the tower is a model sliced with the active presets, the temperatures are
// changed by custom G-codes at the floors, see create_temperature_tower().
struct TemperatureTowerParams
{
    // Nozzle temperatures of the bottom and of the top floor, in degrees Celsius. The temperature goes up the tower
    // from temp_start to temp_end, in either direction.
    int  temp_start { 230 };
    int  temp_end   { 190 };
    int  temp_step  { 5 };
    // Emboss the temperature on the front face of each floor.
    bool labels     { true };

    int  num_floors() const;
    // Nozzle temperature of the given floor, from the bottom one.
    int  floor_temperature(int floor_idx) const;
};

// Set a range (start, end, step) of the tower around the given temperature, hottest at the bottom.
void set_temperature_tower_range(TemperatureTowerParams &params, int center);

// Clears the model and fills it with the temperature tower: a single object with an instance centered on the bed,
// with a part for each floor and a part with the embossed labels, and a custom G-code setting the temperature at the
// first layer of each floor. The floors are scaled with the nozzle diameter. The object is sliced with the layer
// height of config without a raft and without supports, and with a brim at least 8 nozzle diameters wide.
// config is the full print config, i.e. the active print, filament and printer presets merged. resources_dir is
// the directory with the model of a floor and the font of the labels.
// Throws Slic3r::InvalidArgument (with a translated message) if the parameters are invalid, if the tower does not
// fit the bed or the maximum print height of the printer, or if the resources cannot be loaded.
void create_temperature_tower(Model &model, const DynamicPrintConfig &config, const TemperatureTowerParams &params, const std::string &resources_dir);

// Size of the tower, in mm.
Vec3d temperature_tower_size(const DynamicPrintConfig &config, const TemperatureTowerParams &params, const std::string &resources_dir);

} // namespace Slic3r

#endif /* slic3r_TemperatureTower_hpp_ */
