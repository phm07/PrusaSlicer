///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GUI_MaxFlowDialog_hpp_
#define slic3r_GUI_MaxFlowDialog_hpp_

#include <string>

#include "GUI_Utils.hpp"
#include "libslic3r/MaxFlowPattern.hpp"

class wxStaticText;
class wxButton;
class wxFlexGridSizer;
class SpinInput;
class SpinInputDouble;
class CheckBox;

namespace Slic3r {
namespace GUI {

// Parameters of the maximum volumetric flow calibration pattern, with the usage instructions and a calculator showing
// how the measured failure height converts to the flow. The pattern is generated for the active presets when the
// dialog is closed with Generate, see gcode() and filename().
class MaxFlowDialog : public DPIDialog
{
public:
    MaxFlowDialog(wxWindow *parent);

    // Valid after ShowModal() returned wxID_OK.
    const std::string& gcode() const { return m_gcode; }
    // Default file name for the export / upload of the G-code.
    std::string        filename() const;

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    // Max volumetric speed of the filament the pattern is printed with, zero if not limited.
    static double filament_max_volumetric_speed();

    void load_params();
    void save_params() const;
    // Fill the controls from m_params.
    void write_controls();
    // Read m_params from the controls.
    void read_controls();
    // Validate the parameters, update the status line and the result.
    void update_status();
    void update_filament_info();
    // Show the calculation of the flow from the measured failure height.
    void update_result();
    void reset_to_defaults();
    void generate();

    ::SpinInputDouble* add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, double min, double max, double inc, int digits);
    ::CheckBox*        add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip);

    MaxFlowPatternParams m_params;
    std::string          m_gcode;
    // Set while the controls are being filled in, to ignore their change events.
    bool                 m_updating_controls { false };

    ::SpinInputDouble *m_flow_start { nullptr };
    ::SpinInputDouble *m_flow_end { nullptr };
    ::SpinInputDouble *m_flow_step { nullptr };
    ::SpinInputDouble *m_band_height { nullptr };
    ::SpinInputDouble *m_layer_height { nullptr };
    ::SpinInputDouble *m_extrusion_width { nullptr };
    ::SpinInputDouble *m_length { nullptr };
    ::CheckBox        *m_show_on_display { nullptr };
    ::SpinInputDouble *m_height { nullptr };

    wxStaticText      *m_filament_info { nullptr };
    wxStaticText      *m_wall_info { nullptr };
    wxStaticText      *m_result_info { nullptr };
    wxStaticText      *m_status { nullptr };
    wxButton          *m_btn_generate { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_MaxFlowDialog_hpp_
