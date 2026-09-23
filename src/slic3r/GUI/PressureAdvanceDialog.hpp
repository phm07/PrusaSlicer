///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GUI_PressureAdvanceDialog_hpp_
#define slic3r_GUI_PressureAdvanceDialog_hpp_

#include <string>

#include "GUI_Utils.hpp"
#include "libslic3r/PressureAdvancePattern.hpp"

class wxStaticText;
class wxButton;
class wxFlexGridSizer;
class SpinInput;
class SpinInputDouble;
class CheckBox;
class ComboBox;

namespace Slic3r {
namespace GUI {

// Parameters of the pressure advance calibration pattern (Ellis' pattern method). The pattern is generated
// for the active presets when the dialog is closed with Generate, see gcode() and filename().
class PressureAdvanceDialog : public DPIDialog
{
public:
    PressureAdvanceDialog(wxWindow *parent);

    // Valid after ShowModal() returned wxID_OK.
    const std::string& gcode() const { return m_gcode; }
    // Default file name for the export / upload of the G-code.
    std::string        filename() const;

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    void load_params();
    void save_params() const;
    // Fill the controls from m_params.
    void write_controls();
    // Read m_params from the controls.
    void read_controls();
    // Validate the parameters, update the status line and the dependent controls.
    void update_status();
    void on_extruder_type_changed();
    void reset_to_defaults();
    void generate();

    ::SpinInputDouble* add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, double min, double max, double inc, int digits);
    ::SpinInput*       add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, int min, int max);
    ::CheckBox*        add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip);

    GCodeFlavor                  m_flavor;
    PressureAdvancePatternParams m_params;
    std::string                  m_gcode;
    // Set while the controls are being filled in, to ignore their change events.
    bool                         m_updating_controls { false };

    ::ComboBox        *m_extruder_type { nullptr };
    ::SpinInputDouble *m_pa_start { nullptr };
    ::SpinInputDouble *m_pa_end { nullptr };
    ::SpinInputDouble *m_pa_step { nullptr };
    ::SpinInput       *m_num_layers { nullptr };
    ::SpinInput       *m_wall_count { nullptr };
    ::SpinInputDouble *m_wall_side_length { nullptr };
    ::SpinInputDouble *m_pattern_spacing { nullptr };
    ::SpinInputDouble *m_corner_angle { nullptr };
    ::SpinInputDouble *m_print_dir { nullptr };
    ::SpinInputDouble *m_line_ratio { nullptr };
    ::ComboBox        *m_anchor { nullptr };
    ::SpinInput       *m_anchor_perimeters { nullptr };
    ::SpinInputDouble *m_anchor_line_ratio { nullptr };
    ::CheckBox        *m_number_tab { nullptr };
    ::CheckBox        *m_no_leading_zero { nullptr };
    ::CheckBox        *m_show_on_display { nullptr };
    ::SpinInputDouble *m_first_layer_speed { nullptr };
    ::SpinInputDouble *m_perimeter_speed { nullptr };
    ::SpinInputDouble *m_acceleration { nullptr };

    wxStaticText      *m_status { nullptr };
    wxButton          *m_btn_generate { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_PressureAdvanceDialog_hpp_
