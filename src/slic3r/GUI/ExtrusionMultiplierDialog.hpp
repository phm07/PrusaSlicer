///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GUI_ExtrusionMultiplierDialog_hpp_
#define slic3r_GUI_ExtrusionMultiplierDialog_hpp_

#include <string>

#include "GUI_Utils.hpp"
#include "libslic3r/ExtrusionMultiplierPattern.hpp"

class wxStaticText;
class wxButton;
class wxFlexGridSizer;
class SpinInput;
class SpinInputDouble;
class CheckBox;
class ComboBox;

namespace Slic3r {
namespace GUI {

// Parameters of the extrusion multiplier calibration pattern, with the usage instructions and a shortcut to set
// the result as the filament's extrusion multiplier. The pattern is generated for the active presets when the dialog
// is closed with Generate, see gcode() and filename().
class ExtrusionMultiplierDialog : public DPIDialog
{
public:
    ExtrusionMultiplierDialog(wxWindow *parent);

    // Valid after ShowModal() returned wxID_OK.
    const std::string& gcode() const { return m_gcode; }
    // Default file name for the export / upload of the G-code.
    std::string        filename() const;

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    enum class Pass { Coarse, Fine, Custom };

    // Extrusion multiplier of the filament the pattern is printed with.
    static double filament_extrusion_multiplier();

    void load_params();
    void save_params() const;
    // Fill the controls from m_params.
    void write_controls();
    // Read m_params from the controls.
    void read_controls();
    // Validate the parameters, update the status line and the dependent controls.
    void update_status();
    void update_filament_info();
    void on_pass_changed();
    void reset_to_defaults();
    void apply_result();
    void generate();

    ::SpinInputDouble* add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, double min, double max, double inc, int digits);
    ::SpinInput*       add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, int min, int max);
    ::CheckBox*        add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip);

    ExtrusionMultiplierPatternParams m_params;
    std::string                      m_gcode;
    // Set while the controls are being filled in, to ignore their change events.
    bool                             m_updating_controls { false };

    ::ComboBox        *m_pass { nullptr };
    ::SpinInputDouble *m_em_start { nullptr };
    ::SpinInputDouble *m_em_end { nullptr };
    ::SpinInputDouble *m_em_step { nullptr };
    ::SpinInputDouble *m_square_size { nullptr };
    ::SpinInputDouble *m_spacing { nullptr };
    ::SpinInput       *m_num_layers { nullptr };
    ::SpinInput       *m_perimeters { nullptr };
    ::CheckBox        *m_labels { nullptr };
    ::SpinInputDouble *m_result { nullptr };

    wxStaticText      *m_filament_info { nullptr };
    wxStaticText      *m_apply_info { nullptr };
    wxStaticText      *m_status { nullptr };
    wxButton          *m_btn_generate { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_ExtrusionMultiplierDialog_hpp_
