///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GUI_FirstLayerDialog_hpp_
#define slic3r_GUI_FirstLayerDialog_hpp_

#include <string>

#include "GUI_Utils.hpp"
#include "libslic3r/FirstLayerPattern.hpp"

class wxStaticText;
class wxButton;
class wxFlexGridSizer;
class SpinInput;
class SpinInputDouble;

namespace Slic3r {
namespace GUI {

// Parameters of the first layer calibration pattern, with the usage instructions. The pattern is generated for
// the active presets when the dialog is closed with Generate, see gcode() and filename().
class FirstLayerDialog : public DPIDialog
{
public:
    FirstLayerDialog(wxWindow *parent);

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
    // Validate the parameters and update the status line.
    void update_status();
    void reset_to_defaults();
    void generate();

    ::SpinInputDouble* add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, double min, double max, double inc, int digits);
    ::SpinInput*       add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, int min, int max);

    FirstLayerPatternParams m_params;
    std::string             m_gcode;
    // Set while the controls are being filled in, to ignore their change events.
    bool                    m_updating_controls { false };

    ::SpinInput       *m_columns { nullptr };
    ::SpinInput       *m_rows { nullptr };
    ::SpinInputDouble *m_square_size { nullptr };
    ::SpinInputDouble *m_margin { nullptr };
    ::SpinInput       *m_perimeters { nullptr };

    wxStaticText      *m_status { nullptr };
    wxButton          *m_btn_generate { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_FirstLayerDialog_hpp_
