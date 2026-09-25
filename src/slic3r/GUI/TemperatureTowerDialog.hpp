///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GUI_TemperatureTowerDialog_hpp_
#define slic3r_GUI_TemperatureTowerDialog_hpp_

#include <memory>

#include "GUI_Utils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TemperatureTower.hpp"

class wxStaticText;
class wxButton;
class wxFlexGridSizer;
class SpinInput;
class CheckBox;

namespace Slic3r {
namespace GUI {

// Parameters of the temperature tower, with the usage instructions and a shortcut to set the result as the filament's
// temperature. The tower model is created for the active presets when the dialog is closed with Generate, see model().
class TemperatureTowerDialog : public DPIDialog
{
public:
    TemperatureTowerDialog(wxWindow *parent);

    // The tower and its custom G-codes, valid after ShowModal() returned wxID_OK.
    const Model& model() const { return *m_model; }

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    // Temperature of the filament the tower is printed with.
    static int filament_temperature();

    void load_params();
    void save_params() const;
    // Fill the controls from m_params.
    void write_controls();
    // Read m_params from the controls.
    void read_controls();
    // Validate the parameters and update the status line.
    void update_status();
    void update_filament_info();
    void reset_to_defaults();
    void generate();

    ::SpinInput*       add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext, const wxString &tooltip, int min, int max);
    ::CheckBox*        add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip);

    TemperatureTowerParams m_params;
    std::unique_ptr<Model> m_model;
    // Set while the controls are being filled in, to ignore their change events.
    bool                   m_updating_controls { false };

    ::SpinInput       *m_temp_start { nullptr };
    ::SpinInput       *m_temp_end { nullptr };
    ::SpinInput       *m_temp_step { nullptr };
    ::CheckBox        *m_labels { nullptr };

    wxStaticText      *m_filament_info { nullptr };
    wxStaticText      *m_status { nullptr };
    wxButton          *m_btn_generate { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_TemperatureTowerDialog_hpp_
