///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "FirstLayerDialog.hpp"

#include <algorithm>
#include <cmath>

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>

#include "GUI.hpp"
#include "GUI_App.hpp"
#include "I18N.hpp"
#include "MsgDialog.hpp"
#include "format.hpp"
#include "wxExtensions.hpp"
#include "Widgets/SpinInput.hpp"

#include "LocalesUtils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/Utils/ASCIIFolding.hpp"

namespace Slic3r {
namespace GUI {

// Parameters of the last generated pattern.
static const std::string APP_CONFIG_SECTION = "first_layer_calibration";

FirstLayerDialog::FirstLayerDialog(wxWindow *parent) :
    DPIDialog(parent, wxID_ANY, _L("First Layer Calibration"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE)
{
#ifdef _WIN32
    wxGetApp().UpdateDarkUI(this);
#else
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
#endif
    SetFont(wxGetApp().normal_font());

    this->load_params();

    const int em = em_unit();
    auto *main_sizer = new wxBoxSizer(wxVERTICAL);

    // Usage instructions.
    {
        auto *box = new wxStaticBox(this, wxID_ANY, _L("How to calibrate"));
        wxGetApp().UpdateDarkUI(box);
        auto *box_sizer = new wxStaticBoxSizer(box, wxVERTICAL);
        auto *text = new wxStaticText(box, wxID_ANY,
            _L("Prints single layer squares spread over the print bed, the same way as the first layer of a sliced print. "
               "Comparing the squares shows whether the nozzle is at the right height above the whole bed.") + "\n\n" +
            _L("1. Select the printer, print and filament profiles to calibrate. Clean the print bed.") + "\n" +
            _L("2. Click Generate. Export the G-code or send it to the printer and print it. "
               "If your printer can adjust Z while printing (Live Z, babystepping), watch the squares being printed and adjust it.") + "\n" +
            _L("3. Look at each square and run a fingernail over it. Right: a smooth, flat surface with the lines merged "
               "together, peeling off the bed in one piece. Nozzle too high: round lines with gaps between them, which "
               "separate easily. Nozzle too low: a rough surface with ridges, thin or transparent spots and material pushed to the edges.") + "\n" +
            _L("4. If all the squares are the same but not right, adjust the Z offset of the printer's firmware, or the "
               "Z offset in Printer Settings > General, and print the pattern again.") + "\n" +
            _L("5. If the squares differ across the bed, the bed is not level: run the bed leveling or adjust the bed "
               "screws on the side of the squares printed too high or too low, then print the pattern again."));
        text->Wrap(70 * em);
        box_sizer->Add(text, 0, wxEXPAND | wxALL, em / 2);
        main_sizer->Add(box_sizer, 0, wxEXPAND | wxALL, em);
    }

    {
        auto *box = new wxStaticBox(this, wxID_ANY, _L("Squares"));
        wxGetApp().UpdateDarkUI(box);
        auto *box_sizer = new wxStaticBoxSizer(box, wxVERTICAL);
        auto *grid = new wxFlexGridSizer(3, em / 2, em);
        box_sizer->Add(grid, 0, wxEXPAND | wxALL, em / 2);
        main_sizer->Add(box_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, em);

        m_columns     = add_int(box, grid, _L("Columns"), wxEmptyString, _L("Number of squares along X."), 1, 10);
        m_rows        = add_int(box, grid, _L("Rows"), wxEmptyString, _L("Number of squares along Y."), 1, 10);
        m_square_size = add_double(box, grid, _L("Size"), _L("mm"), _L("Side length of a square."), 5., 100., 5., 0);
        m_margin      = add_double(box, grid, _L("Margin"), _L("mm"),
            _L("Distance of the outer squares from the edges of the bed. The squares are spread evenly between the margins. "
               "On a bed which is not rectangular, they are pulled towards the bed center until they fit."), 0., 100., 1., 0);
        m_perimeters  = add_int(box, grid, _L("Perimeters"), wxEmptyString, _L("Number of perimeters of a square."), 1, 10);

        const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
        const double layer_height = config.opt_float("layer_height");
        auto *note = new wxStaticText(box, wxID_ANY,
            format_wxstr(_L("First layer height: %1% mm, Z offset: %2% mm."),
                         float_to_string_decimal_point(config.get_abs_value("first_layer_height", layer_height)),
                         float_to_string_decimal_point(config.opt_float("z_offset"))) + "\n" +
            _L("Extrusion widths, speeds, accelerations, temperatures, cooling and retraction of the first layer are taken from the selected profiles."));
        note->Wrap(40 * em);
        box_sizer->Add(note, 0, wxEXPAND | wxALL, em / 2);
    }

    m_status = new wxStaticText(this, wxID_ANY, wxEmptyString);
    main_sizer->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT, em);

    // Buttons: Reset on the left, Generate / Cancel on the right.
    {
        auto *sizer = new wxBoxSizer(wxHORIZONTAL);
        auto *btn_reset = new wxButton(this, wxID_ANY, _L("Reset to defaults"));
        m_btn_generate  = new wxButton(this, wxID_OK, _L("Generate"));
        auto *btn_cancel = new wxButton(this, wxID_CANCEL, _L("Close"));
        m_btn_generate->SetToolTip(_L("Generate the G-code and show it in the preview, from where it can be exported or sent to the printer."));
        for (wxButton *btn : { btn_reset, m_btn_generate, btn_cancel }) {
            wxGetApp().UpdateDarkUI(btn);
            wxGetApp().SetWindowVariantForButton(btn);
        }
        sizer->Add(btn_reset, 0);
        sizer->AddStretchSpacer();
        sizer->Add(m_btn_generate, 0, wxRIGHT, em);
        sizer->Add(btn_cancel, 0);
        btn_reset->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { this->reset_to_defaults(); });
        m_btn_generate->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { this->generate(); });
        m_btn_generate->SetDefault();
        main_sizer->Add(sizer, 0, wxEXPAND | wxALL, em);
    }

    this->write_controls();
    this->update_status();

    SetSizer(main_sizer);
    main_sizer->SetSizeHints(this);
    CenterOnParent();
}

::SpinInputDouble* FirstLayerDialog::add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
                                                const wxString &tooltip, double min, double max, double inc, int digits)
{
    const int em = em_unit();
    auto *ctrl = new ::SpinInputDouble(parent, "", wxEmptyString, wxDefaultPosition, wxSize(10 * em, -1), wxSP_ARROW_KEYS, min, max, min, inc);
    ctrl->SetDigits(digits);
    ctrl->SetToolTip(tooltip);
    ctrl->Bind(wxEVT_SPINCTRL, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
    sizer->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(new wxStaticText(parent, wxID_ANY, sidetext), 0, wxALIGN_CENTER_VERTICAL);
    return ctrl;
}

::SpinInput* FirstLayerDialog::add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
                                       const wxString &tooltip, int min, int max)
{
    const int em = em_unit();
    auto *ctrl = new ::SpinInput(parent, "", wxEmptyString, wxDefaultPosition, wxSize(10 * em, -1), wxSP_ARROW_KEYS, min, max, min);
    ctrl->SetToolTip(tooltip);
    ctrl->Bind(wxEVT_SPINCTRL, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
    sizer->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(new wxStaticText(parent, wxID_ANY, sidetext), 0, wxALIGN_CENTER_VERTICAL);
    return ctrl;
}

void FirstLayerDialog::load_params()
{
    const AppConfig &cfg = *wxGetApp().app_config;
    FirstLayerPatternParams &p = m_params;
    p = FirstLayerPatternParams();
    auto get_double = [&cfg](const std::string &key, double &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = string_to_double_decimal_point(cfg.get(APP_CONFIG_SECTION, key));
    };
    auto get_int = [&cfg](const std::string &key, int &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = std::atoi(cfg.get(APP_CONFIG_SECTION, key).c_str());
    };
    get_int("columns", p.columns);
    get_int("rows", p.rows);
    get_double("square_size", p.square_size);
    get_double("margin", p.margin);
    get_int("perimeters", p.perimeters);
}

void FirstLayerDialog::save_params() const
{
    AppConfig &cfg = *wxGetApp().app_config;
    const FirstLayerPatternParams &p = m_params;
    auto set_double = [&cfg](const std::string &key, double value) { cfg.set(APP_CONFIG_SECTION, key, float_to_string_decimal_point(value)); };
    auto set_int    = [&cfg](const std::string &key, int value) { cfg.set(APP_CONFIG_SECTION, key, std::to_string(value)); };
    set_int("columns", p.columns);
    set_int("rows", p.rows);
    set_double("square_size", p.square_size);
    set_double("margin", p.margin);
    set_int("perimeters", p.perimeters);
}

void FirstLayerDialog::write_controls()
{
    // Setting the value of a control emits its change event, which would read the other controls, not filled in yet.
    m_updating_controls = true;
    const FirstLayerPatternParams &p = m_params;
    m_columns->SetValue(p.columns);
    m_rows->SetValue(p.rows);
    m_square_size->SetValue(p.square_size);
    m_margin->SetValue(p.margin);
    m_perimeters->SetValue(p.perimeters);
    m_updating_controls = false;
}

void FirstLayerDialog::read_controls()
{
    FirstLayerPatternParams &p = m_params;
    p.columns     = m_columns->GetValue();
    p.rows        = m_rows->GetValue();
    p.square_size = m_square_size->GetValue();
    p.margin      = m_margin->GetValue();
    p.perimeters  = m_perimeters->GetValue();
}

void FirstLayerDialog::update_status()
{
    this->read_controls();

    bool ok = false;
    try {
        const Vec2d size = first_layer_pattern_size(wxGetApp().preset_bundle->full_config(), m_params);
        m_status->SetLabel(format_wxstr(_L("%1% squares, print size %2% x %3% mm."), m_params.columns * m_params.rows,
            float_to_string_decimal_point(std::round(size.x() * 10.) / 10.), float_to_string_decimal_point(std::round(size.y() * 10.) / 10.)));
        m_status->SetForegroundColour(wxGetApp().get_label_clr_default());
        ok = true;
    } catch (const std::exception &ex) {
        m_status->SetLabel(from_u8(ex.what()));
        m_status->SetForegroundColour(wxGetApp().get_label_clr_modified());
    }
    m_status->Wrap(70 * em_unit());
    m_btn_generate->Enable(ok);

    if (GetSizer() != nullptr)
        this->Layout();
}

void FirstLayerDialog::reset_to_defaults()
{
    m_params = FirstLayerPatternParams();
    this->write_controls();
    this->update_status();
}

void FirstLayerDialog::generate()
{
    this->read_controls();
    try {
        m_gcode = generate_first_layer_pattern(wxGetApp().preset_bundle->full_config(), m_params);
    } catch (const std::exception &ex) {
        show_error(this, ex.what(), true);
        return;
    }
    this->save_params();
    EndModal(wxID_OK);
}

std::string FirstLayerDialog::filename() const
{
    // The first layer depends on the printer and its bed more than on the filament.
    std::string printer;
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    if (const auto *opt = config.option<ConfigOptionString>("printer_settings_id"); opt != nullptr)
        printer = opt->value;
    std::string name = "first_layer_" + printer;
    for (char &c : name)
        if (std::string_view("<>:\"/\\|?* ").find(c) != std::string_view::npos)
            c = '_';
    return fold_utf8_to_ascii(name) + ".gcode";
}

void FirstLayerDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    msw_buttons_rescale(this, em_unit(), { wxID_OK, wxID_CANCEL });
    this->Layout();
    this->Fit();
    this->Refresh();
}

} // namespace GUI
} // namespace Slic3r
