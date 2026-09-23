///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ExtrusionMultiplierDialog.hpp"

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
#include "Tab.hpp"
#include "format.hpp"
#include "wxExtensions.hpp"
#include "Widgets/CheckBox.hpp"
#include "Widgets/ComboBox.hpp"
#include "Widgets/SpinInput.hpp"

#include "LocalesUtils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/Utils/ASCIIFolding.hpp"

namespace Slic3r {
namespace GUI {

// Parameters of the last generated pattern.
static const std::string APP_CONFIG_SECTION = "extrusion_multiplier_calibration";

static wxString em_to_string(double value)
{
    return from_u8(float_to_string_decimal_point(std::round(value * 10000.) / 10000.));
}

ExtrusionMultiplierDialog::ExtrusionMultiplierDialog(wxWindow *parent) :
    DPIDialog(parent, wxID_ANY, _L("Extrusion Multiplier Calibration"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE)
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

    auto new_group = [this, em](wxSizer *parent_sizer, const wxString &title) {
        auto *box = new wxStaticBox(this, wxID_ANY, title);
        wxGetApp().UpdateDarkUI(box);
        auto *box_sizer = new wxStaticBoxSizer(box, wxVERTICAL);
        auto *grid = new wxFlexGridSizer(3, em / 2, em);
        box_sizer->Add(grid, 0, wxEXPAND | wxALL, em / 2);
        parent_sizer->Add(box_sizer, 0, wxEXPAND | wxBOTTOM, em);
        return std::make_tuple(static_cast<wxWindow*>(box), box_sizer, grid);
    };

    // Usage instructions.
    {
        auto *box = new wxStaticBox(this, wxID_ANY, _L("How to calibrate"));
        wxGetApp().UpdateDarkUI(box);
        auto *box_sizer = new wxStaticBoxSizer(box, wxVERTICAL);
        auto *text = new wxStaticText(box, wxID_ANY,
            _L("Prints squares with increasing extrusion multipliers, each value embossed on its top surface.") + "\n\n" +
            _L("1. Select the printer, print and filament profiles to calibrate. If your firmware supports pressure advance, "
               "calibrate it first (Calibration > Pressure Advance).") + "\n" +
            _L("2. Select the coarse pass and click Generate. Export the G-code or send it to the printer and print it.") + "\n" +
            _L("3. Look at the top surfaces under a light coming from a shallow angle and run a fingernail over them. "
               "Too low values leave gaps or grooves between the lines. Too high values make the surface rough, with ridges "
               "and bulging edges. Pick the smoothest square: the lowest value without any gaps.") + "\n" +
            _L("4. Enter the value embossed on that square under \"Apply the result\" and click Apply. "
               "Save the filament profile to keep the new value.") + "\n" +
            _L("5. Repeat with the fine pass, which is centered on the new value."));
        text->Wrap(70 * em);
        box_sizer->Add(text, 0, wxEXPAND | wxALL, em / 2);
        main_sizer->Add(box_sizer, 0, wxEXPAND | wxALL, em);
    }

    auto *columns = new wxBoxSizer(wxHORIZONTAL);
    auto *left    = new wxBoxSizer(wxVERTICAL);
    auto *right   = new wxBoxSizer(wxVERTICAL);
    columns->Add(left, 1, wxEXPAND | wxRIGHT, em);
    columns->Add(right, 1, wxEXPAND);
    main_sizer->Add(columns, 0, wxEXPAND | wxLEFT | wxRIGHT, em);

    {
        auto [box, box_sizer, grid] = new_group(left, _L("Extrusion multiplier"));
        m_filament_info = new wxStaticText(box, wxID_ANY, wxEmptyString);
        box_sizer->Insert(0, m_filament_info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, em / 2);

        std::vector<wxString> passes { _L("Coarse"), _L("Fine"), _L("Custom") };
        m_pass = new ::ComboBox(box, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(10 * em, -1), int(passes.size()), passes.data(), wxCB_READONLY | DD_NO_CHECK_ICON);
        m_pass->SetToolTip(_L("Coarse: 7 squares from -0.06 to +0.06 around the filament's extrusion multiplier.\n"
                              "Fine: 9 squares from -0.02 to +0.02 around the filament's extrusion multiplier."));
        wxGetApp().UpdateDarkUI(m_pass);
        m_pass->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) { this->on_pass_changed(); });
        grid->Add(new wxStaticText(box, wxID_ANY, _L("Pass") + ":"), 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(m_pass, 0, wxALIGN_CENTER_VERTICAL);
        grid->AddSpacer(0);

        m_em_start = add_double(box, grid, _L("Start"), wxEmptyString, _L("Extrusion multiplier of the first square."), 0.1, 3., 0.01, 3);
        m_em_end   = add_double(box, grid, _L("End"), wxEmptyString, _L("Extrusion multiplier of the last square."), 0.1, 3., 0.01, 3);
        m_em_step  = add_double(box, grid, _L("Step"), wxEmptyString, _L("Increment between neighbor squares."), 0.001, 0.2, 0.005, 3);
    }
    {
        auto [box, box_sizer, grid] = new_group(left, _L("Apply the result"));
        m_result = new ::SpinInputDouble(box, "", wxEmptyString, wxDefaultPosition, wxSize(10 * em, -1), wxSP_ARROW_KEYS, 0.1, 3., 1., 0.005);
        m_result->SetDigits(3);
        m_result->SetToolTip(_L("Value embossed on the square with the smoothest top surface."));
        grid->Add(new wxStaticText(box, wxID_ANY, _L("Best square") + ":"), 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(m_result, 0, wxALIGN_CENTER_VERTICAL);
        auto *btn_apply = new wxButton(box, wxID_ANY, _L("Apply"));
        btn_apply->SetToolTip(_L("Set the value as the extrusion multiplier of the filament profile. The profile is modified, save it to keep the value."));
        wxGetApp().UpdateDarkUI(btn_apply);
        wxGetApp().SetWindowVariantForButton(btn_apply);
        btn_apply->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { this->apply_result(); });
        grid->Add(btn_apply, 0, wxALIGN_CENTER_VERTICAL);
        m_apply_info = new wxStaticText(box, wxID_ANY, wxEmptyString);
        box_sizer->Add(m_apply_info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, em / 2);
    }
    {
        auto [box, box_sizer, grid] = new_group(right, _L("Squares"));
        m_square_size = add_double(box, grid, _L("Size"), _L("mm"), _L("Side length of a square. Larger squares make the top surface easier to judge."), 10., 100., 5., 0);
        m_spacing     = add_double(box, grid, _L("Spacing"), _L("mm"), _L("Gap between neighbor squares."), 1., 50., 1., 0);
        m_num_layers  = add_int(box, grid, _L("Layers"), wxEmptyString,
            _L("Number of solid layers of a square, including the first and the top layer. "
               "The layers below the top one make the top surface independent of the bed."), 2, 30);
        m_perimeters  = add_int(box, grid, _L("Perimeters"), wxEmptyString, _L("Number of perimeters of a square."), 1, 10);
        m_labels      = add_bool(box, grid, _L("Emboss values"),
            _L("Emboss the extrusion multiplier on top of each square, so the squares can be told apart after taking them off the bed."));
        auto *note = new wxStaticText(box, wxID_ANY,
            _L("Extrusion widths, speeds, accelerations, temperatures, cooling and retraction are taken from the selected profiles."));
        note->Wrap(30 * em);
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

    this->update_filament_info();
    this->write_controls();
    m_result->SetValue(filament_extrusion_multiplier());
    this->update_status();

    SetSizer(main_sizer);
    main_sizer->SetSizeHints(this);
    CenterOnParent();
}

::SpinInputDouble* ExtrusionMultiplierDialog::add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
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

::SpinInput* ExtrusionMultiplierDialog::add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
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

::CheckBox* ExtrusionMultiplierDialog::add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip)
{
    auto *ctrl = new ::CheckBox(parent);
    ctrl->SetToolTip(tooltip);
    ctrl->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
    sizer->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(0);
    return ctrl;
}

double ExtrusionMultiplierDialog::filament_extrusion_multiplier()
{
    // The pattern is printed with the first extruder.
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    const auto *opt = config.option<ConfigOptionFloats>("extrusion_multiplier");
    return opt != nullptr && ! opt->values.empty() ? opt->get_at(0) : 1.;
}

void ExtrusionMultiplierDialog::load_params()
{
    const AppConfig &cfg = *wxGetApp().app_config;
    ExtrusionMultiplierPatternParams &p = m_params;
    p = ExtrusionMultiplierPatternParams();
    auto get_double = [&cfg](const std::string &key, double &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = string_to_double_decimal_point(cfg.get(APP_CONFIG_SECTION, key));
    };
    auto get_int = [&cfg](const std::string &key, int &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = std::atoi(cfg.get(APP_CONFIG_SECTION, key).c_str());
    };
    auto get_bool = [&cfg](const std::string &key, bool &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = cfg.get(APP_CONFIG_SECTION, key) == "1";
    };

    // The coarse and fine passes are centered on the current extrusion multiplier of the filament,
    // only a custom range is restored.
    int pass = int(Pass::Coarse);
    get_int("pass", pass);
    if (Pass(std::clamp(pass, 0, 2)) == Pass::Custom) {
        get_double("em_start", p.em_start);
        get_double("em_end", p.em_end);
        get_double("em_step", p.em_step);
    } else
        set_extrusion_multiplier_range(p, filament_extrusion_multiplier(), pass == int(Pass::Fine) ? ExtrusionMultiplierPass::Fine : ExtrusionMultiplierPass::Coarse);
    get_double("square_size", p.square_size);
    get_double("spacing", p.spacing);
    get_int("num_layers", p.num_layers);
    get_int("perimeters", p.perimeters);
    get_bool("labels", p.labels);
}

void ExtrusionMultiplierDialog::save_params() const
{
    AppConfig &cfg = *wxGetApp().app_config;
    const ExtrusionMultiplierPatternParams &p = m_params;
    auto set_double = [&cfg](const std::string &key, double value) { cfg.set(APP_CONFIG_SECTION, key, float_to_string_decimal_point(value)); };
    auto set_int    = [&cfg](const std::string &key, int value) { cfg.set(APP_CONFIG_SECTION, key, std::to_string(value)); };
    auto set_bool   = [&cfg](const std::string &key, bool value) { cfg.set(APP_CONFIG_SECTION, key, value ? "1" : "0"); };
    set_int("pass", std::clamp(m_pass->GetSelection(), 0, 2));
    set_double("em_start", p.em_start);
    set_double("em_end", p.em_end);
    set_double("em_step", p.em_step);
    set_double("square_size", p.square_size);
    set_double("spacing", p.spacing);
    set_int("num_layers", p.num_layers);
    set_int("perimeters", p.perimeters);
    set_bool("labels", p.labels);
}

void ExtrusionMultiplierDialog::write_controls()
{
    // Setting the value of a control emits its change event, which would read the other controls, not filled in yet.
    m_updating_controls = true;
    const ExtrusionMultiplierPatternParams &p = m_params;
    m_em_start->SetValue(p.em_start);
    m_em_end->SetValue(p.em_end);
    m_em_step->SetValue(p.em_step);
    m_square_size->SetValue(p.square_size);
    m_spacing->SetValue(p.spacing);
    m_num_layers->SetValue(p.num_layers);
    m_perimeters->SetValue(p.perimeters);
    m_labels->SetValue(p.labels);
    m_updating_controls = false;
}

void ExtrusionMultiplierDialog::read_controls()
{
    ExtrusionMultiplierPatternParams &p = m_params;
    p.em_start    = m_em_start->GetValue();
    p.em_end      = m_em_end->GetValue();
    p.em_step     = m_em_step->GetValue();
    p.square_size = m_square_size->GetValue();
    p.spacing     = m_spacing->GetValue();
    p.num_layers  = m_num_layers->GetValue();
    p.perimeters  = m_perimeters->GetValue();
    p.labels      = m_labels->GetValue();
}

void ExtrusionMultiplierDialog::update_filament_info()
{
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    std::string name;
    if (const auto *opt = config.option<ConfigOptionStrings>("filament_settings_id"); opt != nullptr && ! opt->values.empty())
        name = opt->values.front();
    m_filament_info->SetLabel(format_wxstr(_L("Filament \"%1%\": current extrusion multiplier %2%"), name, em_to_string(filament_extrusion_multiplier())));
}

void ExtrusionMultiplierDialog::update_status()
{
    this->read_controls();

    // Show which pass the range corresponds to.
    const double center = filament_extrusion_multiplier();
    m_updating_controls = true;
    m_pass->SetSelection(int(
        is_extrusion_multiplier_range(m_params, center, ExtrusionMultiplierPass::Coarse) ? Pass::Coarse :
        is_extrusion_multiplier_range(m_params, center, ExtrusionMultiplierPass::Fine)   ? Pass::Fine : Pass::Custom));
    m_updating_controls = false;

    bool ok = false;
    try {
        const Vec2d size = extrusion_multiplier_pattern_size(wxGetApp().preset_bundle->full_config(), m_params);
        const int   n    = m_params.num_squares();
        m_status->SetLabel(format_wxstr(_L("%1% squares from %2% to %3%, print size %4% x %5% mm."), n,
            from_u8(m_params.square_label(0)), from_u8(m_params.square_label(n - 1)),
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

void ExtrusionMultiplierDialog::on_pass_changed()
{
    if (m_updating_controls)
        return;
    const Pass pass = Pass(std::clamp(m_pass->GetSelection(), 0, 2));
    if (pass == Pass::Custom)
        return;
    this->read_controls();
    set_extrusion_multiplier_range(m_params, filament_extrusion_multiplier(), pass == Pass::Fine ? ExtrusionMultiplierPass::Fine : ExtrusionMultiplierPass::Coarse);
    this->write_controls();
    this->update_status();
}

void ExtrusionMultiplierDialog::reset_to_defaults()
{
    m_params = ExtrusionMultiplierPatternParams();
    set_extrusion_multiplier_range(m_params, filament_extrusion_multiplier(), ExtrusionMultiplierPass::Coarse);
    this->write_controls();
    this->update_status();
}

void ExtrusionMultiplierDialog::apply_result()
{
    auto *tab = dynamic_cast<TabFilament*>(wxGetApp().get_tab(Preset::TYPE_FILAMENT));
    if (tab == nullptr)
        return;
    // The pattern is printed with the first extruder, edit its filament. Switching the extruder may ask
    // what to do with the unsaved changes of the currently edited filament, and may be canceled.
    if (tab->get_active_extruder() != 0 && ! tab->set_active_extruder(0))
        return;

    // Keep the range centered on the filament's value if it was.
    const double old_center = filament_extrusion_multiplier();
    this->read_controls();
    const bool coarse = is_extrusion_multiplier_range(m_params, old_center, ExtrusionMultiplierPass::Coarse);
    const bool fine   = is_extrusion_multiplier_range(m_params, old_center, ExtrusionMultiplierPass::Fine);

    const double value = m_result->GetValue();
    DynamicPrintConfig new_conf = *tab->get_config();
    auto *opt = new_conf.option<ConfigOptionFloats>("extrusion_multiplier");
    if (opt == nullptr)
        return;
    std::fill(opt->values.begin(), opt->values.end(), value);
    tab->load_config(new_conf);

    this->update_filament_info();
    if (coarse || fine) {
        // After the coarse pass, continue with the fine one around the new value.
        set_extrusion_multiplier_range(m_params, filament_extrusion_multiplier(), ExtrusionMultiplierPass::Fine);
        this->write_controls();
    }
    this->update_status();
    m_apply_info->SetLabel(coarse ?
        _L("Done. The fine pass around the new value is selected, generate and print it. Save the filament profile to keep the new value.") :
        _L("Done. Save the filament profile to keep the new value."));
    m_apply_info->Wrap(30 * em_unit());
    this->Layout();
    this->Fit();
}

void ExtrusionMultiplierDialog::generate()
{
    this->read_controls();
    try {
        m_gcode = generate_extrusion_multiplier_pattern(wxGetApp().preset_bundle->full_config(), m_params);
    } catch (const std::exception &ex) {
        show_error(this, ex.what(), true);
        return;
    }
    this->save_params();
    EndModal(wxID_OK);
}

std::string ExtrusionMultiplierDialog::filename() const
{
    std::string filament;
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    if (const auto *opt = config.option<ConfigOptionStrings>("filament_settings_id"); opt != nullptr && ! opt->values.empty())
        filament = opt->values.front();
    std::string name = "extrusion_multiplier_" + filament;
    for (char &c : name)
        if (std::string_view("<>:\"/\\|?* ").find(c) != std::string_view::npos)
            c = '_';
    return fold_utf8_to_ascii(name) + ".gcode";
}

void ExtrusionMultiplierDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    msw_buttons_rescale(this, em_unit(), { wxID_OK, wxID_CANCEL });
    this->Layout();
    this->Fit();
    this->Refresh();
}

} // namespace GUI
} // namespace Slic3r
