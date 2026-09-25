///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "TemperatureTowerDialog.hpp"

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
#include "Widgets/CheckBox.hpp"
#include "Widgets/SpinInput.hpp"

#include "LocalesUtils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Utils.hpp"

namespace Slic3r {
namespace GUI {

// Parameters of the last generated tower.
static const std::string APP_CONFIG_SECTION = "temperature_tower_calibration";


TemperatureTowerDialog::TemperatureTowerDialog(wxWindow *parent) :
    DPIDialog(parent, wxID_ANY, _L("Temperature Tower Calibration"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE)
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
            _L("Each floor of the tower is printed at a different temperature, embossed on its front.") + "\n\n" +
            _L("1. Select the profiles and set the range recommended by the filament manufacturer.") + "\n" +
            _L("2. Click Generate and print the tower. Generate it again if you change the layer height.") + "\n" +
            _L("3. Compare the floors:") + "\n" +
            "      " + _L("Too hot: stringing, glossy blobby surface, curled overhang, sagging bridge.") + "\n" +
            "      " + _L("Too cold: matte rough surface, under-extrusion, floors break apart easily at the grooves.") + "\n" +
            _L("4. Set the coolest floor that looks good and holds together as the filament's temperature."));
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
        auto [box, box_sizer, grid] = new_group(left, _L("Temperature"));
        m_filament_info = new wxStaticText(box, wxID_ANY, wxEmptyString);
        box_sizer->Insert(0, m_filament_info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, em / 2);
        m_temp_start = add_int(box, grid, _L("Bottom"), _L("°C"), _L("Nozzle temperature of the bottom floor."), 100, 500);
        m_temp_end   = add_int(box, grid, _L("Top"), _L("°C"), _L("Nozzle temperature of the top floor."), 100, 500);
        m_temp_step  = add_int(box, grid, _L("Step"), _L("°C"),
            _L("Temperature difference between neighboring floors. Bottom minus top must be a multiple of it."), 1, 50);
    }
    {
        auto [box, box_sizer, grid] = new_group(right, _L("Tower"));
        m_labels = add_bool(box, grid, _L("Emboss values"), _L("Emboss the temperature on the front face of each floor."));
        auto *note = new wxStaticText(box, wxID_ANY, _L("Smart compact temperature calibration tower by gaaZolee, CC BY-SA 3.0."));
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
        m_btn_generate->SetToolTip(_L("Replace the objects on the plate with the tower and slice it."));
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
    this->update_status();

    SetSizer(main_sizer);
    main_sizer->SetSizeHints(this);
    CenterOnParent();
}

::SpinInput* TemperatureTowerDialog::add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
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

::CheckBox* TemperatureTowerDialog::add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip)
{
    auto *ctrl = new ::CheckBox(parent);
    ctrl->SetToolTip(tooltip);
    ctrl->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
    sizer->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(0);
    return ctrl;
}

int TemperatureTowerDialog::filament_temperature()
{
    // The tower is printed with the first extruder.
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    const auto *opt = config.option<ConfigOptionInts>("temperature");
    return opt != nullptr && ! opt->values.empty() ? opt->get_at(0) : 0;
}

void TemperatureTowerDialog::load_params()
{
    const AppConfig &cfg = *wxGetApp().app_config;
    TemperatureTowerParams &p = m_params;
    p = TemperatureTowerParams();
    auto get_bool = [&cfg](const std::string &key, bool &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = cfg.get(APP_CONFIG_SECTION, key) == "1";
    };
    // The temperature range depends on the filament, it is centered on the filament's temperature and not restored.
    if (const int temp = filament_temperature(); temp > 0)
        set_temperature_tower_range(p, temp);
    get_bool("labels", p.labels);
}

void TemperatureTowerDialog::save_params() const
{
    AppConfig &cfg = *wxGetApp().app_config;
    const TemperatureTowerParams &p = m_params;
    auto set_bool = [&cfg](const std::string &key, bool value) { cfg.set(APP_CONFIG_SECTION, key, value ? "1" : "0"); };
    set_bool("labels", p.labels);
}

void TemperatureTowerDialog::write_controls()
{
    // Setting the value of a control emits its change event, which would read the other controls, not filled in yet.
    m_updating_controls = true;
    const TemperatureTowerParams &p = m_params;
    m_temp_start->SetValue(p.temp_start);
    m_temp_end->SetValue(p.temp_end);
    m_temp_step->SetValue(p.temp_step);
    m_labels->SetValue(p.labels);
    m_updating_controls = false;
}

void TemperatureTowerDialog::read_controls()
{
    TemperatureTowerParams &p = m_params;
    p.temp_start   = m_temp_start->GetValue();
    p.temp_end     = m_temp_end->GetValue();
    p.temp_step    = m_temp_step->GetValue();
    p.labels       = m_labels->GetValue();
}

void TemperatureTowerDialog::update_filament_info()
{
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    std::string name;
    if (const auto *opt = config.option<ConfigOptionStrings>("filament_settings_id"); opt != nullptr && ! opt->values.empty())
        name = opt->values.front();
    int first_layer_temp = 0;
    if (const auto *opt = config.option<ConfigOptionInts>("first_layer_temperature"); opt != nullptr && ! opt->values.empty())
        first_layer_temp = opt->get_at(0);
    m_filament_info->SetLabel(format_wxstr(_L("Filament \"%1%\": temperature %2% °C, first layer %3% °C"), name, filament_temperature(), first_layer_temp));
}

void TemperatureTowerDialog::update_status()
{
    this->read_controls();

    bool ok = false;
    try {
        const Vec3d size = temperature_tower_size(wxGetApp().preset_bundle->full_config(), m_params, resources_dir());
        const int   n    = m_params.num_floors();
        auto to_string = [](double v) { return float_to_string_decimal_point(std::round(v * 10.) / 10.); };
        m_status->SetLabel(format_wxstr(_L("%1% floors from %2% to %3% °C, print size %4% x %5% mm, height %6% mm."), n,
            m_params.floor_temperature(0), m_params.floor_temperature(n - 1), to_string(size.x()), to_string(size.y()), to_string(size.z())));
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

void TemperatureTowerDialog::reset_to_defaults()
{
    m_params = TemperatureTowerParams();
    if (const int temp = filament_temperature(); temp > 0)
        set_temperature_tower_range(m_params, temp);
    this->write_controls();
    this->update_status();
}

void TemperatureTowerDialog::generate()
{
    this->read_controls();
    auto model = std::make_unique<Model>();
    try {
        create_temperature_tower(*model, wxGetApp().preset_bundle->full_config(), m_params, resources_dir());
    } catch (const std::exception &ex) {
        show_error(this, ex.what(), true);
        return;
    }
    m_model = std::move(model);
    this->save_params();
    EndModal(wxID_OK);
}

void TemperatureTowerDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    msw_buttons_rescale(this, em_unit(), { wxID_OK, wxID_CANCEL });
    this->Layout();
    this->Fit();
    this->Refresh();
}

} // namespace GUI
} // namespace Slic3r
