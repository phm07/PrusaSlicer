///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "MaxFlowDialog.hpp"

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
#include "slic3r/Utils/ASCIIFolding.hpp"

namespace Slic3r {
namespace GUI {

// Parameters of the last generated pattern.
static const std::string APP_CONFIG_SECTION = "max_flow_calibration";

static wxString to_string(double value, int decimals)
{
    const double scale = std::pow(10., decimals);
    return from_u8(float_to_string_decimal_point(std::round(value * scale) / scale));
}

MaxFlowDialog::MaxFlowDialog(wxWindow *parent) :
    DPIDialog(parent, wxID_ANY, _L("Maximum Volumetric Flow Calibration"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE)
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
            _L("A thin wall is printed in bands, each one faster than the band below it.") + "\n\n" +
            _L("1. Select the profiles and set the filament temperature you print with.") + "\n" +
            _L("2. Click Generate and print the tower.") + "\n" +
            _L("3. On the long straight walls, find where gaps, rough surface or weak layers start. "
               "Measure that height from the bed.") + "\n" +
            _L("4. Enter the height below to get the flow. Set it as Max volumetric speed in "
               "Filament Settings > Advanced and save the profile.") + "\n" +
            _L("5. No failure? Raise the flow range. Fails in the first band? Lower it."));
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
        auto [box, box_sizer, grid] = new_group(left, _L("Volumetric flow"));
        m_filament_info = new wxStaticText(box, wxID_ANY, wxEmptyString);
        box_sizer->Insert(0, m_filament_info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, em / 2);
        m_flow_start  = add_double(box, grid, _L("Start"), _L("mm³/s"), _L("Flow of the bottom band."), 0.5, 200., 1., 1);
        m_flow_end    = add_double(box, grid, _L("End"), _L("mm³/s"), _L("Flow of the top band."), 1., 200., 1., 1);
        m_flow_step   = add_double(box, grid, _L("Step"), _L("mm³/s"), _L("Flow increase per band."), 0.1, 20., 0.5, 1);
        m_band_height = add_double(box, grid, _L("Band height"), _L("mm"), _L("Height of each band, rounded to whole layers."), 0.5, 20., 0.5, 1);
    }
    {
        auto [box, box_sizer, grid] = new_group(left, _L("Calculate the flow"));
        m_height = new ::SpinInputDouble(box, "", wxEmptyString, wxDefaultPosition, wxSize(10 * em, -1), wxSP_ARROW_KEYS, 0., 500., 0., 0.5);
        m_height->SetDigits(2);
        m_height->SetToolTip(_L("Height from the bed where the wall starts to fail."));
        m_height->Bind(wxEVT_SPINCTRL, [this](wxCommandEvent &) { this->update_result(); });
        grid->Add(new wxStaticText(box, wxID_ANY, _L("Failure height") + ":"), 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(m_height, 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(new wxStaticText(box, wxID_ANY, _L("mm")), 0, wxALIGN_CENTER_VERTICAL);
        m_result_info = new wxStaticText(box, wxID_ANY, wxEmptyString);
        box_sizer->Add(m_result_info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, em / 2);
    }
    {
        auto [box, box_sizer, grid] = new_group(right, _L("Wall"));
        m_layer_height    = add_double(box, grid, _L("Layer height"), _L("mm"), _L("0 = use the print profile."), 0., 2., 0.05, 2);
        m_extrusion_width = add_double(box, grid, _L("Extrusion width"), _L("mm"), _L("0 = use the print profile."), 0., 5., 0.05, 2);
        m_length          = add_double(box, grid, _L("Length"), _L("mm"),
            _L("Length of the straight walls. Longer walls give the print head room to reach high speeds."), 20., 500., 10., 0);
        m_show_on_display = add_bool(box, grid, _L("Show on display"), _L("Show the flow of each band on the printer's display."));
        m_wall_info = new wxStaticText(box, wxID_ANY, wxEmptyString);
        box_sizer->Add(m_wall_info, 0, wxEXPAND | wxALL, em / 2);
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
    this->update_status();

    SetSizer(main_sizer);
    main_sizer->SetSizeHints(this);
    CenterOnParent();
}

::SpinInputDouble* MaxFlowDialog::add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
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

::CheckBox* MaxFlowDialog::add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip)
{
    auto *ctrl = new ::CheckBox(parent);
    ctrl->SetToolTip(tooltip);
    ctrl->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
    sizer->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(0);
    return ctrl;
}

double MaxFlowDialog::filament_max_volumetric_speed()
{
    // The pattern is printed with the first extruder.
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    const auto *opt = config.option<ConfigOptionFloats>("filament_max_volumetric_speed");
    return opt != nullptr && ! opt->values.empty() ? opt->get_at(0) : 0.;
}

void MaxFlowDialog::load_params()
{
    const AppConfig &cfg = *wxGetApp().app_config;
    MaxFlowPatternParams &p = m_params;
    p = MaxFlowPatternParams();
    auto get_double = [&cfg](const std::string &key, double &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = string_to_double_decimal_point(cfg.get(APP_CONFIG_SECTION, key));
    };
    auto get_bool = [&cfg](const std::string &key, bool &value) {
        if (cfg.has(APP_CONFIG_SECTION, key))
            value = cfg.get(APP_CONFIG_SECTION, key) == "1";
    };
    get_double("flow_start", p.flow_start);
    get_double("flow_end", p.flow_end);
    get_double("flow_step", p.flow_step);
    get_double("band_height", p.band_height);
    get_double("layer_height", p.layer_height);
    get_double("extrusion_width", p.extrusion_width);
    get_double("length", p.length);
    get_bool("show_on_display", p.show_on_display);
}

void MaxFlowDialog::save_params() const
{
    AppConfig &cfg = *wxGetApp().app_config;
    const MaxFlowPatternParams &p = m_params;
    auto set_double = [&cfg](const std::string &key, double value) { cfg.set(APP_CONFIG_SECTION, key, float_to_string_decimal_point(value)); };
    auto set_bool   = [&cfg](const std::string &key, bool value) { cfg.set(APP_CONFIG_SECTION, key, value ? "1" : "0"); };
    set_double("flow_start", p.flow_start);
    set_double("flow_end", p.flow_end);
    set_double("flow_step", p.flow_step);
    set_double("band_height", p.band_height);
    set_double("layer_height", p.layer_height);
    set_double("extrusion_width", p.extrusion_width);
    set_double("length", p.length);
    set_bool("show_on_display", p.show_on_display);
}

void MaxFlowDialog::write_controls()
{
    // Setting the value of a control emits its change event, which would read the other controls, not filled in yet.
    m_updating_controls = true;
    const MaxFlowPatternParams &p = m_params;
    m_flow_start->SetValue(p.flow_start);
    m_flow_end->SetValue(p.flow_end);
    m_flow_step->SetValue(p.flow_step);
    m_band_height->SetValue(p.band_height);
    m_layer_height->SetValue(p.layer_height);
    m_extrusion_width->SetValue(p.extrusion_width);
    m_length->SetValue(p.length);
    m_show_on_display->SetValue(p.show_on_display);
    m_updating_controls = false;
}

void MaxFlowDialog::read_controls()
{
    MaxFlowPatternParams &p = m_params;
    p.flow_start      = m_flow_start->GetValue();
    p.flow_end        = m_flow_end->GetValue();
    p.flow_step       = m_flow_step->GetValue();
    p.band_height     = m_band_height->GetValue();
    p.layer_height    = m_layer_height->GetValue();
    p.extrusion_width = m_extrusion_width->GetValue();
    p.length          = m_length->GetValue();
    p.show_on_display = m_show_on_display->GetValue();
}

void MaxFlowDialog::update_filament_info()
{
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    std::string name;
    if (const auto *opt = config.option<ConfigOptionStrings>("filament_settings_id"); opt != nullptr && ! opt->values.empty())
        name = opt->values.front();
    const double speed = filament_max_volumetric_speed();
    m_filament_info->SetLabel(speed > 0. ?
        format_wxstr(_L("Filament \"%1%\": current max volumetric speed %2% mm³/s"), name, to_string(speed, 2)) :
        format_wxstr(_L("Filament \"%1%\": max volumetric speed not set"), name));
}

void MaxFlowDialog::update_status()
{
    this->read_controls();

    bool ok = false;
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    try {
        const MaxFlowPatternLayout layout = max_flow_pattern_layout(config, m_params);
        m_wall_info->SetLabel(format_wxstr(_L("Layer height %1% mm, extrusion width %2% mm, %3% layers per band."),
            to_string(layout.layer_height, 3), to_string(layout.extrusion_width, 3), layout.layers_per_band));
        const Vec2d size = max_flow_pattern_size(config, m_params);
        const int   n    = m_params.num_bands();
        m_status->SetLabel(format_wxstr(_L("%1% bands from %2% to %3% mm³/s, %4% mm high each. Print size %5% x %6% mm, height %7% mm."), n,
            to_string(m_params.band_flow(0), 2), to_string(m_params.band_flow(n - 1), 2), to_string(layout.band_height, 2),
            to_string(size.x(), 1), to_string(size.y(), 1), to_string(layout.total_height, 1)));
        m_status->SetForegroundColour(wxGetApp().get_label_clr_default());
        ok = true;
    } catch (const std::exception &ex) {
        m_status->SetLabel(from_u8(ex.what()));
        m_status->SetForegroundColour(wxGetApp().get_label_clr_modified());
    }
    m_status->Wrap(70 * em_unit());
    m_wall_info->Wrap(30 * em_unit());
    m_btn_generate->Enable(ok);
    this->update_result();

    if (GetSizer() != nullptr)
        this->Layout();
}

void MaxFlowDialog::update_result()
{
    // The flow of the last band printed in full below the failure height, see max_flow_at_height().
    wxString info;
    try {
        const MaxFlowPatternLayout layout = max_flow_pattern_layout(wxGetApp().preset_bundle->full_config(), m_params);
        const MaxFlowPatternParams &p = m_params;
        const double height = m_height->GetValue();
        if (height <= 0.) {
            // The formula with the parameters filled in, to show how the height converts to the flow.
            info = format_wxstr(_L("Full bands = (height − %1% first layer) ÷ %2% band height, rounded down"),
                                to_string(layout.first_layer_height, 2), to_string(layout.band_height, 2)) + "\n" +
                   format_wxstr(_L("Max flow = %1% + (full bands − 1) × %2% mm³/s"),
                                to_string(p.flow_start, 2), to_string(p.flow_step, 2)) + "\n\n" +
                   _L("Enter the failure height to calculate it.");
        } else if (height > layout.total_height + EPSILON) {
            info = format_wxstr(_L("The tower is only %1% mm high."), to_string(layout.total_height, 2));
        } else {
            const double bands_exact = (height - layout.first_layer_height) / layout.band_height;
            const int    bands       = std::min(int(std::floor(bands_exact + EPSILON)), p.num_bands());
            info = format_wxstr(_L("Full bands = (%1% − %2%) ÷ %3% = %4% → %5%"), to_string(height, 2),
                                to_string(layout.first_layer_height, 2), to_string(layout.band_height, 2),
                                to_string(std::max(0., bands_exact), 2), std::max(0, bands));
            if (bands <= 0)
                info += "\n\n" + _L("The wall fails in the first band. Lower the flow range and print again.");
            else if (bands >= p.num_bands())
                info += "\n\n" + _L("The wall did not fail. Raise the flow range and print again.");
            else
                info += "\n" + format_wxstr(_L("Max flow = %1% + (%2% − 1) × %3% = %4% mm³/s"), to_string(p.flow_start, 2),
                                            bands, to_string(p.flow_step, 2), to_string(p.band_flow(bands - 1), 2)) +
                        "\n\n" + format_wxstr(_L("Set %1% mm³/s as Max volumetric speed in Filament Settings > Advanced."),
                                              to_string(p.band_flow(bands - 1), 2));
        }
    } catch (const std::exception &) {
        info = _L("Fix the parameters above to calculate the flow.");
    }
    m_result_info->SetLabel(info);
    m_result_info->Wrap(30 * em_unit());
}

void MaxFlowDialog::reset_to_defaults()
{
    m_params = MaxFlowPatternParams();
    this->write_controls();
    this->update_status();
}

void MaxFlowDialog::generate()
{
    this->read_controls();
    try {
        m_gcode = generate_max_flow_pattern(wxGetApp().preset_bundle->full_config(), m_params);
    } catch (const std::exception &ex) {
        show_error(this, ex.what(), true);
        return;
    }
    this->save_params();
    EndModal(wxID_OK);
}

std::string MaxFlowDialog::filename() const
{
    std::string filament;
    const DynamicPrintConfig config = wxGetApp().preset_bundle->full_config();
    if (const auto *opt = config.option<ConfigOptionStrings>("filament_settings_id"); opt != nullptr && ! opt->values.empty())
        filament = opt->values.front();
    std::string name = "max_flow_" + filament;
    for (char &c : name)
        if (std::string_view("<>:\"/\\|?* ").find(c) != std::string_view::npos)
            c = '_';
    return fold_utf8_to_ascii(name) + ".gcode";
}

void MaxFlowDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    msw_buttons_rescale(this, em_unit(), { wxID_OK, wxID_CANCEL });
    this->Layout();
    this->Fit();
    this->Refresh();
}

} // namespace GUI
} // namespace Slic3r
