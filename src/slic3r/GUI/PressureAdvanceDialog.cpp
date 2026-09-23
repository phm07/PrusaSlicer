///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PressureAdvanceDialog.hpp"

#include <algorithm>

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
#include "Widgets/ComboBox.hpp"
#include "Widgets/SpinInput.hpp"

#include "LocalesUtils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/Utils/ASCIIFolding.hpp"

namespace Slic3r {
namespace GUI {

// Parameters of the last generated pattern.
static const std::string APP_CONFIG_SECTION = "pressure_advance_calibration";

// Items of the extruder type combo box.
enum ExtruderTypeItem { itemDirectDrive, itemBowden, itemCustom };

PressureAdvanceDialog::PressureAdvanceDialog(wxWindow *parent) :
    DPIDialog(parent, wxID_ANY, _L("Pressure Advance Calibration"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE)
{
#ifdef _WIN32
    wxGetApp().UpdateDarkUI(this);
#else
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
#endif
    SetFont(wxGetApp().normal_font());

    m_flavor = wxGetApp().preset_bundle->printers.get_edited_preset().config.option<ConfigOptionEnum<GCodeFlavor>>("gcode_flavor")->value;
    this->load_params();

    const int em = em_unit();
    auto *main_sizer = new wxBoxSizer(wxVERTICAL);

    auto *intro = new wxStaticText(this, wxID_ANY,
        _L("Prints chevrons with increasing pressure advance. Pick the one with the sharpest corner "
           "and enter its value in the filament settings."));
    intro->Wrap(60 * em);
    main_sizer->Add(intro, 0, wxEXPAND | wxALL, em);

    auto new_group = [this, em](wxSizer *parent_sizer, const wxString &title) {
        auto *box = new wxStaticBox(this, wxID_ANY, title);
        wxGetApp().UpdateDarkUI(box);
        auto *box_sizer = new wxStaticBoxSizer(box, wxVERTICAL);
        auto *grid = new wxFlexGridSizer(3, em / 2, em);
        box_sizer->Add(grid, 0, wxEXPAND | wxALL, em / 2);
        parent_sizer->Add(box_sizer, 0, wxEXPAND | wxBOTTOM, em);
        return std::make_pair(static_cast<wxWindow*>(box), grid);
    };
    auto add_combo = [this, em](wxWindow *parent, wxFlexGridSizer *grid, const wxString &label, const wxString &tooltip, std::initializer_list<wxString> items) {
        std::vector<wxString> choices(items);
        auto *combo = new ::ComboBox(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(10 * em, -1), int(choices.size()), choices.data(), wxCB_READONLY | DD_NO_CHECK_ICON);
        combo->SetToolTip(tooltip);
        wxGetApp().UpdateDarkUI(combo);
        grid->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
        grid->Add(combo, 0, wxALIGN_CENTER_VERTICAL);
        grid->AddSpacer(0);
        return combo;
    };

    auto *columns = new wxBoxSizer(wxHORIZONTAL);
    auto *left    = new wxBoxSizer(wxVERTICAL);
    auto *right   = new wxBoxSizer(wxVERTICAL);
    columns->Add(left, 1, wxEXPAND | wxRIGHT, em);
    columns->Add(right, 1, wxEXPAND);
    main_sizer->Add(columns, 0, wxEXPAND | wxLEFT | wxRIGHT, em);

    const bool marlin = m_flavor == gcfMarlinLegacy || m_flavor == gcfMarlinFirmware;
    {
        auto [box, grid] = new_group(left, marlin ? _L("Linear advance (K factor)") : _L("Pressure advance"));
        m_extruder_type = add_combo(box, grid, _L("Extruder"),
            _L("Sets a range of values covering the typical pressure advance of the extruder type. "
               "Bowden extruders need much higher values than direct drive extruders."),
            { _L("Direct drive"), _L("Bowden"), _L("Custom") });
        m_extruder_type->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) { this->on_extruder_type_changed(); });
        m_pa_start = add_double(box, grid, _L("Start"), wxEmptyString, _L("Value of the first chevron."), 0., 10., 0.001, 4);
        m_pa_end   = add_double(box, grid, _L("End"), wxEmptyString, _L("Value of the last chevron."), 0., 10., 0.001, 4);
        m_pa_step  = add_double(box, grid, _L("Step"), wxEmptyString, _L("Increment between neighbor chevrons."), 0.0001, 1., 0.001, 4);
    }
    {
        auto [box, grid] = new_group(left, _L("Speed"));
        m_first_layer_speed = add_double(box, grid, _L("First layer"), _L("mm/s"), _L("Print speed of the first layer, the anchor and the labels."), 1., 1000., 5., 0);
        m_perimeter_speed   = add_double(box, grid, _L("Chevrons"), _L("mm/s"),
            _L("Print speed of the chevrons above the first layer. A high speed makes the effect of pressure advance easier to see."), 1., 1000., 10., 0);
        m_acceleration      = add_double(box, grid, _L("Acceleration"), _L("mm/s²"), _L("Printing acceleration. Zero keeps the acceleration configured in the firmware."), 0., 100000., 500., 0);
    }
    {
        auto [box, grid] = new_group(left, _L("Labels"));
        m_number_tab      = add_bool(box, grid, _L("Print values"), _L("Print the value below every other chevron."));
        m_no_leading_zero = add_bool(box, grid, _L("No leading zero"), _L("Print \".005\" instead of \"0.005\" to save space."));
        m_show_on_display = add_bool(box, grid, _L("Show on display"), _L("Show the current value on the printer display (M117)."));
    }
    {
        auto [box, grid] = new_group(right, _L("Pattern"));
        m_num_layers       = add_int(box, grid, _L("Layers"), wxEmptyString, _L("Number of layers, including the first layer."), 1, 50);
        m_wall_count       = add_int(box, grid, _L("Walls"), wxEmptyString, _L("Number of nested walls of each chevron."), 1, 10);
        m_wall_side_length = add_double(box, grid, _L("Side length"), _L("mm"), _L("Length of each leg of a chevron."), 5., 200., 1., 1);
        m_pattern_spacing  = add_double(box, grid, _L("Spacing"), _L("mm"), _L("Gap between neighbor chevrons."), 0., 20., 0.5, 1);
        m_corner_angle     = add_double(box, grid, _L("Corner angle"), _L("°"), _L("Angle of the chevron tip."), 10., 170., 5., 0);
        m_print_dir        = add_double(box, grid, _L("Print direction"), _L("°"), _L("Rotation of the pattern around the bed center, clockwise."), 0., 359., 45., 0);
        m_line_ratio       = add_double(box, grid, _L("Line width"), _L("%"), _L("Extrusion width of the chevrons, in percent of the nozzle diameter."), 50., 300., 5., 1);
    }
    {
        auto [box, grid] = new_group(right, _L("Anchor"));
        m_anchor = add_combo(box, grid, _L("Type"),
            _L("Improves the bed adhesion of the chevrons: a frame around them or a solid layer below them."),
            { _L("None"), _L("Frame"), _L("Solid layer") });
        m_anchor->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
        m_anchor_perimeters = add_int(box, grid, _L("Perimeters"), wxEmptyString, _L("Number of perimeters of the anchor."), 1, 20);
        m_anchor_line_ratio = add_double(box, grid, _L("Line width"), _L("%"), _L("Extrusion width of the anchor, in percent of the nozzle diameter."), 50., 300., 5., 1);
    }

    m_status = new wxStaticText(this, wxID_ANY, wxEmptyString);
    main_sizer->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT, em);

    // Buttons: Reset on the left, Generate / Cancel on the right.
    {
        auto *sizer = new wxBoxSizer(wxHORIZONTAL);
        auto *btn_reset = new wxButton(this, wxID_ANY, _L("Reset to defaults"));
        m_btn_generate  = new wxButton(this, wxID_OK, _L("Generate"));
        auto *btn_cancel = new wxButton(this, wxID_CANCEL, _L("Cancel"));
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

::SpinInputDouble* PressureAdvanceDialog::add_double(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
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

::SpinInput* PressureAdvanceDialog::add_int(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &sidetext,
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

::CheckBox* PressureAdvanceDialog::add_bool(wxWindow *parent, wxFlexGridSizer *sizer, const wxString &label, const wxString &tooltip)
{
    auto *ctrl = new ::CheckBox(parent);
    ctrl->SetToolTip(tooltip);
    ctrl->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { if (! m_updating_controls) this->update_status(); });
    sizer->Add(new wxStaticText(parent, wxID_ANY, label + ":"), 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(0);
    return ctrl;
}

void PressureAdvanceDialog::load_params()
{
    // Defaults: Ellis' defaults with a range for a direct drive extruder, which is the most common.
    m_params = PressureAdvancePatternParams();
    set_pressure_advance_range(m_params, m_flavor, PressureAdvanceExtruderType::DirectDrive);

    // Parameters of the last generated pattern.
    const AppConfig &cfg = *wxGetApp().app_config;
    PressureAdvancePatternParams &p = m_params;
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
    // The pressure advance range depends on the firmware, only reuse it for the same firmware.
    if (cfg.has(APP_CONFIG_SECTION, "gcode_flavor") && cfg.get(APP_CONFIG_SECTION, "gcode_flavor") == std::to_string(int(m_flavor))) {
        get_double("pa_start", p.pa_start);
        get_double("pa_end", p.pa_end);
        get_double("pa_step", p.pa_step);
    }
    get_int("num_layers", p.num_layers);
    get_int("wall_count", p.wall_count);
    get_double("wall_side_length", p.wall_side_length);
    get_double("pattern_spacing", p.pattern_spacing);
    get_double("corner_angle", p.corner_angle);
    get_double("print_dir", p.print_dir);
    get_double("line_ratio", p.line_ratio);
    int anchor = int(p.anchor);
    get_int("anchor", anchor);
    p.anchor = PressureAdvancePatternParams::Anchor(std::clamp(anchor, 0, 2));
    get_int("anchor_perimeters", p.anchor_perimeters);
    get_double("anchor_line_ratio", p.anchor_line_ratio);
    get_bool("number_tab", p.number_tab);
    get_bool("no_leading_zero", p.no_leading_zero);
    get_bool("show_on_display", p.show_on_display);
    get_double("first_layer_speed", p.first_layer_speed);
    get_double("perimeter_speed", p.perimeter_speed);
    get_double("acceleration", p.acceleration);
}

void PressureAdvanceDialog::save_params() const
{
    AppConfig &cfg = *wxGetApp().app_config;
    const PressureAdvancePatternParams &p = m_params;
    auto set_double = [&cfg](const std::string &key, double value) { cfg.set(APP_CONFIG_SECTION, key, float_to_string_decimal_point(value)); };
    auto set_int    = [&cfg](const std::string &key, int value) { cfg.set(APP_CONFIG_SECTION, key, std::to_string(value)); };
    auto set_bool   = [&cfg](const std::string &key, bool value) { cfg.set(APP_CONFIG_SECTION, key, value ? "1" : "0"); };
    set_int("gcode_flavor", int(m_flavor));
    set_double("pa_start", p.pa_start);
    set_double("pa_end", p.pa_end);
    set_double("pa_step", p.pa_step);
    set_int("num_layers", p.num_layers);
    set_int("wall_count", p.wall_count);
    set_double("wall_side_length", p.wall_side_length);
    set_double("pattern_spacing", p.pattern_spacing);
    set_double("corner_angle", p.corner_angle);
    set_double("print_dir", p.print_dir);
    set_double("line_ratio", p.line_ratio);
    set_int("anchor", int(p.anchor));
    set_int("anchor_perimeters", p.anchor_perimeters);
    set_double("anchor_line_ratio", p.anchor_line_ratio);
    set_bool("number_tab", p.number_tab);
    set_bool("no_leading_zero", p.no_leading_zero);
    set_bool("show_on_display", p.show_on_display);
    set_double("first_layer_speed", p.first_layer_speed);
    set_double("perimeter_speed", p.perimeter_speed);
    set_double("acceleration", p.acceleration);
}

void PressureAdvanceDialog::write_controls()
{
    // Setting the value of a control emits its change event, which would read the other controls, not filled in yet.
    m_updating_controls = true;
    const PressureAdvancePatternParams &p = m_params;
    m_pa_start->SetValue(p.pa_start);
    m_pa_end->SetValue(p.pa_end);
    m_pa_step->SetValue(p.pa_step);
    m_num_layers->SetValue(p.num_layers);
    m_wall_count->SetValue(p.wall_count);
    m_wall_side_length->SetValue(p.wall_side_length);
    m_pattern_spacing->SetValue(p.pattern_spacing);
    m_corner_angle->SetValue(p.corner_angle);
    m_print_dir->SetValue(p.print_dir);
    m_line_ratio->SetValue(p.line_ratio);
    m_anchor->SetSelection(int(p.anchor));
    m_anchor_perimeters->SetValue(p.anchor_perimeters);
    m_anchor_line_ratio->SetValue(p.anchor_line_ratio);
    m_number_tab->SetValue(p.number_tab);
    m_no_leading_zero->SetValue(p.no_leading_zero);
    m_show_on_display->SetValue(p.show_on_display);
    m_first_layer_speed->SetValue(p.first_layer_speed);
    m_perimeter_speed->SetValue(p.perimeter_speed);
    m_acceleration->SetValue(p.acceleration);
    m_updating_controls = false;
}

void PressureAdvanceDialog::read_controls()
{
    PressureAdvancePatternParams &p = m_params;
    p.pa_start          = m_pa_start->GetValue();
    p.pa_end            = m_pa_end->GetValue();
    p.pa_step           = m_pa_step->GetValue();
    p.num_layers        = m_num_layers->GetValue();
    p.wall_count        = m_wall_count->GetValue();
    p.wall_side_length  = m_wall_side_length->GetValue();
    p.pattern_spacing   = m_pattern_spacing->GetValue();
    p.corner_angle      = m_corner_angle->GetValue();
    p.print_dir         = m_print_dir->GetValue();
    p.line_ratio        = m_line_ratio->GetValue();
    p.anchor            = PressureAdvancePatternParams::Anchor(std::clamp(m_anchor->GetSelection(), 0, 2));
    p.anchor_perimeters = m_anchor_perimeters->GetValue();
    p.anchor_line_ratio = m_anchor_line_ratio->GetValue();
    p.number_tab        = m_number_tab->GetValue();
    p.no_leading_zero   = m_no_leading_zero->GetValue();
    p.show_on_display   = m_show_on_display->GetValue();
    p.first_layer_speed = m_first_layer_speed->GetValue();
    p.perimeter_speed   = m_perimeter_speed->GetValue();
    p.acceleration      = m_acceleration->GetValue();
}

void PressureAdvanceDialog::update_status()
{
    this->read_controls();

    const bool anchor = m_params.anchor != PressureAdvancePatternParams::Anchor::None;
    m_anchor_perimeters->Enable(anchor);
    m_anchor_line_ratio->Enable(anchor);
    m_no_leading_zero->Enable(m_params.number_tab);

    // Show which extruder type the range corresponds to.
    m_updating_controls = true;
    m_extruder_type->SetSelection(
        is_pressure_advance_range(m_params, m_flavor, PressureAdvanceExtruderType::DirectDrive) ? itemDirectDrive :
        is_pressure_advance_range(m_params, m_flavor, PressureAdvanceExtruderType::Bowden)      ? itemBowden : itemCustom);
    m_updating_controls = false;

    bool ok = false;
    try {
        const Vec2d size = pressure_advance_pattern_size(wxGetApp().preset_bundle->full_config(), m_params);
        m_status->SetLabel(format_wxstr(_L("%1% chevrons, print size %2% x %3% mm."),
            m_params.num_patterns(), float_to_string_decimal_point(std::round(size.x() * 10.) / 10.), float_to_string_decimal_point(std::round(size.y() * 10.) / 10.)));
        m_status->SetForegroundColour(wxGetApp().get_label_clr_default());
        ok = true;
    } catch (const std::exception &ex) {
        m_status->SetLabel(from_u8(ex.what()));
        m_status->SetForegroundColour(wxGetApp().get_label_clr_modified());
    }
    m_status->Wrap(60 * em_unit());
    m_btn_generate->Enable(ok);

    if (GetSizer() != nullptr)
        this->Layout();
}

void PressureAdvanceDialog::on_extruder_type_changed()
{
    if (m_updating_controls)
        return;
    const int selection = m_extruder_type->GetSelection();
    if (selection == itemCustom)
        return;
    this->read_controls();
    set_pressure_advance_range(m_params, m_flavor, selection == itemBowden ? PressureAdvanceExtruderType::Bowden : PressureAdvanceExtruderType::DirectDrive);
    this->write_controls();
    this->update_status();
}

void PressureAdvanceDialog::reset_to_defaults()
{
    m_params = PressureAdvancePatternParams();
    set_pressure_advance_range(m_params, m_flavor, PressureAdvanceExtruderType::DirectDrive);
    this->write_controls();
    this->update_status();
}

void PressureAdvanceDialog::generate()
{
    this->read_controls();
    try {
        m_gcode = generate_pressure_advance_pattern(wxGetApp().preset_bundle->full_config(), m_params);
    } catch (const std::exception &ex) {
        show_error(this, ex.what(), true);
        return;
    }
    this->save_params();
    EndModal(wxID_OK);
}

std::string PressureAdvanceDialog::filename() const
{
    std::string name = "pressure_advance_" + wxGetApp().preset_bundle->filaments.get_edited_preset().name;
    for (char &c : name)
        if (std::string_view("<>:\"/\\|?* ").find(c) != std::string_view::npos)
            c = '_';
    return fold_utf8_to_ascii(name) + ".gcode";
}

void PressureAdvanceDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    msw_buttons_rescale(this, em_unit(), { wxID_OK, wxID_CANCEL });
    this->Layout();
    this->Fit();
    this->Refresh();
}

} // namespace GUI
} // namespace Slic3r
