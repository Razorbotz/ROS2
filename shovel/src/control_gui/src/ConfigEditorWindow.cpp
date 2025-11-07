#include "ConfigEditorWindow.hpp"
#include "Speedometer.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

bool allowConfig = true;

// --- Constructor ---
ConfigEditorWindow::ConfigEditorWindow(const std::string& config_file) {
    set_title("Configuration Editor");
    set_default_size(1000, 700);
    initialize_maps();

    m_subsystems["TALON"] = {"Talon", "TALON", nullptr, nullptr, get_talon_keys()};
    m_subsystems["FALCON"] = {"Falcon", "FALCON", nullptr, nullptr, get_falcon_keys()};
    m_subsystems["LINEAR"] = {"Linear", "LINEAR", nullptr, nullptr, get_linear_keys()};
    m_subsystems["AUTONOMY"] = {"Autonomy", "AUTONOMY", nullptr, nullptr, get_autonomy_keys()};
    m_subsystems["ZED"] = {"Zed", "ZED", nullptr, nullptr, get_zed_keys()};
    m_subsystems["COMMUNICATION"] = {"Communication", "COMMUNICATION", nullptr, nullptr, get_communication_keys()};
    m_subsystems["POWER"] = {"Power", "POWER", nullptr, nullptr, get_power_keys()};
    m_subsystems["POWER2"] = {"Power2", "POWER2", nullptr, nullptr, get_power2_keys()};
    m_subsystems["DRIVETRAIN"] = {"Drivetrain", "DRIVETRAIN", nullptr, nullptr, get_drivetrain_keys()};

    setup_ui();
    load_config(config_file);

    for (const auto& pair : m_subsystems) {
        const auto& prefix = pair.first;
        const auto& editor = pair.second;
        for (const auto& key : editor.local_keys) {
            std::string full_key = "SHOW_" + prefix + "_" + key;
            // If a key wasn't in the config file, add it to our map now with a default of 'true'.
            if (m_item_visibility.find(full_key) == m_item_visibility.end()) {
                m_item_visibility[full_key] = true;
            }
        }
    }

    sync_ui_lists_with_state();

    signal_delete_event().connect(sigc::mem_fun(*this, &ConfigEditorWindow::on_delete_event)); 
    show_all();
}

ConfigEditorWindow::~ConfigEditorWindow() {}


// --- UI Setup ---
void ConfigEditorWindow::setup_ui() {
    auto scrolled_window = Gtk::make_managed<Gtk::ScrolledWindow>();
    scrolled_window->set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
    add(*scrolled_window);

    auto main_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    main_box->set_border_width(10);
    scrolled_window->add(*main_box);

    auto grid = Gtk::make_managed<Gtk::Grid>();
    grid->set_column_spacing(10);
    grid->set_row_spacing(5);
    main_box->pack_start(*grid, Gtk::PACK_SHRINK);

    auto outer_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    main_box->pack_start(*outer_box);
    
    setup_general_settings(grid);
    setup_subsystem_editors(outer_box);
    setup_action_buttons(main_box);
}

void ConfigEditorWindow::setup_general_settings(Gtk::Grid* grid) {
    m_file_entry = Gtk::make_managed<Gtk::Entry>();
    grid->attach(*Gtk::make_managed<Gtk::Label>("Config File:"), 0, 0, 1, 1);
    grid->attach(*m_file_entry, 1, 0, 1, 1);

    m_light_color_button = Gtk::make_managed<Gtk::ColorButton>();
    grid->attach(*Gtk::make_managed<Gtk::Label>("Light BG Color:"), 0, 1, 1, 1);
    grid->attach(*m_light_color_button, 1, 1, 1, 1);

    m_dark_color_button = Gtk::make_managed<Gtk::ColorButton>();
    grid->attach(*Gtk::make_managed<Gtk::Label>("Dark BG Color:"), 0, 2, 1, 1);
    grid->attach(*m_dark_color_button, 1, 2, 1, 1);
}

void ConfigEditorWindow::setup_subsystem_editors(Gtk::Box* parent_box) {
    auto speed_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);
    parent_box->pack_start(*speed_box, Gtk::PACK_SHRINK);

    m_test_speedometer = Gtk::make_managed<Speedometer>("Test");
    m_test_speedometer->set_size_request(200, 200);
    speed_box->pack_start(*m_test_speedometer, Gtk::PACK_SHRINK);

    auto speed_options_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 5);
    speed_box->pack_start(*speed_options_box, Gtk::PACK_SHRINK);

    for (const auto& key : {"DISPLAY_SPEED", "NUMBERS_INSIDE", "NUMBER_TICKS"}) {
        m_speedometer_buttons[key] = Gtk::make_managed<Gtk::CheckButton>(key);
        speed_options_box->pack_start(*m_speedometer_buttons[key], Gtk::PACK_SHRINK);
        m_speedometer_buttons[key]->signal_toggled().connect(sigc::bind(sigc::mem_fun(*this, &ConfigEditorWindow::on_speedometer_toggle_changed), key));
    }

    auto sensors_box = Gtk::make_managed<Gtk::FlowBox>();
    sensors_box->set_selection_mode(Gtk::SELECTION_NONE);
    parent_box->pack_start(*sensors_box);

    for (auto& pair : m_subsystems) {
        auto& editor = pair.second;
        auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
        sensors_box->add(*box);

        editor.preview_frame = Gtk::make_managed<InfoFrame>(editor.label);
        box->pack_start(*editor.preview_frame);
        
        editor.options_frame = Gtk::make_managed<InfoFrame>(editor.label + " Options");
        auto list_widget = create_reorderable_list(editor.prefix);
        editor.options_frame->addWidget(*list_widget);
        box->pack_start(*editor.options_frame);
    }
}

void ConfigEditorWindow::setup_action_buttons(Gtk::Box* parent_box) {
    auto button_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);
    button_box->set_halign(Gtk::ALIGN_CENTER);
    parent_box->pack_start(*button_box, Gtk::PACK_SHRINK);

    auto save_button = Gtk::make_managed<Gtk::Button>("Save");
    save_button->signal_clicked().connect(sigc::mem_fun(*this, &ConfigEditorWindow::on_save_button_clicked));
    button_box->pack_start(*save_button);

    auto reset_button = Gtk::make_managed<Gtk::Button>("Reset");
    reset_button->signal_clicked().connect(sigc::mem_fun(*this, &ConfigEditorWindow::on_reset_button_clicked));
    button_box->pack_start(*reset_button);
}

// --- Core Logic ---

void ConfigEditorWindow::load_config(const std::string& config_file) {
    m_file_entry->set_text(config_file);
    m_item_visibility.clear();
    std::ifstream file("../resources/" + config_file);
    if (!file.is_open()) return;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string key, value;
        if (!(std::getline(ss, key, '=') && std::getline(ss, value))) continue;

        if (key == "LIGHT_BACKGROUND") m_light_color_button->set_rgba(Gdk::RGBA(value));
        else if (key == "DARK_BACKGROUND") m_dark_color_button->set_rgba(Gdk::RGBA(value));
        else if (m_speedometer_buttons.count(key)) m_speedometer_buttons[key]->set_active(value == "true");
        else if (key.rfind("ORDER_", 0) == 0) {
            std::string prefix = key.substr(6); // Get prefix like "TALON" from "ORDER_TALON"
            if (m_subsystems.count(prefix)) {
                std::vector<std::string> ordered_keys;
                std::stringstream key_ss(value);
                std::string item;
                while(std::getline(key_ss, item, ',')) {
                    ordered_keys.push_back(item);
                }
                m_subsystems.at(prefix).local_keys = ordered_keys;
            }
        }
        else { 
            m_item_visibility[key] = (value == "true");
        }
    }
}

void ConfigEditorWindow::sync_ui_lists_with_state() {
    for (auto& subsystem_pair : m_subsystems) {
        const std::string& prefix = subsystem_pair.first;
        auto& editor = subsystem_pair.second;
        auto model = m_list_stores[prefix];

        model->clear(); 

        for (const auto& key : editor.local_keys) {
            auto row = *(model->append());
            row[m_columns.col_text] = key;
            row[m_columns.col_key] = prefix + "_" + key;

            std::string full_key = "SHOW_" + prefix + "_" + key;
            bool is_active = m_item_visibility.count(full_key) ? m_item_visibility[full_key] : true;
            row[m_columns.col_active] = is_active;
        }
    }

    for (const auto& pair : m_subsystems) {
        update_preview_frame(pair.first);
    }
}

Gtk::Widget* ConfigEditorWindow::create_reorderable_list(const std::string& prefix) {
    auto scrolled_window = Gtk::make_managed<Gtk::ScrolledWindow>();
    scrolled_window->set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
    scrolled_window->set_min_content_height(300);

    auto tree_view = Gtk::make_managed<Gtk::TreeView>();
    scrolled_window->add(*tree_view);

    m_list_stores[prefix] = Gtk::ListStore::create(m_columns);
    tree_view->set_model(m_list_stores[prefix]);
    tree_view->set_reorderable(true);

    auto cell_toggle = Gtk::make_managed<Gtk::CellRendererToggle>();
    tree_view->append_column("Active", *cell_toggle);
    tree_view->get_column(0)->add_attribute(cell_toggle->property_active(), m_columns.col_active);
    cell_toggle->signal_toggled().connect([this, prefix](const Glib::ustring& path) {
        this->on_list_item_toggled(prefix, path);
    });

    tree_view->append_column("Item", m_columns.col_text);
    tree_view->signal_drag_end().connect([this, prefix](const Glib::RefPtr<Gdk::DragContext>&) {
        on_list_drag_end(prefix);
    });
    
    return scrolled_window;
}

void ConfigEditorWindow::update_preview_frame(const std::string& prefix) {
    auto subsystem_it = m_subsystems.find(prefix);
    if (subsystem_it == m_subsystems.end()) return;
    auto& editor = subsystem_it->second;

    InfoFrame* frame = editor.preview_frame;
    if (!frame) return;

    frame->removeAllItems();

    BinaryMessage message(editor.label);
    populate_binary_message(prefix, message);

    std::map<std::string, Element> element_map;
    for (const auto& el : message.getObject().elementList) {
        element_map[el.label] = el;
    }
    
    for (const auto& key : editor.local_keys) {
        std::string full_key = "SHOW_" + prefix + "_" + key;
        std::cout << full_key << ": ";
        std::cout << m_item_visibility.at(full_key) << std::endl;
        if (m_item_visibility.count(full_key) ? m_item_visibility.at(full_key) : true) {
            auto el_it = element_map.find(key);
            if (el_it != element_map.end()) {
                addElementToInfoFrame(frame, el_it->second);
            }
        }
    }
    frame->show_all();
}

void ConfigEditorWindow::populate_binary_message(const std::string& prefix, BinaryMessage& message) {
    auto defs_it = get_element_definitions().find(prefix);
    if (defs_it == get_element_definitions().end()) return;

    for (const auto& def : defs_it->second) {
        std::cout << def.name << std::endl;
        switch(def.type) {
            case ElementType::UInt8:   message.addElementUInt8(def.name, 0); break;
            case ElementType::UInt16:  message.addElementUInt16(def.name, 0); break;
            case ElementType::Int8:    message.addElementInt8(def.name, 0); break;
            case ElementType::Int32:   message.addElementInt32(def.name, 0); break;
            case ElementType::Float32: message.addElementFloat32(def.name, 0.0f); break;
            case ElementType::Boolean: message.addElementBoolean(def.name, false); break;
            case ElementType::String:  message.addElementString(def.name, ""); break;
        }
    }
}

// --- Signal Handlers ---

void ConfigEditorWindow::on_save_button_clicked() {
    save_config();
    hide();
}

void ConfigEditorWindow::save_config() {
    std::ofstream outfile("../resources/" + m_file_entry->get_text());

    for (const auto& pair : m_subsystems) {
        const auto& editor = pair.second;
        
        // ** NEW: Save the reordered keys to the file **
        outfile << "ORDER_" << editor.prefix << "=";
        for (size_t i = 0; i < editor.local_keys.size(); ++i) {
            outfile << editor.local_keys[i] << (i == editor.local_keys.size() - 1 ? "" : ",");
        }
        outfile << "\n";

        // This part updates the live, in-memory order for the main app
        if (editor.prefix == "TALON") get_talon_keys() = editor.local_keys;
        else if (editor.prefix == "FALCON") get_falcon_keys() = editor.local_keys;
        else if (editor.prefix == "LINEAR") get_linear_keys() = editor.local_keys;
        else if (editor.prefix == "AUTONOMY") get_autonomy_keys() = editor.local_keys;
        else if (editor.prefix == "ZED") get_zed_keys() = editor.local_keys;
        else if (editor.prefix == "COMMUNICATION") get_communication_keys() = editor.local_keys;
        else if (editor.prefix == "POWER") get_power_keys() = editor.local_keys;
        else if (editor.prefix == "POWER2") get_power2_keys() = editor.local_keys;
        else if (editor.prefix == "DRIVETRAIN") get_drivetrain_keys() = editor.local_keys;
    }

    for (const auto& pair : m_item_visibility) {
        outfile << pair.first << "=" << (pair.second ? "true" : "false") << "\n";
    }

    outfile << "LIGHT_BACKGROUND=" << m_light_color_button->get_rgba().to_string() << "\n";
    outfile << "DARK_BACKGROUND=" << m_dark_color_button->get_rgba().to_string() << "\n";
    for(const auto& pair : m_speedometer_buttons) {
        outfile << pair.first << "=" << (pair.second->get_active() ? "true" : "false") << "\n";
    }

    for (const auto& pair : m_subsystems) {
        auto& values_map = getMap(pair.second.label);
        values_map.clear();
        for(const auto& key : pair.second.local_keys) {
            std::string full_key = "SHOW_" + pair.second.prefix + "_" + key;
            values_map[key] = m_item_visibility.count(full_key) ? m_item_visibility.at(full_key) : true;
        }
    }
    
    lightBackgroundColor = m_light_color_button->get_rgba().to_string();
    darkBackgroundColor = m_dark_color_button->get_rgba().to_string();
    displaySpeed = m_speedometer_buttons["DISPLAY_SPEED"]->get_active();
    numbersInside = m_speedometer_buttons["NUMBERS_INSIDE"]->get_active();
    numberTicks = m_speedometer_buttons["NUMBER_TICKS"]->get_active();

    updateGUI();
}

void ConfigEditorWindow::on_reset_button_clicked() {
    m_item_visibility.clear(); 

    for (auto& pair : m_subsystems) {
        auto& editor = pair.second;
        const std::string& prefix = editor.prefix;
        
        if (prefix == "TALON") editor.local_keys = get_reset_talon_keys();
        else if (prefix == "FALCON") editor.local_keys = get_reset_falcon_keys();
        else if (prefix == "LINEAR") editor.local_keys = get_reset_linear_keys();
        else if (prefix == "AUTONOMY") editor.local_keys = get_reset_autonomy_keys();
        else if (prefix == "ZED") editor.local_keys = get_reset_zed_keys();
        else if (prefix == "COMMUNICATION") editor.local_keys = get_reset_communication_keys();
        else if (prefix == "POWER") editor.local_keys = get_reset_power_keys();
        else if (prefix == "POWER2") editor.local_keys = get_reset_power2_keys();
        else if (prefix == "DRIVETRAIN") editor.local_keys = get_reset_drivetrain_keys();
    }
    sync_ui_lists_with_state();
}

void ConfigEditorWindow::on_list_item_toggled(const std::string& prefix, const Glib::ustring& path) {
    auto store = m_list_stores.at(prefix);
    auto iter = store->get_iter(path);
    if (iter) {
        bool active = !(*iter)[m_columns.col_active];
        (*iter)[m_columns.col_active] = active;
        
        std::string item_key = Glib::ustring((*iter)[m_columns.col_text]).raw();
        std::string full_key = "SHOW_" + prefix + "_" + item_key;
        m_item_visibility[full_key] = active;
        
        InfoFrame* frame = m_subsystems.at(prefix).preview_frame;
        if (!frame) return;

        if (active) {
            update_preview_frame(prefix);
        }
        else {
            frame->removeItem(item_key);
        }
    }
}

void ConfigEditorWindow::on_list_drag_end(const std::string& prefix) {
    auto& editor = m_subsystems.at(prefix);
    auto model = m_list_stores.at(prefix);

    editor.local_keys.clear();
    for (const auto& row : model->children()) {
        editor.local_keys.push_back(Glib::ustring(row[m_columns.col_text]).raw());
    }
    update_preview_frame(prefix);
}

void ConfigEditorWindow::on_speedometer_toggle_changed(const std::string& key) {
    if(!m_test_speedometer) return;
    bool active = m_speedometer_buttons.at(key)->get_active();
    if (key == "DISPLAY_SPEED") m_test_speedometer->set_display_speed(active);
    else if (key == "NUMBERS_INSIDE") m_test_speedometer->set_numbers_inside(active);
    else if (key == "NUMBER_TICKS") m_test_speedometer->set_numbers_on_ticks(active);
    m_test_speedometer->queue_draw();
}

bool ConfigEditorWindow::on_delete_event(GdkEventAny* event) {
    return false;
}