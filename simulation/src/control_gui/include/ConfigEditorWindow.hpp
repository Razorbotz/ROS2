#pragma once

#include <gtkmm.h>
#include <map>
#include <string>
#include "InfoFrame.hpp"
#include "BinaryMessage.hpp"
#include "ConfigDefinitions.hpp"

// Forward declaration
class Speedometer;

// Helper struct for managing TreeView columns
class ListColumns : public Gtk::TreeModel::ColumnRecord {
public:
    ListColumns() { add(col_active); add(col_text); add(col_key); }
    Gtk::TreeModelColumn<bool> col_active;
    Gtk::TreeModelColumn<Glib::ustring> col_text;
    Gtk::TreeModelColumn<Glib::ustring> col_key;
};

// The main window class
class ConfigEditorWindow : public Gtk::Window {
public:
    ConfigEditorWindow(const std::string& config_file);
    virtual ~ConfigEditorWindow();

protected:
    // Struct to hold all data for a subsystem editor
    struct SubsystemEditor {
        std::string label;
        std::string prefix;
        InfoFrame* preview_frame = nullptr;
        InfoFrame* options_frame = nullptr;
        std::vector<std::string> local_keys;
    };

    // Signal Handlers
    void on_save_button_clicked();
    void on_reset_button_clicked();
    void on_list_item_toggled(const std::string& prefix, const Glib::ustring& path);
    void on_list_drag_end(const std::string& prefix);
    void on_speedometer_toggle_changed(const std::string& key);
    bool on_delete_event(GdkEventAny* event);

    // Helper Methods
    void setup_ui();
    void setup_general_settings(Gtk::Grid* grid);
    void setup_subsystem_editors(Gtk::Box* parent_box);
    void setup_action_buttons(Gtk::Box* parent_box);
    void load_config(const std::string& config_file);
    void save_config();
    void sync_ui_lists_with_state();
    void populate_binary_message(const std::string& prefix, BinaryMessage& message);
    void update_preview_frame(const std::string& prefix);
    Gtk::Widget* create_reorderable_list(const std::string& prefix);
    
private:
    // Widgets
    Gtk::Entry* m_file_entry;
    Gtk::ColorButton* m_light_color_button;
    Gtk::ColorButton* m_dark_color_button;
    Speedometer* m_test_speedometer;
    std::map<std::string, Gtk::CheckButton*> m_speedometer_buttons;

    // Data
    ListColumns m_columns;
    std::map<std::string, SubsystemEditor> m_subsystems;
    std::map<std::string, Glib::RefPtr<Gtk::ListStore>> m_list_stores;
    std::map<std::string, bool> m_item_visibility;
};