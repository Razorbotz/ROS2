#pragma once

#include <gtkmm.h>
#include <cairomm/context.h>
#include <string>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>

class Speedometer : public Gtk::DrawingArea {
public:
    Speedometer(const std::string& label);

    void set_speed(double speed);
    void set_reverse(bool reverse);
    void set_min_speed(double speed);
    void set_max_speed(double speed);
    void set_num_major_divisions(int divisions);
    void set_num_minor_ticks_per_segment(int minor_ticks);
    void set_display_speed(bool display_speed);
    void set_numbers_inside(bool numbers_inside);
    void set_numbers_on_ticks(bool numbers_on_ticks);
    void set_angle_for_zero(double angle_for_zero);
    void set_angle_for_sweep(double angle_for_sweep);
    void set_low_warning(bool warning);
    void set_low_warning_thresh(double thresh);
    void set_high_warning(bool warning);
    void set_high_warning_thresh(double thresh);
    void set_use_text_label(bool text_label);
    void set_text_label(std::string label);

protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;

private:
    std::string label_;
    double speed_;
    bool reverse_;
    double min_speed_;
    double max_speed_;
    int num_major_divisions_;
    int num_minor_ticks_per_segment_;
    bool display_speed_;
    bool numbers_inside_;
    bool numbers_on_ticks_;
    double angle_for_zero_;
    double angle_for_sweep_;
    bool low_warning_;
    double low_warning_thresh_;
    bool high_warning_;
    double high_warning_thresh_;
    bool use_text_label_;
    std::string text_label_;
};