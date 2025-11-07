#include "Speedometer.hpp"

Speedometer::Speedometer(const std::string& label)
    : label_(label),
      speed_(0.0),
      reverse_(false),
      min_speed_(0.0),
      max_speed_(100.0),
      num_major_divisions_(10),
      num_minor_ticks_per_segment_(4),
      display_speed_(true),
      numbers_inside_(true),
      numbers_on_ticks_(true),
      angle_for_zero_(135.0),
      angle_for_sweep_(270.0),
      low_warning_(false),
      low_warning_thresh_(0.2),
      high_warning_(false),
      high_warning_thresh_(0.2),
      use_text_label_(false),
      text_label_("Label")
{
    set_size_request(250, 250);
}

void Speedometer::set_speed(double speed) {
    if (speed < min_speed_) {
        set_reverse(true);
        speed_ = std::clamp(-speed, min_speed_, max_speed_);
    } else {
        speed_ = std::clamp(speed, min_speed_, max_speed_);
        set_reverse(false);
    }
    queue_draw();
}

void Speedometer::set_reverse(bool reverse) {
    reverse_ = reverse;
    queue_draw();
}

void Speedometer::set_min_speed(double speed) {
    min_speed_ = speed;
    speed_ = std::clamp(speed_, min_speed_, max_speed_);
    queue_draw();
}

void Speedometer::set_max_speed(double speed) {
    if (speed < min_speed_) {
        max_speed_ = min_speed_;
    } else {
        max_speed_ = speed;
    }
    speed_ = std::clamp(speed_, min_speed_, max_speed_);
    queue_draw();
}

void Speedometer::set_num_major_divisions(int divisions) {
    if (divisions > 0) {
        num_major_divisions_ = divisions;
        queue_draw();
    }
}

void Speedometer::set_num_minor_ticks_per_segment(int minor_ticks) {
    if (minor_ticks >= 0) {
        num_minor_ticks_per_segment_ = minor_ticks;
        queue_draw();
    }
}

void Speedometer::set_display_speed(bool display_speed) {
    display_speed_ = display_speed;
}

void Speedometer::set_numbers_inside(bool numbers_inside) {
    numbers_inside_ = numbers_inside;
}

void Speedometer::set_numbers_on_ticks(bool numbers_on_ticks) {
    numbers_on_ticks_ = numbers_on_ticks;
}

void Speedometer::set_angle_for_zero(double angle_for_zero) {
    angle_for_zero_ = angle_for_zero;
}

void Speedometer::set_angle_for_sweep(double angle_for_sweep) {
    angle_for_sweep_ = angle_for_sweep;
}

void Speedometer::set_low_warning(bool warning) {
    low_warning_ = warning;
}

void Speedometer::set_low_warning_thresh(double thresh) {
    low_warning_thresh_ = thresh;
}

void Speedometer::set_high_warning(bool warning) {
    high_warning_ = warning;
}

void Speedometer::set_high_warning_thresh(double thresh) {
    high_warning_thresh_ = thresh;
}

void Speedometer::set_use_text_label(bool text_label) {
    use_text_label_ = text_label;
}

void Speedometer::set_text_label(std::string label) {
    text_label_ = label;
}


bool Speedometer::on_draw(const Cairo::RefPtr<Cairo::Context>& cr) {
    Gtk::Allocation alloc = get_allocation();
    const int w = alloc.get_width() - 15;
    const int h = alloc.get_height() - 15;

    const double smallest_dim = std::min(w, h);
    const double radius = smallest_dim / 2.5;
    const double cx = w / 2.0;
    const double cy = h / 2.0;

    const double angle_for_zero_value_rad = angle_for_zero_ * M_PI / 180.0;
    const double total_sweep_angle_rad = angle_for_sweep_ * M_PI / 180.0;

    // Colors
    Gdk::RGBA color_dial_bg;
    color_dial_bg.set_rgba(0.1, 0.1, 0.1, 1.0); // Dark grey
    Gdk::RGBA color_bezel;
    color_bezel.set_rgba(0.2, 0.2, 0.2, 1.0);
    Gdk::RGBA color_tick_mark;
    color_tick_mark.set_rgba(0.9, 0.9, 0.9, 1.0); // Light grey/white
    Gdk::RGBA color_text;
    color_text.set_rgba(0.9, 0.9, 0.9, 1.0);
    Gdk::RGBA color_needle;
    color_needle.set_rgba(1.0, 0.2, 0.2, 1.0); // Reddish
    Gdk::RGBA color_needle_pivot;
    color_needle_pivot.set_rgba(0.7, 0.7, 0.7, 1.0);
    Gdk::RGBA color_speed_text_normal;
    color_speed_text_normal.set_rgba(0.8, 0.8, 1.0, 1.0); // Light blueish
    Gdk::RGBA color_speed_text_reverse;
    color_speed_text_reverse.set_rgba(1.0, 0.8, 0.8, 1.0); // Light reddish
    Gdk::RGBA label_text;
    label_text.set_rgba(0.1, 0.1, 0.1, 1.0);


    // 1. Bezel
    cr->set_source_rgba(color_bezel.get_red(), color_bezel.get_green(), color_bezel.get_blue(), color_bezel.get_alpha());
    cr->arc(cx, cy, radius + 10, 0, 2 * M_PI);
    cr->fill();

    // 2. Dial background
    cr->set_source_rgba(color_dial_bg.get_red(), color_dial_bg.get_green(), color_dial_bg.get_blue(), color_dial_bg.get_alpha());
    cr->arc(cx, cy, radius + 5, 0, 2 * M_PI);
    cr->fill_preserve();
    cr->set_source_rgba(0.3, 0.3, 0.3, 1.0); // Outline for the dial face
    cr->set_line_width(1.0);
    cr->stroke();

    // 3. Ticks and Labels
    const double major_tick_len = 10.0;
    const double minor_tick_len = 5.0;
    const double text_radius_offset = 20.0; // How far from ticks to place text

    // Draw red arc for warning zone
    auto draw_warning_arc = [&](double danger_speed_start, double danger_speed_end) {
        if (danger_speed_start < danger_speed_end && max_speed_ > min_speed_) {
            double ratio_start = (danger_speed_start - min_speed_) / (max_speed_ - min_speed_);
            double ratio_end = (danger_speed_end - min_speed_) / (max_speed_ - min_speed_);

            double angle_start = angle_for_zero_value_rad + ratio_start * total_sweep_angle_rad;
            double angle_end = angle_for_zero_value_rad + ratio_end * total_sweep_angle_rad;

            cr->set_line_width(major_tick_len * 1.5);
            cr->set_source_rgb(1.0, 0.0, 0.0);
            cr->arc(cx, cy, radius - major_tick_len / 2.0, angle_start, angle_end);
            cr->stroke();
        }
    };

    if (low_warning_) {
        draw_warning_arc(min_speed_, min_speed_ + low_warning_thresh_ * (max_speed_ - min_speed_));
    }
    if (high_warning_) {
        draw_warning_arc(max_speed_ - high_warning_thresh_ * (max_speed_ - min_speed_), max_speed_);
    }


    cr->set_source_rgba(color_tick_mark.get_red(), color_tick_mark.get_green(), color_tick_mark.get_blue(), color_tick_mark.get_alpha());
    for (int i = 0; i <= num_major_divisions_; ++i) {
        double tick_ratio = static_cast<double>(i) / num_major_divisions_;
        double angle = angle_for_zero_value_rad + tick_ratio * total_sweep_angle_rad;

        // Major tick
        double x1 = cx + radius * cos(angle);
        double y1 = cy + radius * sin(angle);
        double x2 = cx + (radius - major_tick_len) * cos(angle);
        double y2 = cy + (radius - major_tick_len) * sin(angle);

        cr->set_line_width(2.0);
        cr->move_to(x1, y1);
        cr->line_to(x2, y2);
        cr->stroke();

        // Number label for major tick
        // Ensure max_speed_ is not zero to avoid issues, though labels can be 0
        double value = tick_ratio * (max_speed_ - min_speed_) + min_speed_;
        std::string tick_text = std::to_string(static_cast<int>(round(value)));

        Cairo::TextExtents extents;
        cr->set_font_size(std::max(10.0, smallest_dim / 20.0)); // Responsive font size
        cr->get_text_extents(tick_text, extents);

        // Adjust text position to be centered and outside ticks
        double label_distance = numbers_inside_
            ? (radius - major_tick_len - text_radius_offset)
            : (radius + text_radius_offset);

        double tx = cx + label_distance * cos(angle) - (extents.width / 2.0 + extents.x_bearing);
        double ty = cy + label_distance * sin(angle) - (extents.height / 2.0 + extents.y_bearing);
        
        if (numbers_inside_) {
            cr->set_source_rgba(color_text.get_red(), color_text.get_green(), color_text.get_blue(), color_text.get_alpha());
        }
        else {
            cr->set_source_rgb(0.0, 0.0, 0.0);
        }
        cr->move_to(tx, ty);
        if(numbers_on_ticks_)
            cr->show_text(tick_text);

        // Reset color after drawing text
        cr->set_source_rgba(color_text.get_red(), color_text.get_green(), color_text.get_blue(), color_text.get_alpha());

        // Minor ticks (except after the last major tick)
        if (i < num_major_divisions_) {
            for (int j = 1; j <= num_minor_ticks_per_segment_; ++j) {
                double minor_tick_ratio = tick_ratio + (static_cast<double>(j) / num_major_divisions_ / (num_minor_ticks_per_segment_ + 1));
                // Ensure minor ticks don't overshoot total_sweep_angle_rad
                if (minor_tick_ratio * total_sweep_angle_rad > total_sweep_angle_rad + 1e-6) continue; 

                double minor_angle = angle_for_zero_value_rad + minor_tick_ratio * total_sweep_angle_rad;
                double mx1 = cx + radius * cos(minor_angle);
                double my1 = cy + radius * sin(minor_angle);
                double mx2 = cx + (radius - minor_tick_len) * cos(minor_angle);
                double my2 = cy + (radius - minor_tick_len) * sin(minor_angle);

                cr->set_line_width(1.0);
                cr->move_to(mx1, my1);
                cr->line_to(mx2, my2);
                cr->stroke();
            }
        }
    }

    // 4. Needle
    double current_speed_ratio = 0.0;
    if (max_speed_ > min_speed_) { // Avoid division by zero or undefined behavior
         current_speed_ratio = (speed_ - min_speed_) / (max_speed_ - min_speed_);
    }
    else if (max_speed_ == min_speed_ && speed_ == min_speed_){
         current_speed_ratio = 0.0; // Or 0.5 if middle, but for 0-max this is fine
    }


    double needle_angle = angle_for_zero_value_rad + current_speed_ratio * total_sweep_angle_rad;
    cr->set_source_rgba(color_needle.get_red(), color_needle.get_green(), color_needle.get_blue(), color_needle.get_alpha());
    cr->set_line_width(std::max(2.0, smallest_dim / 80.0)); // Responsive needle width
    cr->move_to(cx, cy);
    cr->line_to(cx + (radius - major_tick_len/2) * cos(needle_angle), cy + (radius - major_tick_len/2) * sin(needle_angle));
    cr->stroke();

    // 5. Needle Pivot
    cr->set_source_rgba(color_needle_pivot.get_red(), color_needle_pivot.get_green(), color_needle_pivot.get_blue(), color_needle_pivot.get_alpha());
    cr->arc(cx, cy, std::max(4.0, smallest_dim / 40.0), 0, 2 * M_PI);
    cr->fill();
    cr->set_source_rgba(0.1,0.1,0.1,1); // Pivot outline
    cr->set_line_width(0.5);
    cr->arc(cx, cy, std::max(4.0, smallest_dim / 40.0), 0, 2 * M_PI);
    cr->stroke();


    // 6. Speed Text Display
    if(!use_text_label_){
        std::ostringstream speed_stream;
        speed_stream << std::fixed << std::setprecision(1) << speed_;
        std::string speed_str = speed_stream.str();
        if (reverse_) {
            speed_str += " R";
            cr->set_source_rgba(color_speed_text_reverse.get_red(), color_speed_text_reverse.get_green(), color_speed_text_reverse.get_blue(), color_speed_text_reverse.get_alpha());
        } else {
            cr->set_source_rgba(color_speed_text_normal.get_red(), color_speed_text_normal.get_green(), color_speed_text_normal.get_blue(), color_speed_text_normal.get_alpha());
        }

        cr->select_font_face("Sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_BOLD);
        cr->set_font_size(std::max(14.0, smallest_dim / 12.0));

        Cairo::TextExtents speed_extents;
        cr->get_text_extents(speed_str, speed_extents);
        cr->move_to(cx - (speed_extents.width / 2.0 + speed_extents.x_bearing), cy + radius * 0.5); // Position below center
        if(display_speed_)
            cr->show_text(speed_str);
    }
    else{
        cr->set_source_rgba(color_speed_text_normal.get_red(), color_speed_text_normal.get_green(), color_speed_text_normal.get_blue(), color_speed_text_normal.get_alpha());

        cr->select_font_face("Sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_BOLD);
        cr->set_font_size(std::max(14.0, smallest_dim / 12.0));

        Cairo::TextExtents speed_extents;
        cr->get_text_extents(text_label_, speed_extents);
        cr->move_to(cx - (speed_extents.width / 2.0 + speed_extents.x_bearing), cy + radius * 0.5); // Position below center
        if(display_speed_)
            cr->show_text(text_label_);
    }

    // 7. Main Label (e.g., "Left Speed")
    cr->set_source_rgba(label_text.get_red(), label_text.get_green(), label_text.get_blue(), label_text.get_alpha());
    cr->select_font_face("Sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_NORMAL);
    cr->set_font_size(std::max(16.0, smallest_dim / 15.0));
    Cairo::TextExtents label_extents;
    cr->get_text_extents(label_, label_extents);
    cr->move_to(cx - (label_extents.width / 2.0 + label_extents.x_bearing), cy + radius + 15 + label_extents.height); // Position below gauge
    cr->show_text(label_);

    return true;
}