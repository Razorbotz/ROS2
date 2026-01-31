#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <string> 
#include <vector>
#include <sys/types.h>
#include <sys/socket.h> 
//#include <cstdlib>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <iostream>
//#include <fstream>
#include <fcntl.h>
#include <thread>
#include <list>
#include <chrono>
#include <cmath>

#include <glibmm/ustring.h>
#include <SDL2/SDL.h>
#include <gtkmm.h>
#include <gdkmm.h>
#include <gtkmm/window.h>
#include <webkit2/webkit2.h>
#include <cairomm/context.h>
#include <pangomm.h>
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <unordered_set>
#include <algorithm>
#include <iomanip>
#include <fstream>

#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <map>
#include <sstream>
#include <curl/curl.h>
#include <variant>
#include <regex>
#include <mutex>
#include <atomic>
#include <zlib.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include "InfoFrame.hpp"
#include "BinaryMessage.hpp"
#include "Speedometer.hpp"
#include "ConfigDefinitions.hpp"
#include "ConfigEditorWindow.hpp"
#include "NetworkHandler.hpp"

/*
TODO: 
Map Issues:
Cosmic map isn't drawing robot in correct location
Robot isn't drawing in correct location, need to offset for camera position

Convert video stream from TCP to UDP

*/

#define ORIN_IP "192.168.1.6"
#define NANO_IP "192.168.1.5"
bool useOrin = true;

std::string package_share_directory;

#define LOW_VOLTAGE 12.0f

float parseFloat(const uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;
    axisYInteger|=uint32_t(array[1])<<16;
    axisYInteger|=uint32_t(array[2])<<8;
    axisYInteger|=uint32_t(array[3])<<0;
    float value=(float)*(static_cast<float*>(static_cast<void*>(&axisYInteger)));

    return value;
}

int parseInt(const uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;
    axisYInteger|=uint32_t(array[1])<<16;
    axisYInteger|=uint32_t(array[2])<<8;
    axisYInteger|=uint32_t(array[3])<<0;
    int value=(int)*(static_cast<int*>(static_cast<void*>(&axisYInteger)));

    return value;
}

 
void insert(float value,uint8_t* array){
    array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
    array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
    array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
    array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}


void insert(int value,uint8_t* array){
    array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
    array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
    array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
    array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}


bool quit(GdkEventAny* event){
    exit(0);
}

Gtk::ListBox* addressListBox;
Gtk::Entry* ipAddressEntry;
Gtk::Label* connectionStatusLabel;
  
Gtk::Button* silentRunButton;
Gtk::Button* connectButton;
Gtk::Button* toggleModeButton;
Gtk::Button* settingsButton;

Gtk::ListBox* videoAddressListBox;
Gtk::Entry* videoIPAddressEntry;
Gtk::Label* videoConnectionStatusLabel;
  
Gtk::Button* videoStreamButton;
Gtk::Button* videoConnectButton;
bool isGray = true;
std::mutex frameMutex;
cv::Mat latestFrame;
std::atomic<bool> newFrameAvailable;
Glib::Dispatcher videoDisconnectDispatcher;
std::atomic<bool> shouldVideoDisconnect = false;
Glib::Dispatcher connection_finished_dispatcher;
Glib::Dispatcher video_connection_finished_dispatcher;
  
Gtk::FlowBox* sensorBox;
Gtk::Box* innerLeftBox;
Gtk::Box* innerRightBox;
Gtk::Box* bottomLowerBox;

Gtk::Window* window;

bool initVals = false;
bool threeMonitors = false;
bool smallLaptop = false;
bool noVideo = false;
bool noArena = false;
std::string mapUsed = "NASA";
bool testInput = false;
bool useAltLayout = false;
bool isController = false;
bool twoJoysticks = false;

ServerUI server_ui;
VideoServerUI video_server_ui;

Gtk::Window* arenaWindow;
Gtk::Window* sensorsWindow;
ConfigEditorWindow* configWindow = nullptr;
Gtk::Window* motorWindow;
int monitor_count = 0;

std::string darkBackgroundColor = "#0b1a21";
std::string lightBackgroundColor = "#f0faf2";
bool isLightMode = true;


double roll_rotation_angle = 0.0;
Glib::RefPtr<Gdk::Pixbuf> roll_pixbuf;
Gtk::Image* roll_image;

double pitch_rotation_angle = 0.0;
Glib::RefPtr<Gdk::Pixbuf> pitch_pixbuf;
Glib::RefPtr<Gdk::Pixbuf> lvl_pixbuf;
Gtk::Image* pitch_image;
Gtk::Image* lvl_image;


double MULTIPLIER_X = 1100.0 / 6.88;
double MULTIPLIER_Y = 800.0 / 5.0;

double ARENA_WIDTH_M = 6.88, ARENA_HEIGHT_M = 5.0;
double ARENA_WIDTH_P = 1100.0, ARENA_HEIGHT_P = 800.0;

double UCF_WIDTH_M = 8.14, UCF_HEIGHT_M = 4.57;
double UCF_WIDTH_P = 1300.0, UCF_HEIGHT_P = 730;

double COSMIC_WIDTH_M = 5.48, COSMIC_HEIGHT_M = 4.87;
double COMSIC_WIDTH_P = 877, COSMIC_HEIGHT_P = 780;

double LAB_WIDTH_M = 5.0, LAB_HEIGHT_M = 4.0;
double LAB_WIDTH_P = 800, LAB_HEIGHT_P = 640;


std::vector<InfoFrame*> infoFrameList;

struct AxisEvent{
    bool isSet=false;
    uint8_t which;
    uint8_t axis;  //0-roll 1-pitch 2-throttle 3-yaw
    int value;
};
std::vector<std::vector<AxisEvent*>*>* axisEventList;


class DrawingArea : public Gtk::DrawingArea {
    public:
        DrawingArea() : top_color_("#D3D3D3"), bottom_color_("#A9A9A9"), ratio_(2.0 / 3.0) {}
        
        void set_height_ratio(double ratio){
            ratio_ = ratio;
            queue_draw();
        }
    
    protected:
        bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override {
            int width = get_allocated_width();
            int height = get_allocated_height();
            cr->set_source_rgb(top_color_.get_red(), top_color_.get_green(), top_color_.get_blue());
            cr->rectangle(0, 0, width, height * ratio_);
            cr->fill();
            
            cr->set_source_rgb(bottom_color_.get_red(), bottom_color_.get_green(), bottom_color_.get_blue());
            cr->rectangle(0, height * ratio_, width, height * (1 - ratio_));
            cr->fill();
            
            return true;
        }
    
    private:
        Gdk::RGBA top_color_;
        Gdk::RGBA bottom_color_;
        double ratio_;
    };

DrawingArea* right_arm;
DrawingArea* left_arm;
DrawingArea* right_bucket;
DrawingArea* left_bucket;
Gtk::Box* armBox;
Gtk::Box* bucketBox;
bool arm_init = false, bucket_init = false, roll_init = false, pitch_init = false, bucketLevel_init = false;

int right_arm_pos = 0, left_arm_pos = 0, right_bucket_pos = 0, left_bucket_pos = 0;

class ImageOverlay : public Gtk::DrawingArea {
    public:
        ImageOverlay() :
            img_x(100), img_y(50), rotation_angle(0.0), dest_x(-1), dest_y(-1) {
                load_images();
            }
    
        bool update_image_position(double x, double y){
            img_x = x;
            img_y = y;
            queue_draw();
            return true;
        }

        bool update_image_rotation(double rotation){
            rotation_angle = ((rotation * M_PI) / 180);
            queue_draw();
            return true;
        }

        bool update_image_x(double x){
            img_x = x;
            queue_draw();
            return true;
        }

        bool update_image_y(double y){
            img_y = y;
            queue_draw();
            return true;
        }

        // Scale factor of map means 1m = 160px, so scale multiplier sets
        // the size of the rock and hole to scale multiplier meters in radius
        void add_rock_image(int x, int y, double scale_multiplier) {
            rock_data.emplace_back(x, y, scale_multiplier);
            queue_draw();
        }

        void add_hole_image(int x, int y, double scale_multiplier) {
            hole_data.emplace_back(x, y, scale_multiplier);
            queue_draw();
        }

        void add_dest_loc(int x, int y){
            dest_x = x;
            dest_y = y;
            queue_draw();
        }

    protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override {
        if (!background || !overlay) return false;

        int height = 0;
        if(mapUsed == "NASA"){
            height = ARENA_HEIGHT_P;
        }
        else if(mapUsed == "UCF"){
            height = UCF_HEIGHT_P;
        }
        else if(mapUsed == "Cosmic"){
            height = COSMIC_HEIGHT_P;
        }
        else if(mapUsed == "Lab"){
            height = LAB_HEIGHT_P;
        }
        else{
            height = ARENA_HEIGHT_P;
        }
    
        // Get widget and image sizes to scale the images correctly
        int widget_width = get_allocation().get_width();
        int widget_height = get_allocation().get_height();
    
        int img_width = background->get_width();
        int img_height = background->get_height();
    
        double scale_x = static_cast<double>(widget_width) / img_width;
        double scale_y = static_cast<double>(widget_height) / img_height;
        double scale = std::min(scale_x, scale_y);
    
        double scaled_width = img_width * scale;
        double scaled_height = img_height * scale;
        double offset_x = (widget_width - scaled_width) / 2.0;
        double offset_y = (widget_height - scaled_height) / 2.0;
    
        // Apply transformations for both background and overlay
        cr->save();
        cr->translate(offset_x, offset_y);
        cr->scale(scale, scale);
    
        cr->save();
        Gdk::Cairo::set_source_pixbuf(cr, background, 0, 0);
        cr->paint();
        cr->restore();

        cr->save();
        
        double cam_offset_x = 20.0; // meters * 160
        double cam_offset_y = 60.0;

        double cos_theta = std::cos(rotation_angle);
        double sin_theta = std::sin(rotation_angle);
        double rotated_offset_x = cam_offset_x * cos_theta - cam_offset_y * sin_theta;
        double rotated_offset_y = cam_offset_x * sin_theta + cam_offset_y * cos_theta;

        cr->translate(img_x + rotated_offset_x + overlay->get_width() / 2,
                    height - (img_y + rotated_offset_y + overlay->get_height() / 2));
        cr->rotate(rotation_angle);
        cr->translate(-overlay->get_width() / 2, -overlay->get_height() / 2);

        Gdk::Cairo::set_source_pixbuf(cr, overlay, 0, 0);
        cr->paint();
        cr->restore();

        // Draw rocks
        for (const auto& data : rock_data) {
            int new_width = rock->get_width() * data.scale_multiplier;
            int new_height = rock->get_height() * data.scale_multiplier;
            auto scaled_pixbuf = rock->scale_simple(new_width, new_height, Gdk::INTERP_BILINEAR);
            int draw_x = data.x - (new_width / 2);
            int draw_y = height - (data.y + new_height / 2);
            Gdk::Cairo::set_source_pixbuf(cr, scaled_pixbuf, draw_x, draw_y);
            cr->paint();
        }
    
        // Draw holes
        for (const auto& data : hole_data) {
            int new_width = hole->get_width() * data.scale_multiplier;
            int new_height = hole->get_height() * data.scale_multiplier;
            auto scaled_pixbuf = hole->scale_simple(new_width, new_height, Gdk::INTERP_BILINEAR);
            int draw_x = data.x - (new_width / 2);
            int draw_y = height - (data.y + new_height / 2);
            Gdk::Cairo::set_source_pixbuf(cr, scaled_pixbuf, draw_x, draw_y);
            cr->paint();
        }

        if(dest_x != -1 && dest_y != -1){
            int dest_img_w = dest_image->get_width();
            int dest_img_h = dest_image->get_height();

            int draw_x = dest_x - dest_img_w / 2;
            int draw_y = height - (dest_y + dest_img_h / 2);

            Gdk::Cairo::set_source_pixbuf(cr, dest_image, draw_x, draw_y);
            cr->paint();
        }

        cr->restore();
        cr->reset_clip();
    
        return true;
    }
    
    private:
        Glib::RefPtr<Gdk::Pixbuf> background, overlay, rock, hole, dest_image;
        double img_x, img_y;
        double rotation_angle;

        int dest_x, dest_y;

        struct ImageData {
            int x, y;
            double scale_multiplier;
            ImageData(int x, int y, double scale) : x(x), y(y), scale_multiplier(scale) {}
        };
        std::vector<ImageData> rock_data;
        std::vector<ImageData> hole_data;
        double m_scale_multiplier;
    
        void load_images(){
            try{
                if(mapUsed == "NASA"){
                    background = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/Arena.png");
                }
                else if(mapUsed == "UCF"){
                    background = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/UCFArena.png");
                }
                else if(mapUsed == "Cosmic"){
                    background = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/CosmicArena.png");
                }
                else if(mapUsed == "Lab"){
                    background = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/LabArena.png");
                }
                else{
                    background = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/Arena.png");
                }
                overlay = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/RobotTop.png");
                rock = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/Rock.png");
                hole = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/Hole.png");
                dest_image = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/X.png");
            }
            catch(const Glib::Exception& ex){
                g_warning("Failed to load images: %s", ex.what().c_str());
            }
        }
};

ImageOverlay* overlay_area;

bool set_source_hex_color(const Cairo::RefPtr<Cairo::Context>& cr, const std::string& color_string) {
    if (color_string.empty()) return false;

    if (color_string[0] == '#' && color_string.length() == 7) {
        try {
            int r = std::stoi(color_string.substr(1, 2), nullptr, 16);
            int g = std::stoi(color_string.substr(3, 2), nullptr, 16);
            int b = std::stoi(color_string.substr(5, 2), nullptr, 16);
            float R = r / 255.0;
            float G = g / 255.0;
            float B = b / 255.0;
            cr->set_source_rgb(R, G, B);
            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Invalid hex color: " << color_string << std::endl;
            return false;
        }
    }

    std::regex rgb_regex(R"(rgb\((\d+),\s*(\d+),\s*(\d+)\))");
    std::smatch match;
    if (std::regex_match(color_string, match, rgb_regex)) {
        try {
            int r = std::stoi(match[1]);
            int g = std::stoi(match[2]);
            int b = std::stoi(match[3]);
            float R = r / 255.0;
            float G = g / 255.0;
            float B = b / 255.0;
            // Not entirely sure why it needs to be BGR instead of RGB, but it does
            cr->set_source_rgb(B, G, R);
            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Invalid rgb() values: " << color_string << std::endl;
            return false;
        }
    }

    std::cerr << "Unsupported color format: " << color_string << std::endl;
    return true;
}

Glib::RefPtr<Gdk::Pixbuf> rotate_image(Glib::RefPtr<Gdk::Pixbuf> pixbuf, double angle_deg, int target_width, int target_height) {
    double angle_rad = angle_deg * M_PI / 180.0;

    int width = pixbuf->get_width();
    int height = pixbuf->get_height();

    int new_width = static_cast<int>(std::abs(width * std::cos(angle_rad)) + std::abs(height * std::sin(angle_rad)));
    int new_height = static_cast<int>(std::abs(width * std::sin(angle_rad)) + std::abs(height * std::cos(angle_rad)));

    auto surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, new_width, new_height);
    auto cr = Cairo::Context::create(surface);

    // Fill background
    cr->set_source_rgb(1.0, 1.0, 1.0); // Default to white
    if (angle_deg > 30 || angle_deg < -30) {
        cr->set_source_rgb(1.0, 0.0, 0.0); // Red for high angle warning
    }
    cr->paint();

    cr->translate(new_width / 2.0, new_height / 2.0);
    cr->rotate(angle_rad);
    cr->translate(-width / 2.0, -height / 2.0);

    Gdk::Cairo::set_source_pixbuf(cr, pixbuf, 0, 0);
    cr->paint();

    Glib::RefPtr<Gdk::Pixbuf> rotated_pixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, true, 8, new_width, new_height);

    const unsigned char* src_pixels = surface->get_data();
    int src_stride = surface->get_stride();
    unsigned char* dest_pixels = rotated_pixbuf->get_pixels();
    int dest_stride = rotated_pixbuf->get_rowstride();

    // Manually copy pixels, converting ARGB (Cairo) to RGBA (GdkPixbuf)
    for (int y = 0; y < new_height; ++y) {
        for (int x = 0; x < new_width; ++x) {
            const guint32* src_pixel = reinterpret_cast<const guint32*>(src_pixels + y * src_stride) + x;
            guint8* dest_pixel = dest_pixels + y * dest_stride + x * 4;

            // Cairo is ARGB (BGRA in little-endian memory) -> 0xAARRGGBB
            // GdkPixbuf wants RGBA
            dest_pixel[0] = (*src_pixel >> 16) & 0xFF; // Red
            dest_pixel[1] = (*src_pixel >> 8) & 0xFF;  // Green
            dest_pixel[2] = (*src_pixel >> 0) & 0xFF;  // Blue
            dest_pixel[3] = (*src_pixel >> 24) & 0xFF; // Alpha
        }
    }

    int crop_x = std::max(0, (new_width - target_width) / 2);
    int crop_y = std::max(0, (new_height - target_height) / 2);
    Glib::RefPtr<Gdk::Pixbuf> cropped_pixbuf = rotated_pixbuf->create_subpixbuf(rotated_pixbuf, crop_x, crop_y, target_width, target_height);

    // Draw black markers (unchanged)
    unsigned char* new_pixels = cropped_pixbuf->get_pixels();
    int new_rowstride = cropped_pixbuf->get_rowstride();
    int new_channels = cropped_pixbuf->get_n_channels();

    for (int y = 98; y <= 101; ++y) {
        unsigned char* row_start = new_pixels + y * new_rowstride;
        for (int x = 0; x <= 15; ++x) {
            unsigned char* new_pixel = row_start + x * new_channels;
            new_pixel[0] = 0; new_pixel[1] = 0; new_pixel[2] = 0;
            if (new_channels == 4) new_pixel[3] = 255;
        }
        for (int x = 185; x <= 199; ++x) {
            unsigned char* new_pixel = row_start + x * new_channels;
            new_pixel[0] = 0; new_pixel[1] = 0; new_pixel[2] = 0;
            if (new_channels == 4) new_pixel[3] = 255;
        }
    }

    return cropped_pixbuf;
}

class BorderedBox : public Gtk::Box {
    public:
    BorderedBox(Gtk::Orientation orientation, int spacing)
    : Gtk::Box(orientation, spacing) {}
    
    protected:
        bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override {
            Gtk::Box::on_draw(cr); 
    
            auto allocation = get_allocation();
            double width = allocation.get_width();
            double height = allocation.get_height();
    
            cr->set_line_width(1.0);
            cr->set_source_rgb(0, 0, 0);
    
            cr->rectangle(1, 1, width - 2, height - 2);
            cr->stroke();
    
            return true;
        }
    };

class CircleDrawingArea : public Gtk::DrawingArea{
    public:
        CircleDrawingArea()
        {
            color_.set_rgba(0.0, 0.0, 0.0, 1.0);
            background_color_.set_rgba(1.0, 1.0, 1.0, 1.0);
        }
    
        void set_color(const Gdk::RGBA& color)
        {
            color_ = color;
            queue_draw();
        }

        void set_background_color(const Gdk::RGBA& color){
            background_color_ = color;
            queue_draw();
        }
    
    protected:
        bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override
        {
            cr->set_source_rgba(background_color_.get_red(), background_color_.get_green(), background_color_.get_blue(), background_color_.get_alpha());
            cr->paint();
    
            cr->set_source_rgba(color_.get_red(), color_.get_green(), color_.get_blue(), color_.get_alpha());
    
            double width = get_width();
            double height = get_height();
            double radius = std::min(width, height) / 4;
    
            cr->arc(width/2, height/2, radius, 0, 2*M_PI);
            cr->fill();
    
            return true;
        }
    private:
        Gdk::RGBA color_;
        Gdk::RGBA background_color_;
    };

CircleDrawingArea* talon1Circle;
CircleDrawingArea* talon3Circle;
CircleDrawingArea* falcon1Circle;
CircleDrawingArea* falcon2Circle;
CircleDrawingArea* falcon3Circle;
CircleDrawingArea* falcon4Circle;
CircleDrawingArea* lowerFalcon1Circle;
CircleDrawingArea* lowerFalcon2Circle;
CircleDrawingArea* lowerFalcon3Circle;
CircleDrawingArea* lowerFalcon4Circle;

// TODO: Modify this to be more descriptive and make the graphs better
// Not entirely sure what all that will entail
// TODO: Fix potentiometer not displaying correctly
class MultiMotorGraph : public Gtk::Box {
    public:
        enum GraphType {
            VOLTAGE,
            CURRENT,
            POSITION,
            OUTPUT_PERCENT,
            SPEED,
            POTENTIOMETER
        };
    
        MultiMotorGraph(const std::string& title, GraphType type, const std::vector<std::string>& motorNames)
            : Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5),
              title(title),
              graphType(type),
              motorNames(motorNames) {
            
            // Setup colors and ranges based on graph type
            colors = {
                {1.0, 0.0, 0.0}, // Red - Motor 1
                {0.0, 0.5, 0.0}, // Green - Motor 2
                {0.0, 0.0, 1.0}, // Blue - Motor 3
                {1.0, 0.0, 1.0}, // Magenta - Motor 4
                {1.0, 0.5, 0.0}, // Orange - Motor 5
                {0.0, 0.5, 0.5}  // Teal - Motor 6
            };
    
            // Set ranges based on graph type
            switch(graphType) {
                case VOLTAGE:
                    minVal = 14.5f;
                    maxVal = 17.0f; // 14-17V for Talon voltage
                    yLabel = "Voltage (V)";
                    break;
                case CURRENT:
                    minVal = 0.0f;
                    maxVal = 50.0f; // 0-50A for current (adjust as needed)
                    yLabel = "Current (A)";
                    break;
                case POSITION:
                    minVal = 0.0f;
                    maxVal = 1024.0f; // 0-1024 for position (adjust based on your sensor)
                    yLabel = "Position (units)";
                    break;
                case OUTPUT_PERCENT:
                    minVal = 0.0f;
                    maxVal = 1.0f; // -100% to 100% output
                    yLabel = "Output (%)";
                    break;
                case SPEED:
                //TODO: SPEED RANGE 
                minVal = 0.0f;
                maxVal = 1.0f; // -100% to 100% speed
                yLabel = "Speed (normalized)";
                break;
                case POTENTIOMETER:
                //TODO: POTENTIOMETER RANGE 
                    minVal = 0.0f;
                    maxVal = 1024.0f; // 0-1024 typical for potentiometers
                    yLabel = "Potentiometer";
                    break;
            }
    
            // Create legend
            setup_legend();
            
            // Create drawing area
            setup_graph_area();
        }
    
        void update_data(const std::string& motorName, float value) {
            // For output percentage and speed, clamp values to [-1, 1] range
            if (graphType == OUTPUT_PERCENT || graphType == SPEED) {
                value = std::max(-1.0f, std::min(1.0f, value));
            }
            // For potentiometer, clamp to [0, 5] range
            else if (graphType == POTENTIOMETER) {
                value = std::max(0.0f, std::min(1024.0f, value));
            }
            
            data[motorName].push_back(value);
            
            if (data[motorName].size() > 100) {
                data[motorName].pop_front();
            }
            graphArea->queue_draw();
        }
    
    private:
        void setup_legend() {
            Gtk::Box* legendBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10));
            legendBox->property_margin().set_value(5);
            
            // Title with units
            Gtk::Label* titleLabel = Gtk::manage(new Gtk::Label(title + " (" + yLabel + ")"));
            titleLabel->set_halign(Gtk::ALIGN_START);
            legendBox->add(*titleLabel);
            
            // Color indicators
            for (size_t i = 0; i < motorNames.size(); i++) {
                Gtk::DrawingArea* colorSwatch = Gtk::manage(new Gtk::DrawingArea());
                colorSwatch->set_size_request(15, 15);
                colorSwatch->signal_draw().connect(
                    sigc::bind(sigc::mem_fun(*this, &MultiMotorGraph::draw_color_swatch), i));
                
                Gtk::Label* motorLabel = Gtk::manage(new Gtk::Label(motorNames[i]));
                motorLabel->set_margin_start(5);
                
                Gtk::Box* legendItem = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 0));
                legendItem->add(*colorSwatch);
                legendItem->add(*motorLabel);
                legendItem->set_margin_end(15);
                
                legendBox->add(*legendItem);
            }
            
            this->add(*legendBox);
        }
    
        void setup_graph_area() {
            graphArea = Gtk::manage(new Gtk::DrawingArea());
            graphArea->set_hexpand(true);
            graphArea->set_vexpand(true);
            graphArea->signal_draw().connect(
                sigc::mem_fun(*this, &MultiMotorGraph::draw_graph));
            this->add(*graphArea);
        }
    
        bool draw_color_swatch(const Cairo::RefPtr<Cairo::Context>& cr, int colorIndex) {
            const auto& color = colors[colorIndex % colors.size()];
            cr->set_source_rgb(color[0], color[1], color[2]);
            cr->rectangle(0, 0, 15, 15);
            cr->fill();
            return true;
        }
    
        bool draw_graph(const Cairo::RefPtr<Cairo::Context>& cr) {
            Gtk::Allocation alloc = graphArea->get_allocation();
            const int width = alloc.get_width();
            const int height = alloc.get_height();

            // Define colors for text, grid, and border based on the current mode.
            Gdk::RGBA text_color, grid_color, border_color;
            if (isLightMode) {
                set_source_hex_color(cr, lightBackgroundColor);
                text_color.set("black");
                grid_color.set_rgba(0.9, 0.9, 0.9, 1.0);
                border_color.set_rgba(0.7, 0.7, 0.7, 1.0);
            }
            else {
                set_source_hex_color(cr, darkBackgroundColor);
                text_color.set("white");
                grid_color.set_rgba(0.25, 0.25, 0.25, 1.0);
                border_color.set_rgba(0.4, 0.4, 0.4, 1.0);
            }
    
            // Clear background
            cr->paint();
    
            // Draw border
            cr->set_source_rgba(border_color.get_red(), border_color.get_green(), border_color.get_blue(), border_color.get_alpha());
            cr->rectangle(0, 0, width, height);
            cr->stroke();
    
            // Calculate grid steps based on range
            float range = maxVal - minVal;
            float step;
            
            if (graphType == OUTPUT_PERCENT || graphType == SPEED) {
                step = 0.1f; // 25% increments for output and speed
            }
            else if (graphType == POTENTIOMETER) {
                step = 100.0f; // 1V increments for potentiometer
            }
            else {
                step = (range > 1000) ? 100.0f :
                      (range > 20) ? 5.0f : 
                      (range > 10) ? 1.0f : 
                      (range > 5) ? 1.0f : 0.5f;
            }
    
            // Draw grid and labels
            cr->set_source_rgba(grid_color.get_red(), grid_color.get_green(), grid_color.get_blue(), grid_color.get_alpha());
            cr->select_font_face("Sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_NORMAL);
            cr->set_font_size(10);
            
            // Special case for output percentage and speed to show 0 line
            if (graphType == OUTPUT_PERCENT || graphType == SPEED) {
                float zeroY = height - ((0 - minVal) / range) * (height - 20);
                cr->set_source_rgba(border_color.get_red(), border_color.get_green(), border_color.get_blue(), border_color.get_alpha());
                cr->move_to(0, zeroY);
                cr->line_to(width, zeroY);
                cr->stroke();
                
                cr->set_source_rgba(text_color.get_red(), text_color.get_green(), text_color.get_blue(), text_color.get_alpha());
                cr->move_to(5, zeroY - 5);
                cr->show_text("0");
            }
            
            for (float v = minVal; v <= maxVal; v += step) {
                // Skip 0 if we already drew it specially
                if ((graphType == OUTPUT_PERCENT || graphType == SPEED) && v == 0) {
                    continue;
                }
                
                float y = height - ((v - minVal) / range) * (height - 20);
                cr->set_source_rgba(grid_color.get_red(), grid_color.get_green(), grid_color.get_blue(), grid_color.get_alpha());
                cr->move_to(0, y);
                cr->line_to(width, y);
                cr->stroke();
                
                cr->set_source_rgba(text_color.get_red(), text_color.get_green(), text_color.get_blue(), text_color.get_alpha());
                cr->move_to(5, y - 5);
                
                // Format label based on value size and type
                if (graphType == OUTPUT_PERCENT || graphType == SPEED) {
                    cr->show_text(Glib::ustring::format(std::fixed, std::setprecision(0), v * 100) + "%");
                }
                else if (graphType == POTENTIOMETER) {
                    cr->show_text(Glib::ustring::format(std::fixed, std::setprecision(1), v) + "V");
                }
                else if (maxVal > 100) {
                    cr->show_text(Glib::ustring::format(std::fixed, std::setprecision(0), v));
                }
                else {
                    cr->show_text(Glib::ustring::format(std::fixed, std::setprecision(1), v));
                }
            }
    
            // Draw each motor's data
            for (size_t i = 0; i < motorNames.size(); i++) {
                const auto& name = motorNames[i];
                if (data[name].empty()) continue;
    
                const auto& color = colors[i % colors.size()];
                cr->set_source_rgb(color[0], color[1], color[2]);
                cr->set_line_width(1.5);
    
                bool first = true;
                for (size_t j = 0; j < data[name].size(); j++) {
                    float x = (j / 100.0) * (width - 20) + 10;
                    float y = height - ((data[name][j] - minVal) / range) * (height - 20);
                    
                    if (first) {
                        cr->move_to(x, y);
                        first = false;
                    } else {
                        cr->line_to(x, y);
                    }
                }
                cr->stroke();
            }
    
            return true;
        }
    
        std::string title;
        std::string yLabel;
        GraphType graphType;
        std::vector<std::string> motorNames;
        std::map<std::string, std::deque<float>> data;
        std::vector<std::array<double, 3>> colors;
        float minVal;
        float maxVal;
        Gtk::DrawingArea* graphArea;
};

MultiMotorGraph* talonVoltageGraph;
MultiMotorGraph* talonCurrentGraph;
MultiMotorGraph* talonPositionGraph;
MultiMotorGraph* talonOutputGraph;

MultiMotorGraph* falconVoltageGraph;
MultiMotorGraph* falconCurrentGraph;
MultiMotorGraph* falconPositionGraph;
MultiMotorGraph* falconOutputGraph;

MultiMotorGraph* linearSpeedGraph;
MultiMotorGraph* linearPotentiometerGraph;

Speedometer* leftSpeedometer;
Speedometer* rightSpeedometer;
bool displaySpeed = true;
bool numbersInside = true;
bool numberTicks = true;

std::string motorDisplayed = "Talon 1";
Speedometer* voltageDial;
Speedometer* temperatureDial;
DrawingArea* positionDial;
Speedometer* percentDial;
Speedometer* velocityDial;
Speedometer* currentDial;
bool displayMotor = false;


extern "C" void destroy_pixbuf_data(const guint8* data) {
    delete[] data;
}

class VideoWidget : public Gtk::DrawingArea {
public:
    VideoWidget() {}

    void setFrame(const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (frame.empty()) {
            latestFrame.release();
            currentPixbuf.reset();
        } else {
            latestFrame = frame.clone();

            cv::Mat frameToDisplay_CV = latestFrame;
            if (frameToDisplay_CV.channels() == 1) {
                cv::cvtColor(frameToDisplay_CV, frameToDisplay_CV, cv::COLOR_GRAY2RGB);
            }

            int width = frameToDisplay_CV.cols;
            int height = frameToDisplay_CV.rows;
            int cv_channels = frameToDisplay_CV.channels();
            int pixbuf_rowstride = width * cv_channels;
            size_t data_size = static_cast<size_t>(height) * pixbuf_rowstride;

            guchar* copiedData = new guchar[data_size];
            if (frameToDisplay_CV.isContinuous()) {
                std::memcpy(copiedData, frameToDisplay_CV.data, data_size);
            } else {
                for (int r = 0; r < height; ++r) {
                    std::memcpy(copiedData + r * pixbuf_rowstride,
                                frameToDisplay_CV.data + r * frameToDisplay_CV.step,
                                static_cast<size_t>(width) * cv_channels);
                }
            }

            currentPixbuf = Gdk::Pixbuf::create_from_data(
                static_cast<const guint8*>(copiedData),
                Gdk::COLORSPACE_RGB,
                false,
                8,
                width,
                height,
                pixbuf_rowstride,
                [](const guint8* data){
                    delete[] data;
                }
            );
        }
        queue_draw();
    }

protected:
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override {
        std::lock_guard<std::mutex> lock(frameMutex);
        Gtk::Allocation allocation = get_allocation();

        if (!currentPixbuf) {
            cr->set_source_rgb(0.1, 0.1, 0.1);
            cr->rectangle(0, 0, allocation.get_width(), allocation.get_height());
            cr->fill();
            return true;
        }

        int width = currentPixbuf->get_width();
        int height = currentPixbuf->get_height();
        int widget_width = allocation.get_width();
        int widget_height = allocation.get_height();

        double scale_ratio_x = (width > 0) ? static_cast<double>(widget_width) / width : 1.0;
        double scale_ratio_y = (height > 0) ? static_cast<double>(widget_height) / height : 1.0;
        double actual_scale_ratio = std::min(scale_ratio_x, scale_ratio_y);

        int scaled_width = static_cast<int>(width * actual_scale_ratio);
        int scaled_height = static_cast<int>(height * actual_scale_ratio);

        double draw_x = (widget_width - scaled_width) / 2.0;
        double draw_y = (widget_height - scaled_height) / 2.0;

        Glib::RefPtr<Gdk::Pixbuf> scaled_pixbuf = currentPixbuf;
        if (scaled_width > 0 && scaled_height > 0 &&
            (scaled_width != width || scaled_height != height)) {
            scaled_pixbuf = currentPixbuf->scale_simple(
                scaled_width, scaled_height, Gdk::INTERP_BILINEAR);
        }

        if (scaled_pixbuf) {
            Gdk::Cairo::set_source_pixbuf(cr, scaled_pixbuf, draw_x, draw_y);
            cr->paint();
        }
        return true;
    }

private:
    cv::Mat latestFrame;
    std::mutex frameMutex;
    Glib::RefPtr<Gdk::Pixbuf> currentPixbuf;
};

VideoWidget* videoArea;


class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("gazebo_image_subscriber")
    {
        // --- IMPORTANT ---
        // Change this topic name to match your Gazebo camera's topic.
        // You can find it by running 'ros2 topic list' while Gazebo is running.
        // Common examples: "/camera/image_raw", "/gazebo_cam/image_raw"
        const std::string GAZEBO_TOPIC = "/zed2i/left/image_raw"; 

        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", GAZEBO_TOPIC.c_str());

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            GAZEBO_TOPIC, 10, 
            std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Use cv_bridge to convert the ROS2 image message to an OpenCV Mat
            // We request BGR8 encoding, which is what OpenCV and GTK expect.
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            if (cv_ptr)
            {
                cv::Mat display_img;
                
                // Resize the image to fit the 1600x1000 video area
                // (just like the original videoMain thread did)
                cv::resize(cv_ptr->image, display_img, cv::Size(1600, 1000), 0, 0, cv::INTER_LINEAR);

                // Use the existing thread-safe mechanism to update the latestFrame
                {
                    std::lock_guard<std::mutex> lock(frameMutex);
                    latestFrame = display_img.clone();
                    newFrameAvailable = true; // Signal the main thread to redraw
                }
            }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};


void setBackgroundColors(Gdk::RGBA color){
    if(talon1Circle)
        talon1Circle->set_background_color(color);
    if(talon3Circle)
        talon3Circle->set_background_color(color);

    if(falcon1Circle)
        falcon1Circle->set_background_color(color);
    if(falcon2Circle)
        falcon2Circle->set_background_color(color);
    if(falcon3Circle)
        falcon3Circle->set_background_color(color);
    if(falcon4Circle)
        falcon4Circle->set_background_color(color);
    if(lowerFalcon1Circle)
        lowerFalcon1Circle->set_background_color(color);
    if(lowerFalcon2Circle)
        lowerFalcon2Circle->set_background_color(color);
    if(lowerFalcon3Circle)
        lowerFalcon3Circle->set_background_color(color);
    if(lowerFalcon4Circle)
        lowerFalcon4Circle->set_background_color(color);
}

InfoFrame* getInfoFrame(std::string label){
    for (InfoFrame* frame : infoFrameList) {
        if (frame->get_label() != label) continue;
        return frame;
    }
    return nullptr;
}


Gtk::Widget* get_flowbox_child_for(Gtk::FlowBox& flowbox, Gtk::Widget* target_widget) {
    for (auto* child : flowbox.get_children()) {
        auto* flowbox_child = dynamic_cast<Gtk::FlowBoxChild*>(child);
        if (!flowbox_child) continue;

        if (flowbox_child->get_child() == target_widget)
            return flowbox_child;
    }
    return nullptr;
}


// Dark mode
std::string darkMode =
    "* { font-family: 'Proxima Nova'; font-weight: bold; }\n"
    "window { background-color: " + darkBackgroundColor + "; }\n"
    "#dark_text, #dark_text label { color: #000000; }\n"
    "label, button, entry { color: #edf6fa; }\n"
    "button { border: 1px solid #edf6fa; background-color: transparent; }\n";

std::string lightMode = 
    "* { font-family: 'Proxima Nova'; font-weight: bold }\n"
    "window { background-color: " + lightBackgroundColor + "; }\n"
    "label, button, entry { color: #000000; }\n"
    "button {  border: 1px solid #000000; background-color: #f0f0f0; }\n";


std::string generateDarkModeString(const std::string& color) {
    return
        "* { font-family: 'Proxima Nova'; font-weight: bold; }\n"
        "window, notebook, box, flowbox { background-color: " + color + "; }\n"
        "#dark_text, #dark_text label { color: #000000; }\n"
        "label, button, entry { color: #edf6fa; }\n"
        "button { border: 1px solid #edf6fa; background-color: transparent; }\n"
        
        "notebook tab { background-color: #2a2a2e; border-color: #444; }\n"
        "notebook tab label { color: #edf6fa; }\n"
        "notebook tab:checked { background-color: " + color + "; }\n";
}

// Light mode
std::string generateLightModeString(const std::string& color) {
    return
        "* { font-family: 'Proxima Nova'; font-weight: bold }\n"
        "window, notebook, box, flowbox { background-color: " + color + "; }\n"
        "label, button, entry { color: #000000; }\n"
        "button { border: 1px solid #000000; background-color: #f0f0f0; }\n"
        
        "notebook tab { background-color: #e6e6e6; border-color: #cccccc; }\n"
        "notebook tab label { color: #000000; }\n"
        "notebook tab:checked { background-color: " + color + "; }\n";
}

void updateBackgroundColor(InfoFrame* infoFrame, std::string label){
    if(isLightMode){
        infoFrame->setBackground(label, lightBackgroundColor);
        infoFrame->setTextColor(label, "#000000", false);
    }
    else{
        infoFrame->setBackground(label, darkBackgroundColor);
        infoFrame->setTextColor(label, "white", false);
    }
}

void toggleMode() {
    Gdk::RGBA background;
    isLightMode = !isLightMode;

    auto css_provider = Gtk::CssProvider::create();
    if (isLightMode) {
        css_provider->load_from_data(generateLightModeString(lightBackgroundColor));
        background.set(lightBackgroundColor);    
    }
    else {
        css_provider->load_from_data(generateDarkModeString(darkBackgroundColor));
        background.set(darkBackgroundColor);
    }

    auto screen = Gdk::Screen::get_default();
    Gtk::StyleContext::add_provider_for_screen(
        screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION
    );
    
    for (InfoFrame* frame : infoFrameList) {
        std::string label = frame->get_label();
        std::vector<std::string> keys = getKeys(label);
        for (const std::string& key : keys) {
            updateBackgroundColor(frame, key);
        }
    }

    if(!noVideo) {
        setBackgroundColors(background);
    }
}

void updateBackgroundColor(Gtk::Box* box, bool synced){
    if(synced){
        Gdk::RGBA red;
        red.set_rgba(1.0,0,0,1.0);
        box->override_background_color(red);
    }
    else{
        Gdk::RGBA white;
        white.set_rgba(1.0,1.0,1.0,1.0);
        box->override_background_color(white);
    }
}

const std::set<std::string> talonLabels = {"Talon 1", "Talon 3"};
const std::set<std::string> falconLabels = {"Falcon 1", "Falcon 2", "Falcon 3", "Falcon 4"};


CircleDrawingArea* getTalonCircle(const std::string& label) {
    if (label == "Talon 1") return talon1Circle;
    if (label == "Talon 3") return talon3Circle;
    return nullptr;
}


CircleDrawingArea* getFalconCircle(const std::string& label) {
    if (label == "Falcon 1") return falcon1Circle;
    if (label == "Falcon 2") return falcon2Circle;
    if (label == "Falcon 3") return falcon3Circle;
    if (label == "Falcon 4") return falcon4Circle;
    return nullptr;
}

CircleDrawingArea* getLowerFalconCircle(const std::string& label) {
    if (label == "Falcon 1") return lowerFalcon1Circle;
    if (label == "Falcon 2") return lowerFalcon2Circle;
    if (label == "Falcon 3") return lowerFalcon3Circle;
    if (label == "Falcon 4") return lowerFalcon4Circle;
    return nullptr;
}


void updateCircleColor(CircleDrawingArea* circle, bool lowVoltage) {
    if (!circle || noVideo) return;

    Gdk::RGBA color;
    if (lowVoltage)
        color.set_rgba(1.0, 0.0, 0.0, 1.0); // Red
    else
        color.set_rgba(0.0, 1.0, 0.0, 1.0); // Green

    circle->set_color(color);
}


void updateCircleColor(CircleDrawingArea* circle, Gdk::RGBA color) {
    if (!circle || noVideo) return;
    circle->set_color(color);
}


/* Functions associated with the motor details window */
bool updateMotorDetails = false;
bool allowMotorsDoubleClick = true;
void updateMotor(std::string label, const std::vector<Element>& elements) {
    if(label != motorDisplayed)
        return;
    
    for (const auto& element : elements) {
        if (element.label == "Bus Voltage") {
            float voltage = element.data.front().uint16 / 100.0f;
            voltageDial->set_speed((double)voltage);
        }
        else if(element.label == "Output Current"){
            float current = element.data.front().uint16 / 100.0f;
            currentDial->set_speed((double)current);
        }
        else if(element.label == "Output Percent"){
            float percent = element.data.front().float32;
            percentDial->set_speed((double)percent * 100);
        }
        else if(element.label == "Temperature"){
            int temperature = element.data.front().uint16;
            temperatureDial->set_speed((double)temperature);
        }
        else if(element.label == "Sensor Position"){
            int pos = element.data.front().uint16;
            if(label == "Talon 1" || label == "Talon 3")
                positionDial->set_height_ratio((920 - pos) / 920.0);
        }
        else if(element.label == "Sensor Velocity"){
            int pos = element.data.front().uint16;
            velocityDial->set_speed((double)pos);
        }
    }
}

Speedometer* createDial(std::string label, double min_speed, double max_speed, 
                        int major_divisions, int minor_ticks, double zero_angle, double sweep){
    auto speedometer = Gtk::manage(new Speedometer(label));
    speedometer->set_size_request(300, 300);
    speedometer->set_display_speed(displaySpeed);
    speedometer->set_numbers_inside(numbersInside);
    speedometer->set_numbers_on_ticks(numberTicks);
    speedometer->set_min_speed(min_speed);
    speedometer->set_max_speed(max_speed);
    speedometer->set_num_major_divisions(major_divisions);
    speedometer->set_num_minor_ticks_per_segment(minor_ticks);
    speedometer->set_angle_for_zero(zero_angle);
    speedometer->set_angle_for_sweep(sweep);
    speedometer->set_hexpand(false);
    return speedometer;
}

void create_motor_detail_window(const std::string& label){
    motorDisplayed = label;
    motorWindow = new Gtk::Window();
    std::string title = label + " Details";
    motorWindow->set_title(title);
    motorWindow->set_default_size(900, 900);
    auto outerBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    auto upperBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 5));
    auto lowerBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 5));

    voltageDial = createDial("Voltage", 14.0, 17.0, 3, 4, 210.0, 120.0);
    voltageDial->set_speed(16.0);
    voltageDial->set_low_warning(true);
    voltageDial->set_low_warning_thresh(0.333);
    voltageDial->set_use_text_label(true);
    voltageDial->set_text_label("Volts DC");
    upperBox->add(*voltageDial);

    temperatureDial = createDial("Temperature", 20.0, 100.0, 8, 4, 180.0, 180.0);
    temperatureDial->set_speed(45.0);
    temperatureDial->set_high_warning(true);
    temperatureDial->set_high_warning_thresh(0.25);
    temperatureDial->set_use_text_label(true);
    temperatureDial->set_text_label("* C");
    upperBox->add(*temperatureDial);

    if(label == "Talon 1" || label == "Talon 3"){
        auto positionBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
        positionDial = Gtk::manage(new DrawingArea());
        positionDial->set_size_request(40, 250);
        positionDial->set_hexpand(true);
        positionDial->set_halign(Gtk::ALIGN_CENTER);
        positionDial->show();
        positionDial->set_height_ratio(0.5);
        positionBox->add(*positionDial);
        auto positionLabel = Gtk::manage(new Gtk::Label("Position"));
        positionBox->add(*positionLabel);
        upperBox->add(*positionBox);
    }
    

    percentDial = createDial("Output Percent", 0.0, 100.0, 10, 4, 135.0, 270.0);
    percentDial->set_speed(45.0);
    percentDial->set_high_warning(true);
    percentDial->set_high_warning_thresh(0.1);
    percentDial->set_use_text_label(true);
    percentDial->set_text_label("% Power");
    lowerBox->add(*percentDial);

    velocityDial = createDial("Velocity", 0.0, 10.0, 10, 4, 135.0, 270.0);
    velocityDial->set_speed(5.0);
    lowerBox->add(*velocityDial);

    currentDial = createDial("Output Current", 0.0, 100.0, 10, 4, 135.0, 270.0);
    currentDial->set_speed(45.0);
    currentDial->set_high_warning(true);
    currentDial->set_high_warning_thresh(0.25);
    currentDial->set_use_text_label(true);
    currentDial->set_text_label("Amps");
    lowerBox->add(*currentDial);

    outerBox->add(*upperBox);
    outerBox->add(*lowerBox);

    motorWindow->add(*outerBox);

    motorWindow->signal_hide().connect([]() {
        allowMotorsDoubleClick = true;
        updateMotorDetails = false;
    });

    motorWindow->show_all_children();
    motorWindow->show_all();
}

bool onMotorClick(GdkEventButton* event, const std::string& label){
    if(!allowMotorsDoubleClick)
        return false;
    if (event->type == GDK_2BUTTON_PRESS) {
        allowMotorsDoubleClick = false;
        updateMotorDetails = true;
        create_motor_detail_window(label);
        return true;
    }
    return false;
}


/*** Functions associated with GUI initialization ***/
/**
 * Creates a position indicator widget composed of two vertical bars and labels.
 */
Gtk::Box* createPositionIndicator(const std::string& title, int spacing,
                                  DrawingArea*& left_indicator, 
                                  DrawingArea*& right_indicator, 
                                  Gtk::Box*& container_box) 
{
    auto text_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 2));
    container_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, spacing));
    container_box->set_size_request(110, -1);
    
    left_indicator = Gtk::manage(new DrawingArea());
    left_indicator->set_size_request(40, 180);
    left_indicator->set_hexpand(true);
    left_indicator->set_halign(Gtk::ALIGN_CENTER);
    container_box->add(*left_indicator);
    left_indicator->show();
    
    right_indicator = Gtk::manage(new DrawingArea());
    right_indicator->set_size_request(40, 180);
    right_indicator->set_hexpand(true);
    right_indicator->set_halign(Gtk::ALIGN_CENTER);
    container_box->add(*right_indicator);
    right_indicator->show();
    right_indicator->set_height_ratio(0.5);
    
    container_box->set_halign(Gtk::ALIGN_CENTER);
    container_box->set_valign(Gtk::ALIGN_CENTER);
    
    text_box->add(*container_box);
    text_box->set_halign(Gtk::ALIGN_CENTER);
    
    auto pos_label = Gtk::manage(new Gtk::Label("L         R"));
    auto title_label = Gtk::manage(new Gtk::Label(title));
    
    pos_label->set_halign(Gtk::ALIGN_CENTER);
    title_label->set_halign(Gtk::ALIGN_CENTER);
    
    text_box->add(*pos_label);
    text_box->add(*title_label);
    
    return text_box;
}

/**
 * Creates and initializes an image widget from a file.
 */
bool createImageIndicator(Gtk::Image*& image_widget, Glib::RefPtr<Gdk::Pixbuf>& pixbuf, 
                          const std::string& file_path, Gtk::Container* parent, double initial_rotation)
{
    image_widget = Gtk::manage(new Gtk::Image());
    try {
        pixbuf = Gdk::Pixbuf::create_from_file(file_path);
    } catch(const Glib::FileError& e) {
        g_print("Failed to load image: %s\n", e.what().c_str());
        return false;
    }
    
    parent->add(*image_widget);

    Glib::RefPtr<Gdk::Pixbuf> new_pixbuf = rotate_image(pixbuf, initial_rotation, 200, 200);
    image_widget->set(new_pixbuf);
    return true;
}

void initRoll() {
    if (!roll_init) {
        Gtk::Container* parent = noVideo ? static_cast<Gtk::Container*>(sensorBox) : bottomLowerBox;
        
        bool success = createImageIndicator(roll_image, roll_pixbuf, 
            package_share_directory + "/resources/RobotSide.png", parent, roll_rotation_angle);

        if (success) {
            if (!noVideo) {
                auto* padding = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 5));
                padding->set_size_request(100, 100);
                bottomLowerBox->add(*padding);
            }
            roll_init = true;
            window->show_all();
        }
    }
}

void initPitch() {
    if (!pitch_init) {
        if (!noVideo) {
            auto* padding = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 5));
            padding->set_size_request(100, 100);
            bottomLowerBox->add(*padding);
        }

        Gtk::Container* parent = noVideo ? static_cast<Gtk::Container*>(sensorBox) : bottomLowerBox;
        
        bool success = createImageIndicator(pitch_image, pitch_pixbuf, 
            package_share_directory + "/resources/RobotBack.png", parent, pitch_rotation_angle);

        if (success) {
            pitch_init = true;
            window->show_all();
        }
    }
}

void initBucketLvl() {
    if (!bucketLevel_init) {
        if (!noVideo) {
            auto* padding = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 5));
            padding->set_size_request(100, 100);
            bottomLowerBox->add(*padding);
        }

        Gtk::Container* parent = noVideo ? static_cast<Gtk::Container*>(sensorBox) : bottomLowerBox;
        if (createImageIndicator(lvl_image, lvl_pixbuf, package_share_directory + "/resources/bucket.png", parent, 0)) {
            bucketLevel_init = true;
            window->show_all();
        }
    }
}

void initArmPos() {
    if (!arm_init) {
        auto* arm_widget = createPositionIndicator("Arm Positions", 5, left_arm, right_arm, armBox);
        
        if (noVideo)
            sensorBox->add(*arm_widget);
        else
            innerLeftBox->add(*arm_widget);

        arm_init = true;
        window->show_all();
    }
}

void initBucketPos() {
    if (!bucket_init) {
        auto* bucket_widget = createPositionIndicator("Bucket Positions", 20, left_bucket, right_bucket, bucketBox);
        
        if (noVideo)
            sensorBox->add(*bucket_widget);
        else
            innerRightBox->add(*bucket_widget);

        bucket_init = true;
        window->show_all();
    }
}


/*** Functions associated with GUI Updates ***/
const std::unordered_set<std::string> validLabels = {
    "Falcon 1", "Falcon 2", "Falcon 3", "Falcon 4",
    "Talon 1", "Talon 2", "Talon 3", "Talon 4",
    "Linear 1", "Linear 2", "Linear 3", "Linear 4",
    "Zed", "Autonomy", "Communication", "Power", "Power2", "Drivetrain"
};

void addElementToInfoFrame(std::string label, InfoFrame* frame, const Element& element) {
    std::map<std::string, bool>& values = getMap(label);
    auto it = values.find(element.label);
    bool end = it == values.end();
    if(it == values.end() || !it->second){
        return;
    }

    addElementToInfoFrame(frame, element);
}

/*
The following functions with the names handleNodeElements handle any 
specific logic that is required to update any widgets that use the 
values from the node. The Generic elements function then updates the 
values displayed in the sensors tab.
*/
void handleZedElements(const std::vector<Element>& elements) {
    for (const auto& element : elements) {
        if (element.type != TYPE::FLOAT32) continue;
        float value = element.data.front().float32;

        if (element.label == "roll") {
            roll_rotation_angle = std::round(value);
            roll_image->set(rotate_image(roll_pixbuf, -roll_rotation_angle, 200, 200));
        }
        else if (element.label == "yaw") {
            pitch_rotation_angle = std::round(value);
            pitch_image->set(rotate_image(pitch_pixbuf, pitch_rotation_angle, 200, 200));
        }
        else if (element.label == "pitch" && !noArena) {
            overlay_area->update_image_rotation(value - 90);
        }
        else if (element.label == "Z" && !noArena) {
            overlay_area->update_image_y(value * MULTIPLIER_Y);
        }
        else if (element.label == "X" && !noArena) {
            overlay_area->update_image_x(value * MULTIPLIER_X);
        }
    }
}

void handleDrivetrainElements(const std::vector<Element>& elements) {
    
}

void handleTalonElements(const std::string& label, const std::vector<Element>& elements) {
    for (const auto& element : elements) {
        if (element.label == "Sensor Position") {
            int pos = element.data.front().uint16;
            if (label == "Talon 1") {
                left_arm_pos = pos;
                left_arm->set_height_ratio((920 - pos) / 920.0);
            }
            else if (label == "Talon 3") {
                left_bucket_pos = pos;
                left_bucket->set_height_ratio((700 - pos) / 700.0);
            }

            bool synced = std::abs(left_arm_pos - right_arm_pos) > 50;
            if (label == "Talon 1" || label == "Talon 2")
                updateBackgroundColor(armBox, synced);
            else
                updateBackgroundColor(bucketBox, synced);

            if (!noVideo) talonPositionGraph->update_data(label, pos);
        }
        else if (element.label == "Bus Voltage") {
            float voltage = element.data.front().uint16 / 100.0f;
            if (!noVideo) talonVoltageGraph->update_data(label, voltage);
            updateCircleColor(getTalonCircle(label), voltage < LOW_VOLTAGE);
        }
        else if (element.label == "Output Current") {
            float current = element.data.front().uint16 / 100.0f;
            if (!noVideo) talonCurrentGraph->update_data(label, current);
        }
        else if (element.label == "Output Percent") {
            float percent = element.data.front().float32;
            if (!noVideo) talonOutputGraph->update_data(label, percent);
        }
    }
}

void handleFalconElements(const std::string& label, const std::vector<Element>& elements) {
    for (const auto& element : elements) {
        if (element.label == "Bus Voltage") {
            float voltage = element.data.front().uint16 / 100.0f;
            if (!noVideo) falconVoltageGraph->update_data(label, voltage);
            updateCircleColor(getFalconCircle(label), voltage < LOW_VOLTAGE);
        }
        else if (element.label == "Output Current") {
            float current = element.data.front().uint16 / 100.0f;
            if (!noVideo) falconCurrentGraph->update_data(label, current);
        }
        else if (element.label == "Output Percent") {
            float percent = element.data.front().float32;
            if (!noVideo) falconOutputGraph->update_data(label, percent);
            if(label == "Falcon 2" || label == "Falcon 4"){
                leftSpeedometer->set_speed(percent * 100.0);
            }
            if(label == "Falcon 1" || label == "Falcon 3"){
                rightSpeedometer->set_speed(percent * 100.0);
            }
        }
        else if (element.label == "Error"){
            bool error = element.data.front().boolean;
            updateCircleColor(getLowerFalconCircle(label), error);
        }
    }
}

void handleCommunicationElements(InfoFrame* frame, const std::vector<Element>& elements) {
    for (const auto& element : elements) {
        if (element.label != "Wi-Fi" && element.label != "CAN Bus") continue;

        std::string text;
        for (const auto& c : element.data) text += c.character;

        if (text == "NON-FUNCTIONAL" || text == "INTERFERENCE" || text == "DOWN") {
            frame->setBackground(element.label, "#FF0000");
            frame->setTextColor(element.label, "white", true);
        }
        else {
            updateBackgroundColor(frame, element.label);
        }
    }
}

void handleAutonomyElements(const std::string& label, const std::vector<Element>& elements) {
    int destX = -1;
    int destY = -1;
    for (const auto& element : elements) {
        if (element.label == "Dest X") {
            destX = element.data.front().float32 * MULTIPLIER_X;
            if(destY != -1){
                overlay_area->add_dest_loc(destX, destY);
                break;
            }
        }
        else if(element.label == "Dest Z"){
            destY = element.data.front().float32 * MULTIPLIER_Y;
            if(destX != -1){
                overlay_area->add_dest_loc(destX, destY);
                break;
            }
        }
    }
}

void handleGenericElements(std::string label, InfoFrame* frame, const std::vector<Element>& elements) {
    std::map<std::string, bool>& values = getMap(label);
    for (const auto& element : elements) {
        auto it = values.find(element.label);
        if(it == values.end() || !it->second)
            continue;
        const auto& value = element.data.front();

        if (element.type == TYPE::BOOLEAN)       frame->setItem(element.label, value.boolean);
        else if (element.type == TYPE::UINT8)     frame->setItem(element.label, value.uint8);
        else if (element.type == TYPE::INT8)      frame->setItem(element.label, value.int8);
        else if (element.type == TYPE::UINT16) {
            if(element.label == "Bus Voltage" || element.label == "Output Current"){
                float val = value.uint16 / 100.0f;
                updateBackgroundColor(frame, element.label);

                if (element.label == "Bus Voltage" && val < LOW_VOLTAGE) {
                    frame->setBackground(element.label, "#FF0000");
                    frame->setTextColor(element.label, "white", true);
                }
                
                frame->setItem(element.label, val);
            }
            else{
                frame->setItem(element.label, value.uint16);
            }

        }
        else if (element.type == TYPE::INT16)     frame->setItem(element.label, value.int16);
        else if (element.type == TYPE::UINT32)    frame->setItem(element.label, value.uint32);
        else if (element.type == TYPE::INT32)     frame->setItem(element.label, value.int32);
        else if (element.type == TYPE::UINT64)    frame->setItem(element.label, value.uint64);
        else if (element.type == TYPE::INT64)     frame->setItem(element.label, value.int64);
        else if (element.type == TYPE::FLOAT32)   frame->setItem(element.label, value.float32);
        else if (element.type == TYPE::FLOAT64)   frame->setItem(element.label, value.float64);
        else if (element.type == TYPE::STRING) {
            std::string text;
            for (const auto& c : element.data) text += c.character;
            frame->setItem(element.label, text);
        }
    }
    frame->show_all();
}

void updateGUI(BinaryMessage& message) {
    std::string label = message.getLabel();

    for (InfoFrame* frame : infoFrameList) {
        if (frame->get_label() != label) continue;

        const auto& elements = message.getObject().elementList;

        if (label == "Zed") {
            handleZedElements(elements);
        }
        else if (label == "Communication") {
            handleCommunicationElements(frame, elements);
        }
        else if (talonLabels.count(label)) {
            handleTalonElements(label, elements);
        }
        else if (falconLabels.count(label)) {
            handleFalconElements(label, elements);
        }
        else if(label == "Autonomy"){
            handleAutonomyElements(label, elements);
        }
        else if(label == "Drivetrain"){
            handleDrivetrainElements(elements);
        }
        if(updateMotorDetails){
            updateMotor(label, elements);
        }

        handleGenericElements(label, frame, elements);
        return;
    }
    if (!validLabels.count(label)) return;

    if ((label == "Talon 1" || label == "Talon 2") && !arm_init) 
        initArmPos();
    if ((label == "Talon 3" || label == "Talon 4") && !bucket_init) 
        initBucketPos();
    if (label == "Zed" && !roll_init) 
        initRoll();
    if(label == "Zed" && !pitch_init)
        initPitch();

    InfoFrame* infoFrame = Gtk::manage(new InfoFrame(label));
    infoFrameList.push_back(infoFrame);

    for (const Element& element : message.getObject().elementList) {
        addElementToInfoFrame(infoFrame, element);
    }

    Gtk::EventBox* frameBox = Gtk::manage(new Gtk::EventBox());
    frameBox->add(*infoFrame);
    frameBox->show_all();
    if(label == "Talon 1" || label == "Talon 3" ||
       label == "Falcon 1" || label == "Falcon 2" || label == "Falcon 3" || label == "Falcon 4"){
        frameBox->signal_button_press_event().connect(
            [label](GdkEventButton* event) -> bool {
                return onMotorClick(event, label);
            },
            false
        );
    }

    sensorBox->add(*frameBox);
    infoFrame->show_all();
}

/**
 * @brief Processes an incoming payload, decompressing it only if necessary.
 * * This function reads the first byte of the payload as a flag.
 * - If the flag is '1', it assumes the data is compressed, extracts the
 * original size, and performs zlib decompression.
 * - If the flag is '0', it assumes the data is uncompressed and copies it directly.
 * * @param received_payload The raw data buffer received from the socket.
 * @param processed_data A vector that will be filled with the final, usable data.
 * @return True if processing was successful, false otherwise.
 */
bool process_payload(const std::vector<uint8_t>& received_payload, std::vector<uint8_t>& processed_data) {
    if (received_payload.empty()) {
        return false;
    }

    // Read the first byte as the compression flag.
    uint8_t compression_flag = received_payload[0];

    if (compression_flag == 1) {
        if (received_payload.size() < 5) { // 1-byte flag + 4-byte size
            std::cerr << "Error: Compressed payload is too small." << std::endl;
            return false;
        }

        // Extract the original uncompressed size from the next 4 bytes.
        uLong original_size = 0;
        original_size |= static_cast<uLong>(received_payload[1]) << 24;
        original_size |= static_cast<uLong>(received_payload[2]) << 16;
        original_size |= static_cast<uLong>(received_payload[3]) << 8;
        original_size |= static_cast<uLong>(received_payload[4]) << 0;
        
        processed_data.resize(original_size);
        uLongf dest_len = processed_data.size();

        // Point to the actual compressed data (after flag and size).
        const Bytef* source = received_payload.data() + 5;
        uLong source_len = received_payload.size() - 5;

        // Perform decompression.
        int result = uncompress(processed_data.data(), &dest_len, source, source_len);
        if (result != Z_OK) {
            std::cerr << "Decompression failed with error: " << result << std::endl;
            return false;
        }
        processed_data.resize(dest_len);

    } else {
        // Just copy the data, skipping the '0' flag byte.
        processed_data.assign(received_payload.begin() + 1, received_payload.end());
    }

    return true;
}

// This function populates a binary message with default values for all of the values that are
// associated with the particular info frame
void populateBinaryMessage(const std::string& name, const std::string& prefix, BinaryMessage& message) {
    std::string vector_name = getNameFromPrefix(prefix);
    auto keys_it = get_key_vectors().find(vector_name);
    auto defs_it = get_element_definitions().find(prefix);
    if (keys_it == get_key_vectors().end() || defs_it == get_element_definitions().end()) {
        std::cerr << "Warning: Missing keys or definitions for prefix " << prefix << std::endl;
        return;
    }
    const auto& keys = *keys_it->second;
    const auto& defs = defs_it->second;
    std::map<std::string, ElementType> type_map;
    for (const auto& def : defs) {
        type_map[def.name] = def.type;
    }
    for (const std::string& key : keys) {
        auto type_it = type_map.find(key);
        if (type_it == type_map.end()) continue;

        ElementType type = type_it->second;

        if      (type == ElementType::UInt8)   message.addElementUInt8(key, 0);
        else if (type == ElementType::UInt16)  message.addElementUInt16(key, 0);
        else if (type == ElementType::Int8)    message.addElementInt8(key, 0);
        else if (type == ElementType::Int32)   message.addElementInt32(key, 0);
        else if (type == ElementType::Float32) message.addElementFloat32(key, 0.0f);
        else if (type == ElementType::Boolean) message.addElementBoolean(key, false);
        else if (type == ElementType::String)  message.addElementString(key, "");
    }
}

void createMessage(std::string name, std::string prefix){
    BinaryMessage message(name);
    populateBinaryMessage(name, prefix, message);
    updateGUI(message);
}

void initGUI() {
    if(initVals){
        createMessage("Talon 1", "TALON");
        createMessage("Talon 3", "TALON");
        createMessage("Falcon 1", "FALCON");
        createMessage("Falcon 2", "FALCON");
        createMessage("Falcon 3", "FALCON");
        createMessage("Falcon 4", "FALCON");
        
        createMessage("Linear 1", "LINEAR");
        createMessage("Linear 3", "LINEAR");
        
        initRoll();
        initPitch();
        initArmPos();
        initBucketPos();
        
        createMessage("Communication", "COMMUNICATION");
        createMessage("Autonomy", "AUTONOMY");
        createMessage("Zed", "ZED");
        createMessage("Power", "POWER");
        createMessage("Power2", "POWER2");
        createMessage("Drivetrain", "DRIVETRAIN");
    }
    
    // Ensure proper initial display
    window->set_default_size(1200, 900);
    window->show_all();
}

void updateGUI(){
    for (InfoFrame* frame : infoFrameList) {
        std::string label = frame->get_label();
        frame->removeAllItems();
        std::map<std::string, bool>& values = getMap(label);
        std::vector<std::string> keys = getKeys(label);
        for (const std::string& key : keys) {
            auto it = values.find(key);
            if (it != values.end() && it->second) {
                frame->addItem(key);
                updateBackgroundColor(frame, key); 
            }
        }
    }

    initGUI();
}


/*** Helper functions and variables for the video and robot server connections ***/
bool contains(std::vector<std::string>& list, std::string& value){
    for(std::string storedValue: list) if(storedValue==value) return true;
    return false;
}


/*** Functions associated with the server ***/
void resetUIOnDisconnect() {
    for (InfoFrame* frame : infoFrameList) {
        frame->setAllItemsStale();
    }


    // Reset the motor status indicator circles to black
    if(!noVideo) {
        Gdk::RGBA black;
        black.set_rgba(0.0, 0.0, 0.0, 1.0);
        updateCircleColor(talon1Circle, black);
        updateCircleColor(talon3Circle, black);
        updateCircleColor(falcon1Circle, black);
        updateCircleColor(falcon2Circle, black);
        updateCircleColor(falcon3Circle, black);
        updateCircleColor(falcon4Circle, black);
        updateCircleColor(lowerFalcon1Circle, black);
        updateCircleColor(lowerFalcon2Circle, black);
        updateCircleColor(lowerFalcon3Circle, black);
        updateCircleColor(lowerFalcon4Circle, black);
    }
}


/*** Functions associated with the Gear Select dial ***/
Gtk::ScrolledWindow* create_gear_dial(const std::vector<std::string>& gears,
                                      std::map<std::string, Gtk::Label*>& gear_labels,
                                      Gtk::Box*& label_container)
{
    auto* scroll = new Gtk::ScrolledWindow();
    scroll->set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
    scroll->set_propagate_natural_height(true);
    scroll->set_size_request(80, 120); // Dial size

    label_container = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 0));
    scroll->add(*label_container);

    for (const auto& gear : gears) {
        auto* label = Gtk::manage(new Gtk::Label(gear));
        label->set_margin_top(8);
        label->set_margin_bottom(8);
        label->set_alignment(0.5, 0.5);
        label->set_markup("<span size='8000' foreground='gray'>" + gear + "</span>");

        gear_labels[gear] = label;
        label_container->pack_start(*label, Gtk::PACK_SHRINK);
    }

    return scroll;
}


void highlight_gear_and_scroll(const std::string& current_gear,
                               const std::vector<std::string>& gears,
                               const std::map<std::string, Gtk::Label*>& gear_labels,
                               Gtk::ScrolledWindow* scroll,
                               Gtk::Box* label_container)
{
    int gear_index = 0;
    for (size_t i = 0; i < gears.size(); ++i) {
        const auto& gear = gears[i];
        auto* label = gear_labels.at(gear);

        if (gear == current_gear) {
            label->set_markup("<span size='12000' weight='bold' background='red' foreground='white'>" + gear + "</span>");
            gear_index = i;
        } else {
            label->set_markup("<span size='8000' foreground='gray'>" + gear + "</span>");
        }
    }

    // Scroll so current gear is in the middle
    auto adj = scroll->get_vadjustment();
    double row_height = 30.0; // Approximate
    double new_value = std::max(0.0, gear_index * row_height - scroll->get_height() / 2);
    adj->set_value(new_value);
}

std::vector<std::string> gears = {"M", "5", "4", "3", "2", "1"};
std::map<std::string, Gtk::Label*> gear_labels;
Gtk::Box* gear_label_box = nullptr;
Gtk::ScrolledWindow* gear_dial = nullptr;
std::string currentGear = "3";

void increaseGear(){
    auto it = std::find(gears.begin(), gears.end(), currentGear);
    if (it != gears.begin()) {
        std::string nextGear = *std::prev(it);  // Increase gear
        currentGear = nextGear;
        highlight_gear_and_scroll(currentGear, gears, gear_labels, gear_dial, gear_label_box);
    } else {
        std::cout << "Already at highest gear." << std::endl;
    }
}

void decreaseGear(){
    auto it = std::find(gears.begin(), gears.end(), currentGear);
    if (it != gears.end() && std::next(it) != gears.end()) {
        std::string nextGear = *std::next(it);  // Decrease gear
        currentGear = nextGear;
        highlight_gear_and_scroll(currentGear, gears, gear_labels, gear_dial, gear_label_box);
    } else {
        std::cout << "Already at lowest gear." << std::endl;
    }
}


/*** Functions associated with the config button and functionality ***/
std::map<std::string, std::string> tooltip_map = {
    {"DISPLAY_SPEED", "Show or hide the speedometer."},
    {"NUMBERS_INSIDE", "Display numbers inside the speedometer ring."},
    {"NUMBER_TICKS", "Align numbers with speedometer tick marks."},
    {"SHOW_FALCON_Device ID", "Show Falcon CAN ID in the telemetry frame."}
};


void add_tooltip(Gtk::CheckButton* check, const std::string& key) {
    auto it = tooltip_map.find(key);
    if (it != tooltip_map.end()) {
        check->set_tooltip_text(it->second);
    }
}


void save_value(std::map<std::string, bool>& values, const std::string& value, bool active) {
    values[value] = active;
}


InfoFrame *talonFrame = nullptr, *falconFrame = nullptr, *linearFrame = nullptr,
    *autonomyFrame = nullptr, *zedFrame = nullptr, *communicationFrame = nullptr,
    *powerFrame = nullptr, *power2Frame = nullptr, *drivetrainFrame = nullptr;

std::unordered_map<std::string, InfoFrame*> frame_map;
void setup_frame_map() {
    frame_map = {
        {"TALON",         talonFrame},
        {"FALCON",        falconFrame},
        {"LINEAR",        linearFrame},
        {"AUTONOMY",      autonomyFrame},
        {"ZED",           zedFrame},
        {"COMMUNICATION", communicationFrame},
        {"POWER",         powerFrame},
        {"POWER2",        power2Frame},
        {"DRIVETRAIN",    drivetrainFrame},
    };
}

std::map<std::string, Gtk::CheckButton*> bool_buttons;
std::vector<std::string> local_talon_keys = get_talon_keys();
std::vector<std::string> local_falcon_keys = get_falcon_keys();
std::vector<std::string> local_linear_keys = get_linear_keys();
std::vector<std::string> local_autonomy_keys = get_autonomy_keys();
std::vector<std::string> local_communication_keys = get_communication_keys();
std::vector<std::string> local_power2_keys = get_power2_keys();
std::vector<std::string> local_power_keys = get_power_keys();
std::vector<std::string> local_zed_keys = get_zed_keys();
std::vector<std::string> local_drivetrain_keys = get_drivetrain_keys();

std::map<std::string, std::vector<std::string>*> local_key_vectors = {
    {"Talon", &local_talon_keys},
    {"Falcon", &local_falcon_keys},
    {"Linear", &local_linear_keys},
    {"Autonomy", &local_autonomy_keys},
    {"Communication", &local_communication_keys},
    {"Power2", &local_power2_keys},
    {"Power", &local_power_keys},
    {"Zed", &local_zed_keys},
    {"Drivetrain", &local_drivetrain_keys}
};

void create_config_editor_window(const std::string& config_file) {
    if (configWindow) {
        configWindow->present();
        return;
    }

    if (!allowConfig) {
        return;
    }
    
    allowConfig = false;
    configWindow = new ConfigEditorWindow(config_file);
    configWindow->signal_hide().connect([]() {
        configWindow = nullptr; // Reset the pointer, as the object is now destroyed.
        allowConfig = true;     // Allow a new window to be created next time.
    });

    configWindow->show();
}


/*** Helper functions for creating GUI windows / binding events ***/
std::string current_ip = "http://192.168.1.8";
void send_servo_command(const std::string& direction) {
    CURL* curl = curl_easy_init();
    if (curl) {
        std::string url = current_ip + "/action?go=" + direction;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);  // Short timeout
        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK)
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        curl_easy_cleanup(curl);
    }
}

bool on_key_release_event(GdkEventKey* key_event){
    switch (key_event->keyval) {
        case GDK_KEY_u:
        case GDK_KEY_i:
        case GDK_KEY_o:
        case GDK_KEY_p:
            send_servo_command("stop");
            return false;
            break;
    }
    sendKeyboardEvent(key_event->keyval, 0); // 0 for key release
    return false;
}

bool on_key_press_event(GdkEventKey* key_event){
    switch (key_event->keyval) {
        case GDK_KEY_u:
            send_servo_command("left");
            return false;
            break;
        case GDK_KEY_i:
            send_servo_command("right");
            return false;
            break;
        case GDK_KEY_o:
            send_servo_command("up");
            return false;
            break;
        case GDK_KEY_p:
            send_servo_command("down");
            return false;
            break;
        case GDK_KEY_1:
            current_ip = "http://192.168.1.8";
            std::cout << "Switched to IP 1: " << current_ip << std::endl;
            return false;
            break;
        case GDK_KEY_2:
            current_ip = "http://192.168.1.9";
            std::cout << "Switched to IP 2: " << current_ip << std::endl;
            return false;
            break;
        case GDK_KEY_minus:
            decreaseGear();
            break;
        case GDK_KEY_plus:
            if(key_event->state & GDK_SHIFT_MASK)
                increaseGear();
            break;
    }
    sendKeyboardEvent(key_event->keyval, 1); // 1 for key press
    return false;
}

Gtk::EventBox* create_labeled_box(const Glib::ustring& label_text, CircleDrawingArea*& out_circle, bool right = false) {
    auto event_box = Gtk::manage(new Gtk::EventBox());

    auto box = Gtk::manage(new BorderedBox(Gtk::ORIENTATION_HORIZONTAL, 5));
    box->set_size_request(300, 75);

    auto label = Gtk::manage(new Gtk::Label(label_text));
    label->set_hexpand(true);

    Pango::FontDescription font;
    font.set_size(20 * Pango::SCALE);
    label->override_font(font);

    out_circle = Gtk::manage(new CircleDrawingArea());
    out_circle->set_size_request(75, 75);
    out_circle->set_hexpand(false);
    out_circle->set_halign(Gtk::ALIGN_CENTER);

    if(right){
        box->add(*label);
        box->add(*out_circle);
    }
    else{
        box->add(*out_circle);
        box->add(*label);
    }   

    event_box->add(*box);
    event_box->add_events(Gdk::BUTTON_PRESS_MASK);
    event_box->set_visible_window(false);

    return event_box;
}

bool onClickEvent(GdkEventButton* event, const std::string& id) {
    if (event->type == GDK_2BUTTON_PRESS) {
        auto target_infoframe = getInfoFrame(id);
        Gtk::FlowBoxChild* flowbox_child = dynamic_cast<Gtk::FlowBoxChild*>(get_flowbox_child_for(*sensorBox, target_infoframe));
        if (flowbox_child) {
            sensorBox->select_child(*flowbox_child);
        }
        return true;
    }
    return false;
}

Gtk::Box* create_motor_column(std::vector<std::pair<Glib::ustring, CircleDrawingArea**>> items, void (*init_hook)(), std::vector<std::string> labels, bool right = false) {
    auto column = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    column->set_size_request(300, 300);
    column->set_hexpand(false);
    column->set_vexpand(false);

    for (size_t i = 0; i < items.size(); ++i) {
        if (i == 2 && init_hook) init_hook();
        auto box = create_labeled_box(items[i].first, *items[i].second, right);
        std::string id = labels[i];
        box->signal_button_press_event().connect(
            [id](GdkEventButton* event) -> bool {
                return onClickEvent(event, id);
            },
            false
        );
        column->add(*box);
    }

    return column;
}


// To change Speedometer sizes, need to change this value
Gtk::Box* create_lower_motor_column(std::vector<std::pair<Glib::ustring, CircleDrawingArea**>> items, std::vector<std::string> labels, bool right = false) {
    auto column = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    column->set_size_request(200, 300);
    column->set_hexpand(false);
    column->set_vexpand(false);

    for (size_t i = 0; i < items.size(); ++i) {
        auto box = create_labeled_box(items[i].first, *items[i].second, right);
        std::string id = labels[i];
        box->signal_button_press_event().connect(
            [id](GdkEventButton* event) -> bool {
                return onClickEvent(event, id);
            },
            false
        );
        column->add(*box);
    }

    return column;
}

void on_connection_finished() {
    update_connection_status(server_ui);
}

void on_video_connection_finished() {
    update_video_connection_status(video_server_ui);
}


/*** Functions that setup the GUI and windows ***/
void setupGUI(Glib::RefPtr<Gtk::Application> application) {
    // 1. Initialize Global Pointers
    window = nullptr;
    ipAddressEntry = nullptr; connectButton = nullptr; connectionStatusLabel = nullptr;
    silentRunButton = nullptr; addressListBox = nullptr;
    videoConnectButton = nullptr; videoConnectionStatusLabel = nullptr; videoStreamButton = nullptr;
    videoIPAddressEntry = nullptr; videoAddressListBox = nullptr;
    toggleModeButton = nullptr; settingsButton = nullptr;
    sensorBox = nullptr; innerLeftBox = nullptr; innerRightBox = nullptr;

    initialize_maps(); 

    // 2. Load the Glade Layout
    auto builder = Gtk::Builder::create();
    try {
        builder->add_from_file(package_share_directory + "/resources/mainLayout.glade");
    }
    catch(const Glib::Error& ex) {
        std::cerr << "CRITICAL: Failed to load mainLayout.glade: " << ex.what() << std::endl;
        exit(1); 
    }

    // 3. Get Main Window
    builder->get_widget("mainWindow", window);
    if (!window) {
        std::cerr << "FATAL: 'mainWindow' ID not found in mainLayout.glade" << std::endl;
        exit(1);
    }
    window->maximize();

    try { window->set_icon_from_file(package_share_directory + "/resources/razorbotz.png"); } catch (...) {}

    // 4. Setup Event Handling
    window->add_events(Gdk::KEY_PRESS_MASK | Gdk::KEY_RELEASE_MASK);
    window->signal_key_press_event().connect(sigc::ptr_fun(&on_key_press_event));
    window->signal_key_release_event().connect(sigc::ptr_fun(&on_key_release_event));

    // 5. Bind Widgets
    builder->get_widget("list_robot_address", addressListBox);
    builder->get_widget("entry_robot_ip", ipAddressEntry);
    builder->get_widget("btn_robot_connect", connectButton);
    builder->get_widget("lbl_robot_status", connectionStatusLabel);
    builder->get_widget("btn_silent_run", silentRunButton);
    
    // Video controls
    builder->get_widget("list_video_address", videoAddressListBox);
    builder->get_widget("entry_video_ip", videoIPAddressEntry);
    builder->get_widget("btn_video_connect", videoConnectButton);
    builder->get_widget("lbl_video_status", videoConnectionStatusLabel);
    builder->get_widget("btn_video_stream", videoStreamButton);
    
    // Global controls
    builder->get_widget("btn_toggle_mode", toggleModeButton);
    builder->get_widget("btn_settings", settingsButton);

    // 6. Setup Sensor Box
    Gtk::Box* topLevelBox = nullptr;
    builder->get_widget("topLevelBox", topLevelBox);
    
    sensorBox = Gtk::manage(new Gtk::FlowBox());
    sensorBox->set_orientation(Gtk::ORIENTATION_HORIZONTAL);
    
    if (topLevelBox) {
        topLevelBox->add(*sensorBox);
    }

    // Set Defaults
    if(ipAddressEntry) ipAddressEntry->set_text(useOrin ? ORIN_IP : NANO_IP);
    if(videoIPAddressEntry) videoIPAddressEntry->set_text(useOrin ? ORIN_IP : NANO_IP);

    Gdk::RGBA red; red.set_rgba(1.0, 0, 0, 1.0);
    if(connectionStatusLabel) connectionStatusLabel->override_background_color(red);
    if(videoConnectionStatusLabel) videoConnectionStatusLabel->override_background_color(red);

    if (settingsButton) {
        try {
            auto pixbuf = Gdk::Pixbuf::create_from_file(package_share_directory + "/resources/SettingsIcon.png");
            auto scaled = pixbuf->scale_simple(24, 24, Gdk::INTERP_BILINEAR);
            auto image = Gtk::manage(new Gtk::Image(scaled));
            settingsButton->set_image(*image);
        } catch (...) {}
    }

    auto css_provider = Gtk::CssProvider::create();
    css_provider->load_from_data(generateLightModeString(lightBackgroundColor));
    Gtk::StyleContext::add_provider_for_screen(Gdk::Screen::get_default(), css_provider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    try {
        auto font_provider = Gtk::CssProvider::create();
        font_provider->load_from_data("* { font-family: 'Proxima Nova'; }");
        Gtk::StyleContext::add_provider_for_screen(Gdk::Screen::get_default(), font_provider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    } catch (...) {}

    // 7. Update UI Structs
    server_ui.connectButton = connectButton;
    server_ui.connectionStatusLabel = connectionStatusLabel;
    server_ui.silentRunButton = silentRunButton;
    server_ui.ipAddressEntry = ipAddressEntry;
    server_ui.addressListBox = addressListBox;
    server_ui.parentWindow = window;

    video_server_ui.connectButton = videoConnectButton;
    video_server_ui.connectionStatusLabel = videoConnectionStatusLabel;
    video_server_ui.streamButton = videoStreamButton;
    video_server_ui.ipAddressEntry = videoIPAddressEntry;
    video_server_ui.addressListBox = videoAddressListBox;
    video_server_ui.parentWindow = window;

    // 8. Connect Signals
    if (connectButton) connectButton->signal_clicked().connect([&](){ connectOrDisconnect(server_ui, useOrin, connection_finished_dispatcher); });
    if (silentRunButton) silentRunButton->signal_clicked().connect([&](){ silentRun(server_ui); });
    if (addressListBox) addressListBox->signal_row_activated().connect([&](Gtk::ListBoxRow* row){ rowActivated(row, server_ui); });
    if (toggleModeButton) toggleModeButton->signal_clicked().connect(sigc::ptr_fun(&toggleMode));
    if (settingsButton) settingsButton->signal_clicked().connect([&](){ if(allowConfig) create_config_editor_window(get_configFile()); });
    if (videoConnectButton) videoConnectButton->signal_clicked().connect([&](){ videoConnectOrDisconnect(video_server_ui, video_connection_finished_dispatcher); });
    if (videoStreamButton) videoStreamButton->signal_clicked().connect([&](){ videoStream(video_server_ui); });
    if (videoAddressListBox) videoAddressListBox->signal_row_activated().connect([&](Gtk::ListBoxRow* row){ videoRowActivated(row, video_server_ui); });

    // 9. Handle Video/Map Layout
    if (!noVideo) {
        sensorBox->set_visible(false);

        Gtk::Box* bottomInnerBox = nullptr;
        builder->get_widget("box_bottom_inner", bottomInnerBox); 
        builder->get_widget("box_bottom_lower", bottomLowerBox);

        // --- FIXED: Use explicit initialization order to prevent Segfault ---

        // Inject Left Controls (Arm/Bucket)
        Gtk::Box* pLeft = nullptr; builder->get_widget("placeholder_inner_left", pLeft);
        if (pLeft) {
            // Pass nullptr for the hook, so we can assign innerLeftBox FIRST
            innerLeftBox = create_motor_column({
                {"Arm", &talon1Circle}, 
                {"Bucket", &talon3Circle}
                }, nullptr, 
                {"Talon 1", "Talon 3"}, 
                true);
            pLeft->add(*innerLeftBox);
            
            // Now that innerLeftBox is valid, initialize Arm Pos
            initArmPos();
        }

        // Inject Video Area
        Gtk::Box* pVideo = nullptr; builder->get_widget("placeholder_video_area", pVideo);
        if (pVideo) {
            videoArea = Gtk::manage(new VideoWidget());
            if(smallLaptop){
                videoArea->set_size_request(800, 500);
            }
            else{
                videoArea->set_size_request(1600, 1000);
            }
            pVideo->add(*videoArea);
        }

        // Inject Right Controls (Falcons)
        Gtk::Box* pRight = nullptr; builder->get_widget("placeholder_inner_right", pRight);
        if (pRight) {
            // Pass nullptr for the hook
            innerRightBox = create_motor_column({
                {"Falcon 1", &falcon1Circle}, 
                {"Falcon 2", &falcon2Circle}, 
                {"Falcon 3", &falcon3Circle}, 
                {"Falcon 4", &falcon4Circle}
                }, nullptr, 
                {"Falcon 1", "Falcon 2", "Falcon 3", "Falcon 4"}, 
                false);
            pRight->add(*innerRightBox);

            // Now that innerRightBox is valid, initialize Bucket Pos
            initBucketPos();
        }

        // Inject Lower Controls (Lower Falcons + Speedometers)
        Gtk::Box* pLowerLeft = nullptr; builder->get_widget("placeholder_lower_left", pLowerLeft);
        if (pLowerLeft) {
            Gtk::Box* lowerLeftBox = create_lower_motor_column({
                {"Falcon 1", &lowerFalcon1Circle}, 
                {"Falcon 2", &lowerFalcon2Circle}
                }, {"Falcon 1", "Falcon 2"}, 
                true);
            pLowerLeft->add(*lowerLeftBox);
        }

        Gtk::Box* pLowerRight = nullptr; builder->get_widget("placeholder_lower_right", pLowerRight);
        if (pLowerRight) {
            Gtk::Box* lowerRightBox = create_lower_motor_column({
                {"Falcon 3", &lowerFalcon3Circle}, 
                {"Falcon 4", &lowerFalcon4Circle}
                }, {"Falcon 3", "Falcon 4"});
            pLowerRight->add(*lowerRightBox);
        }
        
        Gtk::Box* pSpeedLeft = nullptr; builder->get_widget("placeholder_speed_left", pSpeedLeft);
        if(pSpeedLeft) {
            leftSpeedometer = Gtk::manage(new Speedometer("Left Speedometer"));
            leftSpeedometer->set_size_request(300, 175);
            leftSpeedometer->set_display_speed(displaySpeed);
            leftSpeedometer->set_numbers_inside(numbersInside);
            leftSpeedometer->set_numbers_on_ticks(numberTicks);
            pSpeedLeft->add(*leftSpeedometer);
        }

        Gtk::Box* pSpeedRight = nullptr; builder->get_widget("placeholder_speed_right", pSpeedRight);
        if(pSpeedRight) {
            rightSpeedometer = Gtk::manage(new Speedometer("Right Speedometer"));
            rightSpeedometer->set_size_request(300, 175);
            rightSpeedometer->set_display_speed(displaySpeed);
            rightSpeedometer->set_numbers_inside(numbersInside);
            rightSpeedometer->set_numbers_on_ticks(numberTicks);
            pSpeedRight->add(*rightSpeedometer);
        }

        Gtk::Box* pGear = nullptr; builder->get_widget("placeholder_gear_dial", pGear);
        if(pGear) {
            gear_dial = create_gear_dial(gears, gear_labels, gear_label_box);
            highlight_gear_and_scroll("3", gears, gear_labels, gear_dial, gear_label_box);
            pGear->add(*gear_dial);
        }
        
        // Inject Roll/Pitch Indicators
        Gtk::Box* pRoll = nullptr; builder->get_widget("placeholder_roll_image", pRoll);
        if(pRoll) {
            createImageIndicator(roll_image, roll_pixbuf, package_share_directory + "/resources/RobotSide.png", pRoll, roll_rotation_angle);
            roll_init = true;
        }

        Gtk::Box* pPitch = nullptr; builder->get_widget("placeholder_pitch_image", pPitch);
        if(pPitch) {
            createImageIndicator(pitch_image, pitch_pixbuf, package_share_directory + "/resources/RobotBack.png", pPitch, pitch_rotation_angle);
            pitch_init = true;
        }

        Gdk::RGBA background; background.set(lightBackgroundColor);
        setBackgroundColors(background);

    }
    else {
        // No Video Mode
        Gtk::Box* boxMainContent = nullptr;
        builder->get_widget("box_main_content", boxMainContent);
        if(boxMainContent) boxMainContent->set_visible(false);
        sensorBox->set_visible(true);
        initRoll();
        initPitch();
    }

    if (window) {
        window->signal_delete_event().connect(sigc::ptr_fun(quit));
        window->show_all();
    }
}

void initSensorsWindow() {
    sensorsWindow = new Gtk::Window();
    if(monitor_count == 3){
        auto display = Gdk::Display::get_default();
        auto third_monitor = display->get_monitor(2);
        Gdk::Rectangle third_monitor_geometry;
        third_monitor->get_geometry(third_monitor_geometry);
        sensorsWindow->set_default_size(third_monitor_geometry.get_width(), third_monitor_geometry.get_height());
        sensorsWindow->move(third_monitor_geometry.get_x(), third_monitor_geometry.get_y());
    }
    else{
        sensorsWindow->maximize();
    }

    Gtk::Box* mainBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    mainBox->property_margin().set_value(10);

    // Create motor name vectors
    std::vector<std::string> talonNames = {"Talon 1", "Talon 3" };
    std::vector<std::string> falconNames = {"Falcon 1", "Falcon 2", "Falcon 3", "Falcon 4"};
    std::vector<std::string> linearNames = {"Linear 1", "Linear 2"};

    // Create tabbed interface
    Gtk::Notebook* tabs = Gtk::manage(new Gtk::Notebook());
    tabs->set_vexpand(true);

    // Tab 1: Talon Motors
    Gtk::Box* talonTab = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    talonTab->property_margin().set_value(5);

    talonVoltageGraph = Gtk::manage(new MultiMotorGraph(
        "Talon Bus Voltage", MultiMotorGraph::VOLTAGE, talonNames));
    talonCurrentGraph = Gtk::manage(new MultiMotorGraph(
        "Talon Output Current", MultiMotorGraph::CURRENT, talonNames));
    talonPositionGraph = Gtk::manage(new MultiMotorGraph(
        "Talon Sensor Position", MultiMotorGraph::POSITION, talonNames));
    talonOutputGraph = Gtk::manage(new MultiMotorGraph(
        "Talon Output Percentage", MultiMotorGraph::OUTPUT_PERCENT, talonNames));

    talonTab->add(*talonVoltageGraph);
    talonTab->add(*talonCurrentGraph);
    talonTab->add(*talonPositionGraph);
    talonTab->add(*talonOutputGraph);
    tabs->append_page(*talonTab, "Talon Motors");

    // Tab 2: Falcon Motors
    Gtk::Box* falconTab = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    falconTab->property_margin().set_value(5);

    falconVoltageGraph = Gtk::manage(new MultiMotorGraph(
        "Falcon Bus Voltage", MultiMotorGraph::VOLTAGE, falconNames));
    falconCurrentGraph = Gtk::manage(new MultiMotorGraph(
        "Falcon Output Current", MultiMotorGraph::CURRENT, falconNames));
    falconPositionGraph = Gtk::manage(new MultiMotorGraph(
        "Falcon Sensor Position", MultiMotorGraph::POSITION, falconNames));
    falconOutputGraph = Gtk::manage(new MultiMotorGraph(
        "Falcon Output Percentage", MultiMotorGraph::OUTPUT_PERCENT, falconNames));

    falconTab->add(*falconVoltageGraph);
    falconTab->add(*falconCurrentGraph);
    falconTab->add(*falconPositionGraph);
    falconTab->add(*falconOutputGraph);
    tabs->append_page(*falconTab, "Falcon Motors");

    // Tab 3: Linear Actuators
    Gtk::Box* linearTab = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    linearTab->property_margin().set_value(5);

    linearSpeedGraph = Gtk::manage(new MultiMotorGraph(
        "Linear Actuator Speed", MultiMotorGraph::SPEED, linearNames));
    linearPotentiometerGraph = Gtk::manage(new MultiMotorGraph(
        "Linear Actuator Position", MultiMotorGraph::POTENTIOMETER, linearNames));

    linearTab->add(*linearSpeedGraph);
    linearTab->add(*linearPotentiometerGraph);
    tabs->append_page(*linearTab, "Linear Actuators");

    // Tab 4: Sensors Box
    Gtk::Box* sensorsTab = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    sensorsTab->property_margin().set_value(5);
    sensorBox = Gtk::manage(new Gtk::FlowBox());
    sensorBox->set_orientation(Gtk::ORIENTATION_HORIZONTAL);
    sensorsTab->add(*sensorBox);

    tabs->append_page(*sensorsTab, "Sensors");

    // Tab 5: Diagnostics Window
    // TODO: Figure out what information should be displayed here and add it
    Gtk::Box* diagnosticsTab = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    diagnosticsTab->property_margin().set_value(5);

    tabs->append_page(*diagnosticsTab, "Diagnostics");

    // Add everything to main window
    mainBox->add(*tabs);
    sensorsWindow->add(*mainBox);
    sensorsWindow->show_all();
}

void initArenaWindow(){
    arenaWindow = new Gtk::Window();
    arenaWindow->set_title("Arena Map/Cams");

    if(monitor_count == 3){
        auto display = Gdk::Display::get_default();
        auto second_monitor = display->get_monitor(1);
        Gdk::Rectangle second_monitor_geometry;
        second_monitor->get_geometry(second_monitor_geometry);
        arenaWindow->move(second_monitor_geometry.get_x(), second_monitor_geometry.get_y());
        arenaWindow->set_default_size(second_monitor_geometry.get_width(), second_monitor_geometry.get_height());
    }
    else{
        arenaWindow->maximize();
    }

    try {
        auto icon = package_share_directory + "/resources/razorbotz.png";
        arenaWindow->set_icon_from_file(icon);
    } catch (const Glib::FileError& e) {
        g_print("Failed to load image: %s\n", e.what().c_str());
        return;
    }

    // Arena cams
    // Add mainBox to window
    Gtk::Box* mainBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL,10));
    arenaWindow->add(*mainBox);
    
    // Arena map left
    overlay_area = Gtk::manage(new ImageOverlay());
    mainBox->pack_start(*overlay_area, Gtk::PACK_EXPAND_WIDGET);
    
    // Cameras box
    Gtk::Box* camsBox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
    mainBox->pack_start(*camsBox, Gtk::PACK_SHRINK);
    
    // Awareness Cam
    Gtk::Overlay* awareness_overlay = Gtk::manage(new Gtk::Overlay());
    Gtk::Box* livestreamBox1 = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL,0));
    livestreamBox1->set_size_request(800, 600);
    camsBox->pack_start(*awareness_overlay, Gtk::PACK_SHRINK);
    
    // Webview 1 (Awareness)
    auto webview1 = WEBKIT_WEB_VIEW(webkit_web_view_new());
    webkit_web_view_load_uri(webview1, "http://192.168.1.8/mjpeg/1");
    Gtk::Widget* webview_widget1 = Glib::wrap(GTK_WIDGET(webview1));
    livestreamBox1->pack_start(*webview_widget1, Gtk::PACK_EXPAND_WIDGET);
    awareness_overlay->add(*livestreamBox1);

    // Awareness cam label
    Gtk::Label* awareness_label = Gtk::manage(new Gtk::Label("Awareness Camera:"));
    //awareness_label->override_color(Gdk::RGBA("black"));
    awareness_label->set_halign(Gtk::ALIGN_START);
    awareness_label->set_valign(Gtk::ALIGN_START);
    awareness_overlay->add_overlay(*awareness_label);
    
    // Back Cam
    Gtk::Overlay* back_overlay = Gtk::manage(new Gtk::Overlay());
    Gtk::Box* livestreamBox2 = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL,0));
    livestreamBox2->set_size_request(800, 600);
    camsBox->pack_start(*back_overlay, Gtk::PACK_SHRINK);
    
    // Webview 2 (Back)
    auto webview2 = WEBKIT_WEB_VIEW(webkit_web_view_new());
    webkit_web_view_load_uri(webview2, "http://192.168.1.9/mjpeg/1");
    Gtk::Widget* webview_widget2 = Glib::wrap(GTK_WIDGET(webview2));
    livestreamBox2->pack_start(*webview_widget2, Gtk::PACK_EXPAND_WIDGET);
    back_overlay->add(*livestreamBox2);

    // Awareness cam label
    Gtk::Label* back_label = Gtk::manage(new Gtk::Label("Back Camera:"));
    //back_label->override_color(Gdk::RGBA("black"));
    back_label->set_halign(Gtk::ALIGN_START);
    back_label->set_valign(Gtk::ALIGN_START);
    back_overlay->add_overlay(*back_label);

    // Style the overlay label
    auto css_provider = Gtk::CssProvider::create();
    std::string format = "* { font-family: 'Proxima Nova'; }\n"
        ".overlay-text {\n"
            "font-size: 30px;\n"
            "background-color: " + lightBackgroundColor + ";\n"
            "padding: 5px;\n"
            "margin: 10px;\n"
            "border-radius: 3px;\n"
        "}";
    css_provider->load_from_data(format);
    awareness_label->get_style_context()->add_provider(
        css_provider,
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION
    );
    awareness_label->get_style_context()->add_class("overlay-text");
    back_label->get_style_context()->add_provider(
        css_provider,
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION
    );
    back_label->get_style_context()->add_class("overlay-text");

    arenaWindow->show_all();
}


int key = 0x2C;
int checksum_decode(std::list<uint8_t>& byteList){
    //Checks last byte of data for the checksum
    if (byteList.size() < 1) {
        std::cout << "Not enough data to decode checksum." << std::endl;
        return -1;
    }

    // Extracts checksum (last byte)
    auto it = byteList.end();
    std::advance(it, -1);
    uint8_t storedChecksum = *it;

    // Sums byteList, excludes last byte (checksum) 
    uint32_t sum = 0;
    auto dataEnd = byteList.end();
    std::advance(dataEnd, -2);
    //std::cout << "Data: ";
    for (auto dataIt = byteList.begin(); dataIt != dataEnd; ++dataIt) {
        sum += *dataIt;
        //std::cout<<std::hex<<static_cast<int>(*dataIt)<<" ";
        
    }
    //std::cout<<std::endl;

    // Recalculate the checksum as sum modulo key.
    uint8_t computedChecksum = sum % key;

    //std::cout << "Computed checksum from data: 0x" << std::hex << static_cast<int>(computedChecksum) << std::endl;
    //std::cout << "Stored checksum: 0x" << std::hex << static_cast<int>(storedChecksum) << std::endl;

    if (computedChecksum == storedChecksum) {
        //std::cout << "Checksum is valid." << std::endl;
        return 1;
    } else {
        //std::cout << "Checksum is invalid." << std::endl;
        byteList.clear();
        return 0;

    }

}

void print_data(std::list<uint8_t>& byteList){
	auto dataEnd = byteList.end();
	std::advance(dataEnd, -2);
	for (auto dataIt = byteList.begin(); dataIt != dataEnd; ++dataIt){
		std::cout<<std::hex<<static_cast<int>(*dataIt)<<" ";
	}

}


/*
Function to set the various config values. Given a variable name and a 
value, this function gets the correct variable and then sets the value.
*/
void setConfigValues(std::string variableName, std::string value){
    std::cout << "Variable: " << variableName << ", Value: " << value << std::endl;
    if("LIGHT_BACKGROUND" == variableName){
        lightBackgroundColor = value;
    }
    if("DARK_BACKGROUND" == variableName){
        darkBackgroundColor = value;
    }
    if("DISPLAY_SPEED" == variableName){
        if("false" == value){
            displaySpeed = false;
        }
        else{
            displaySpeed = true;
        }
    }
    if("NUMBERS_INSIDE" == variableName){
        if("false" == value){
            numbersInside = false;
        }
        else{
            numbersInside = true;
        }
    }
    if("NUMBER_TICKS" == variableName){
        if("false" == value){
            numberTicks = false;
        }
        else{
            numberTicks = true;
        }
    }
}


/*
Function to parse the config file given by the file name. If the file
doesn't exist, the file check fails and the program uses the default 
values for the config. It splits each line by the = to get the variable
name and value, then sets the values using the setConfigValues function.
*/
void parseConfigFile(std::string filename){
    std::ifstream file(package_share_directory + "/resources/" + filename);

    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            size_t delimiterPos = line.find('=');
            if (delimiterPos != std::string::npos) {
                std::string variableName = line.substr(0, delimiterPos);
                std::string value = line.substr(delimiterPos + 1);
                setConfigValues(variableName, value);
            }
            else {
                std::cerr << "Invalid line (no '='): " << line << std::endl;
            }
        }
        file.close();
    }
    else {
        std::cout << "Unable to open file. Using default configuration" << std::endl;
    }
}


void processArguments(int argc, char** argv){
    if(argc > 1){
        for(int i = 1; i < argc; ++i){
            if(!strcmp("--help", argv[i])){
                std::cout << "Control Flag Options:" << std::endl;
                std::cout << "--init: Initialize GUI with values" << std::endl;
                std::cout << "--no_video: Remove large center space for video stream, Displays sensor values instead" << std::endl;
                std::cout << "--no_arena: Disables arena map window" << std::endl;
                std::cout << "--set_colors: Specifies values to use as background colors. Should have light color, then dark color in" 
                "format \"#FFFFFF\" \"#000000\""<< std::endl;
                std::cout << "NOTE: All strings must be enclosed in \" to have them work properly" << std::endl;
                std::cout << "--set_map: Sets the background map used in the arena" << std::endl;
                std::cout << "--wsl: Sets the video size to a smaller size" << std::endl;
                std::cout << "--config_file: Specifies the config file to be used to load the settings" << std::endl;
                std::cout << "--nano: Switches IP address used to connect to the Jetson Nano" << std::endl;
                std::cout << "--test_input: Allows for testing inputs without being connected to robot" << std::endl;
                std::cout << "--alt_layout: Uses alternate joystick control mapping for robot" << std::endl;
                exit(0);
            }
            else if(!strcmp("--init", argv[i])){
                initVals = true;
            }
            else if(!strcmp("--testing", argv[i])){
                
            }
            else if(!strcmp("--no_video", argv[i])){
                noVideo = true;
            }
            else if(!strcmp("--no_arena", argv[i])){
                noArena = true;
            }
            else if(!strcmp("--set_colors", argv[i])){
                if(i+1 < argc){
                    lightBackgroundColor = argv[i+1];
                    i++;
                }
                if(i+1 < argc){
                    darkBackgroundColor = argv[i+1];
                    i++;
                }
            }
            else if(!strcmp("--set_map", argv[i])){
                mapUsed = argv[i+1];
            }
            else if(!strcmp("--wsl", argv[i])){
                smallLaptop = true;
            }
            else if(!strcmp("--config_file", argv[i])){
                parseConfigFile(argv[i+1]);
                if(allowConfig)
                    create_config_editor_window(argv[i+1]);
                return;
            }
            else if(!strcmp("--nano", argv[i])){
                useOrin = false;
            }
            else if(!strcmp("--test_input", argv[i])){
                testInput = true;
            }
            else if(!strcmp("--alt_layout", argv[i])){
                useAltLayout = true;
            }
        }
    }
}


/* Function to check whether the old laptop is running the control program.
Because the old laptop has a smaller screen, the size of the window should be smaller.*/
void checkSize(){
    auto display = Gdk::Display::get_default();
    auto primary_monitor = display->get_monitor(0);
    if (primary_monitor) {
        Gdk::Rectangle geometry;
        primary_monitor->get_geometry(geometry);
        int x = geometry.get_x();
        int y = geometry.get_y();
        int width = geometry.get_width();
        int height = geometry.get_height();
        std::cout << "Height: " << height << std::endl << "Width: " << width << std::endl;
        if(width < 1920){
            smallLaptop = true;
        }
    }
}

void moveWindows(){
    auto display = Gdk::Display::get_default();
    monitor_count = display->get_n_monitors();
    if(monitor_count == 1){
        auto primary_monitor = display->get_monitor(0);
        Gdk::Rectangle primary_monitor_geometry;
        primary_monitor->get_geometry(primary_monitor_geometry);
        window->move(primary_monitor_geometry.get_x(), primary_monitor_geometry.get_y());
        window->show();
        window->raise();
    }
    
}


void remapJoystickInputs(uint8_t* which, uint8_t* axis){
    // Expected values are as follows:
    // Joystick 0:
    // Axis 0 - Roll
    // Axis 1 - Pitch
    // Joystick 1:
    // Axis 0 - Bucket
    // Axis 1 - Arm
    if(isController){
        // If a controller is used, axes 0 and 1 should be mapped to joystick 0
        // Axes 2 and 3 should be mapped to joystick 1
        if(*axis == 2){
            *which = 1;
            *axis = 0;
        }
        if(*axis == 3){
            *which = 1;
            *axis = 1;
        }
    }
    if(useAltLayout){
        // Alt layout is as follows:
        // Joystick 0:
        // Axis 0 - Left Speed
        // Axis 1 - Arm
        // Joystick 1:
        // Axis 0 - Right Speed
        // Axis 1 - Bucket
        // Note: This probably isn't going to respond as expected. The speed calculations aren't meant
        // to have individual speed components like this
        if(*which == 0){
            if(*axis == 1){
                *which = 1;
                *axis = 1;
            }
        }
        if(*which == 1){
            if(*axis == 0){
                *which = 0;
                *axis = 0;
            }
        }
    }
    if(!twoJoysticks){
        // If a single joystick is used, control the bucket speed with twist of axis 2
        // Buttons 
        if(*axis == 2){
            *which = 1;
            *axis = 0;
        }
    }
}


//UDP Version
int main(int argc, char** argv) { 
    rclcpp::init(argc, argv);
    package_share_directory = ament_index_cpp::get_package_share_directory("control_gui");
    //Setup GUI
    Glib::RefPtr<Gtk::Application> application = Gtk::Application::create(argc, argv, "edu.uark.razorbotz");
    processArguments(argc, argv);
    checkSize();
    setupGUI(application);
    if(!noArena)
        initArenaWindow();
    if(!noVideo)
        initSensorsWindow();
    moveWindows();
    initGUI();
    
    //Start a thread to listen to updates from the robot
    videoDisconnectDispatcher.connect([&]() {
        if (shouldVideoDisconnect) {
            handleVideoDisconnect(video_server_ui);
            shouldVideoDisconnect = false;
        }
    });

    connection_finished_dispatcher.connect(sigc::ptr_fun(&on_connection_finished));
    video_connection_finished_dispatcher.connect(sigc::ptr_fun(&on_video_connection_finished));
    
    std::thread broadcastListenThread(broadcastListen);
    broadcastListenThread.detach();

    auto image_subscriber_node = std::make_shared<ImageSubscriber>();

    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) != 0) {
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
        return 1;
    }
    SDL_JoystickEventState(SDL_ENABLE);

    //-------------------------------------------------------------------------Initializing joystick(s)--------------------------------------------------------------------------
    int joystickCount=SDL_NumJoysticks();
    std::cout << "number of joysticks " << joystickCount << std::endl;
    if(joystickCount == 2){
        twoJoysticks = true;
    }
    SDL_Joystick* joystickList[joystickCount];
    SDL_GameController* controller = nullptr;

    if(joystickCount>0){
        axisEventList = new std::vector<std::vector<AxisEvent*>*>(joystickCount);
        for(int joystickIndex=0;joystickIndex<joystickCount;joystickIndex++) {

            if(SDL_IsGameController(joystickIndex)){
                isController = true;
                controller = SDL_GameControllerOpen(joystickIndex);
                if(controller){
                    std::cout << "Opened controller: " << SDL_GameControllerName(controller) << std::endl;
                    joystickList[joystickIndex]=SDL_GameControllerGetJoystick(controller);
                }
            }
            else{
                joystickList[joystickIndex]=SDL_JoystickOpen(joystickIndex);
            }
            if (joystickList[joystickIndex]) {
                axisEventList->at(joystickIndex) = new std::vector<AxisEvent*>(SDL_JoystickNumAxes(joystickList[joystickIndex]));
                for(int axisIndex=0; axisIndex < SDL_JoystickNumAxes(joystickList[joystickIndex]); axisIndex++){
                    axisEventList->at(joystickIndex)->at(axisIndex) = new AxisEvent();
                }
                std::cout << "Opened Joystick " << joystickIndex << std::endl;
                std::cout << "   Name: " << SDL_JoystickName(joystickList[joystickIndex]) << std::endl;
                std::cout << "   Number of Axes: " << SDL_JoystickNumAxes(joystickList[joystickIndex]) << std::endl;
                std::cout << "   Number of Buttons: " << SDL_JoystickNumButtons(joystickList[joystickIndex]) << std::endl;
                std::cout << "   Number of Balls: " << SDL_JoystickNumBalls(joystickList[joystickIndex]) << std::endl;
            }
            else {
                (*axisEventList)[joystickIndex] = new std::vector<AxisEvent*>(0);
                std::cout << "Couldn't open Joystick " << joystickIndex << std::endl;
            }
        }
    }

    SDL_Event event;
    char buffer[16384] = {0}; 
    int bytesRead=0;

    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point lastTransmitTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point lastReceiveTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point lastHeartbeatTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point lastVideoHeartbeatTime = std::chrono::high_resolution_clock::now();
    now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastTransmitTime);
    double deltaTime = time_span.count();
    
    std::list<uint8_t> messageBytesList; //List to store incoming bytes
    uint8_t message[256];
    bool running=true;
    while(running){
        rclcpp::spin_some(image_subscriber_node);

        adjustRobotList(addressListBox);
        adjustVideoRobotList(videoAddressListBox);

        while(Gtk::Main::events_pending()){
            Gtk::Main::iteration();
        }

        if (newFrameAvailable) {
            if (videoArea) {
                videoArea->setFrame(latestFrame);
            }
            newFrameAvailable = false;
        }

        if(!testInput && !isServerInitialized()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Don't busy-wait
            continue;
        }

        std::vector<uint8_t> data_buffer;
        bytesRead = receiveRobotData(data_buffer);
        if(isSilentRunning())
            lastReceiveTime = std::chrono::high_resolution_clock::now();
        if (bytesRead > 0) {
            std::vector<uint8_t> processed_buffer;
            if (process_payload(data_buffer, processed_buffer)) { 
                for(uint8_t byte : processed_buffer) {
                    messageBytesList.push_back(byte);
                }
                lastReceiveTime = std::chrono::high_resolution_clock::now();
            }
        }
        else if (bytesRead < 0 && isServerConnected()) {
            now = std::chrono::high_resolution_clock::now();
            time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastReceiveTime);
            deltaTime = time_span.count();
            if(deltaTime > 5.0){
                std::cout << "Connection timed out." << std::endl;
                setDisconnectedState(server_ui);
                resetUIOnDisconnect();
            }
        }
        
        while(BinaryMessage::hasMessage(messageBytesList)){
            if (checksum_decode(messageBytesList) == 1) {
                BinaryMessage message(messageBytesList);
                updateGUI(message);
                uint64_t size = BinaryMessage::decodeSizeBytes(messageBytesList);
                for(int count=0; count < size + 1; count++){
                    messageBytesList.pop_front();
                }
            }
            else {
                break; 
            }
        }

        now = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastHeartbeatTime);
        deltaTime = time_span.count();
        if(deltaTime > 1.0 && isServerConnected()){
            lastHeartbeatTime = now;
            sendHeartbeat();
        }

        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastVideoHeartbeatTime);
        if (time_span.count() > 1.0 && isVideoConnected()) {
            lastVideoHeartbeatTime = now;
            sendVideoHeartbeat();
        }


        /******************************Handle control events******************************/
        while(SDL_PollEvent(&event)){
            switch(event.type){
                case SDL_JOYHATMOTION:{
                    sendJoystickHat(event.jhat.which, event.jhat.hat, event.jhat.value);
                    break;
                }
                case SDL_JOYBUTTONDOWN:{
                    sendJoystickButton(event.jbutton.which, event.jbutton.button, event.jbutton.state);
                    break;
                }
                case SDL_JOYBUTTONUP:{
                    sendJoystickButton(event.jbutton.which, event.jbutton.button, event.jbutton.state);
                    break;
                }
                case SDL_JOYAXISMOTION: {
                    int deadZone=4000;
                    if(event.jaxis.value < -deadZone || deadZone < event.jaxis.value ) {
                        axisEventList->at(event.jaxis.which)->at(event.jaxis.axis)->isSet = true;
                        axisEventList->at(event.jaxis.which)->at(event.jaxis.axis)->value = event.jaxis.value;
                    }
                    else{
                        axisEventList->at(event.jaxis.which)->at(event.jaxis.axis)->isSet = true;
                        axisEventList->at(event.jaxis.which)->at(event.jaxis.axis)->value = 0;
                    }
                    break;
                }
                default:
                    break;
            }
        }

        now = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastTransmitTime);
        deltaTime = time_span.count();
        if(deltaTime > 0.05 ){
            lastTransmitTime = std::chrono::high_resolution_clock::now();
            for(int joystickIndex=0; joystickIndex < axisEventList->size(); joystickIndex++){
                for(int axisIndex=0; axisIndex < axisEventList->at(joystickIndex)->size(); axisIndex++){
                    if(axisEventList->at(joystickIndex)->at(axisIndex)->isSet){
                        axisEventList->at(joystickIndex)->at(axisIndex)->isSet = false;
                        float value = ((float)axisEventList->at(joystickIndex)->at(axisIndex)->value) / -32768.0;
                        uint8_t which = joystickIndex;
                        uint8_t axis  = axisIndex;
                        remapJoystickInputs(&which, &axis);
                        sendJoystickAxis(which, axis, value);
                    }
                }
            }
        }
    }
    rclcpp::shutdown();
    return 0; 
}
