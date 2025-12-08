#ifndef NETWORK_HANDLER_HPP
#define NETWORK_HANDLER_HPP

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>

// Forward declare GTK types to avoid including heavy headers
namespace Gtk {
    class Button;
    class Label;
    class Entry;
    class ListBox;
    class ListBoxRow;
    class Window;
}

namespace Glib {
    class Dispatcher;
}

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

// Struct to hold pointers to the UI elements the server functions need
struct ServerUI {
    Gtk::Button* connectButton;
    Gtk::Label* connectionStatusLabel;
    Gtk::Button* silentRunButton;
    Gtk::Entry* ipAddressEntry;
    Gtk::ListBox* addressListBox;
    Gtk::Window* parentWindow;
};

// Struct for video server UI elements
struct VideoServerUI {
    Gtk::Button* connectButton;
    Gtk::Label* connectionStatusLabel;
    Gtk::Button* streamButton;
    Gtk::Entry* ipAddressEntry;
    Gtk::ListBox* addressListBox;
    Gtk::Window* parentWindow;
};

// --- Function Declarations for Main Robot Server ---
void connectOrDisconnect(ServerUI& ui, bool useOrin, Glib::Dispatcher& dispatcher);
void silentRun(ServerUI& ui);
void rowActivated(Gtk::ListBoxRow* listBoxRow, ServerUI& ui);
void shutdownDialog(Gtk::Window* parentWindow);
void broadcastListen();
void adjustRobotList(Gtk::ListBox* addressListBox);
void sendHeartbeat();
void sendVideoHeartbeat();
void sendJoystickAxis(uint8_t which, uint8_t axis, float value);
void sendJoystickButton(uint8_t which, uint8_t button, uint8_t state);
void sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value);
void sendKeyboardEvent(uint32_t keyval, uint8_t state);
int receiveRobotData(std::vector<uint8_t>& buffer);
void setDisconnectedState(ServerUI& ui);
void update_connection_status(ServerUI& ui);

// --- Function Declarations for Video Server ---
extern struct sockaddr_in video_serv_addr;
extern socklen_t video_addr_len;
void videoConnectOrDisconnect(VideoServerUI& ui, Glib::Dispatcher& dispatcher);
void videoStream(VideoServerUI& ui);
void videoRowActivated(Gtk::ListBoxRow* listBoxRow, VideoServerUI& ui);
void videoBroadcastListen();
void adjustVideoRobotList(Gtk::ListBox* videoAddressListBox);
void videoMain(cv::Mat& latestFrame, std::mutex& frameMutex, std::atomic<bool>& newFrameAvailable, Glib::Dispatcher& videoDisconnectDispatcher, std::atomic<bool>& shouldVideoDisconnect);
void handleVideoDisconnect(VideoServerUI& ui);
void update_video_connection_status(VideoServerUI& ui);

// --- State Accessor Functions ---
bool isServerConnected();
bool isServerInitialized();
bool isSilentRunning();
bool isVideoStreamActive();
bool isVideoConnected();

#endif