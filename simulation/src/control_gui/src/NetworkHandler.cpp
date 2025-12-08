#include "NetworkHandler.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <list>

// GTK and System includes
#include <gtkmm.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>


// --- Main Robot Server Globals & Implementation ---

int sock = 0;
bool connected = false;
bool silentRunning = true;
bool initialized = false;

struct sockaddr_in serv_addr;
socklen_t addr_len = sizeof(serv_addr);
std::chrono::high_resolution_clock::time_point lastHeartbeatTime;

struct RemoteRobot {
    std::string tag;
    time_t lastSeenTime;
};
std::vector<RemoteRobot> robotList;
std::mutex robotListMutex;

#define PORT 31337
#define ORIN_IP "192.168.1.6"
#define NANO_IP "192.168.1.5"

// State Accessors
bool isServerConnected() { return connected; }
bool isServerInitialized() { return initialized; }
bool isSilentRunning() { return silentRunning; }

void setDisconnectedState(ServerUI& ui) {
    ui.connectButton->set_label("Connect");
    ui.connectionStatusLabel->set_text("Not Connected");
    ui.silentRunButton->set_label("Silent Running");
    Gdk::RGBA red;
    red.set_rgba(1.0, 0, 0, 1.0);
    ui.connectionStatusLabel->override_background_color(red);
    ui.ipAddressEntry->set_can_focus(true);
    ui.ipAddressEntry->set_editable(true);
    
    if (connected) {
        if (sock > 0) {
            close(sock);
            sock = 0;
        }
    }
    connected = false;
    silentRunning = true;
    initialized = false;
}

void setConnectedState(ServerUI& ui) {
    ui.connectButton->set_label("Disconnect");
    ui.connectionStatusLabel->set_text("Connected");
    Gdk::RGBA green;
    green.set_rgba(0, 1.0, 0, 1.0);
    ui.connectionStatusLabel->override_background_color(green);
    ui.ipAddressEntry->set_can_focus(false);
    ui.ipAddressEntry->set_editable(false);
    connected = true;
}

void disconnectFromServer(ServerUI& ui) {
    Gtk::MessageDialog dialog(*ui.parentWindow, "Disconnect now?", false, Gtk::MESSAGE_QUESTION, Gtk::BUTTONS_OK_CANCEL);
    if (dialog.run() == Gtk::RESPONSE_OK) {
        if (close(sock) == 0) {
            setDisconnectedState(ui);
        }
        else {
            Gtk::MessageDialog errDialog(*ui.parentWindow, "Failed Close", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            errDialog.run();
        }
    }
}

enum class ConnStatus { PENDING, SUCCESS, FAILURE };
std::atomic<ConnStatus> connection_status = ConnStatus::PENDING;

void connectToServer(ServerUI& ui, bool useOrin, Glib::Dispatcher& dispatcher) {
    if (connected) return;

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    const char* ip = useOrin ? ORIN_IP : NANO_IP;
    // Use the IP from the entry box for user flexibility
    if (inet_pton(AF_INET, ui.ipAddressEntry->get_text().c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid IP Address" << std::endl;
        return;
    }

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation error");
        return;
    }
    fcntl(sock, F_SETFL, O_NONBLOCK);
    
    std::string hello("Hello Robot");
    sendto(sock, hello.c_str(), hello.length(), 0, (struct sockaddr *)&serv_addr, addr_len);

    auto startTime = std::chrono::steady_clock::now();
    char buffer[1024];
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count() < 2) {
        if (recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, &addr_len) > 0) {
            std::cout << "Received reply from server. Connection established." << std::endl;
            connection_status = ConnStatus::SUCCESS;
            dispatcher.emit();
            lastHeartbeatTime = std::chrono::high_resolution_clock::now();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Connection to server failed (timeout)." << std::endl;
    connection_status = ConnStatus::FAILURE;
    dispatcher.emit();
}

void update_connection_status(ServerUI& ui) {
    ConnStatus status = connection_status;
    if (status == ConnStatus::SUCCESS) {
        setConnectedState(ui);
        initialized = true;
        lastHeartbeatTime = std::chrono::high_resolution_clock::now();
    }
    else if (status == ConnStatus::FAILURE) {
        setDisconnectedState(ui);
    }
    ui.connectButton->set_sensitive(true);
}


void connectOrDisconnect(ServerUI& ui, bool useOrin, Glib::Dispatcher& dispatcher) {
     if (ui.connectButton->get_label() == "Connect") {
        ui.connectButton->set_sensitive(false);
        ui.connectionStatusLabel->set_text("Connecting...");
        
        std::thread conn_thread(connectToServer, std::ref(ui), useOrin, std::ref(dispatcher));
        conn_thread.detach();
    }
    else {
        disconnectFromServer(ui);
    }
}

namespace {
    bool contains(const std::vector<std::string>& list, const std::string& value) {
        for (const std::string& storedValue : list) {
            if (storedValue == value) return true;
        }
        return false;
    }

    std::vector<std::string> getAddressList() {
        std::vector<std::string> addressList;
        ifaddrs* interfaceAddresses = nullptr;
        if (getifaddrs(&interfaceAddresses) == 0) {
            for (ifaddrs* interface = interfaceAddresses; interface != nullptr; interface = interface->ifa_next) {
                if (interface->ifa_addr != nullptr && interface->ifa_addr->sa_family == AF_INET) {
                    sockaddr_in* socketAddress = reinterpret_cast<sockaddr_in*>(interface->ifa_addr);
                    std::string addressString(inet_ntoa(socketAddress->sin_addr));
                    if (addressString != "0.0.0.0" && addressString != "127.0.0.1" && !contains(addressList, addressString)) {
                        addressList.push_back(addressString);
                    }
                }
            }
            freeifaddrs(interfaceAddresses);
        }
        return addressList;
    }
}

void silentRun(ServerUI& ui) {
    if (!connected) return;
    
    std::string currentButtonState = ui.silentRunButton->get_label();
    uint8_t message[3];
    message[0] = 3;  // messageSize
    message[1] = 7;  // command (silence)

    if (currentButtonState == "Silent Running") {
        message[2] = 0; // Not silent
        sendto(sock, message, sizeof(message), 0, (struct sockaddr *)&serv_addr, addr_len);
        ui.silentRunButton->set_label("Not Silent Running");
        silentRunning = false;
    } else {
        message[2] = 1; // Silent
        sendto(sock, message, sizeof(message), 0, (struct sockaddr *)&serv_addr, addr_len);
        ui.silentRunButton->set_label("Silent Running");
        silentRunning = true;
    }
}

void rowActivated(Gtk::ListBoxRow* listBoxRow, ServerUI& ui) {
    auto label = static_cast<Gtk::Label*>(listBoxRow->get_child());
    Glib::ustring connectionString(label->get_text());
    
    size_t index = connectionString.rfind('@');
    if (index == Glib::ustring::npos) return;

    Glib::ustring addressString = connectionString.substr(index + 1);
    ui.ipAddressEntry->set_text(addressString);
}

static void shutdownRobot() {
    uint8_t message[2];
    message[0] = 2; // messageSize
    message[1] = 8; // command (shutdown)
    sendto(sock, message, sizeof(message), 0, (struct sockaddr *)&serv_addr, addr_len);
}

void shutdownDialog(Gtk::Window* parentWindow) {
    Gtk::MessageDialog dialog(*parentWindow, "Shutdown now?", false, Gtk::MESSAGE_QUESTION, Gtk::BUTTONS_OK_CANCEL);
    if (dialog.run() == Gtk::RESPONSE_OK) {
        shutdownRobot();
    }
}

void broadcastListen() {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) {
        perror("Opening datagram socket error");
        return;
    }

    int reuse = 1;
    if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse)) < 0) {
        perror("Setting SO_REUSEADDR error");
        close(sd);
        return;
    }

    struct sockaddr_in localSock;
    localSock.sin_family = AF_INET;
    localSock.sin_port = htons(4321);
    localSock.sin_addr.s_addr = INADDR_ANY;
    if (bind(sd, (struct sockaddr*)&localSock, sizeof(localSock))) {
        perror("Binding datagram socket error");
        close(sd);
        return;
    }

    std::vector<std::string> addressList = getAddressList();
    for (const std::string& addressString : addressList) {
        struct ip_mreq group;
        group.imr_multiaddr.s_addr = inet_addr("226.1.1.1");
        group.imr_interface.s_addr = inet_addr(addressString.c_str());
        if (setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0) {
            perror("Adding multicast group error");
        }
    }

    char databuf[2048];
    while (true) {
        ssize_t bytesRead = read(sd, databuf, sizeof(databuf));
        if (bytesRead > 0) {
            std::string message(databuf, bytesRead);
            std::lock_guard<std::mutex> lock(robotListMutex);

            bool robotExists = false;
            for (auto& robot : robotList) {
                if (robot.tag == message) {
                    time(&robot.lastSeenTime);
                    robotExists = true;
                    break;
                }
            }

            if (!robotExists) {
                robotList.push_back({message, time(nullptr)});
            }
        }
    }
}

void adjustRobotList(Gtk::ListBox* addressListBox) {
    std::lock_guard<std::mutex> lock(robotListMutex);
    time_t now = time(nullptr);
    std::vector<std::string> robots_in_gui;
    
    // Build a list of robots currently in the GUI
    for (auto* child : addressListBox->get_children()) {
        if (auto* row = dynamic_cast<Gtk::ListBoxRow*>(child)) {
            auto* label = static_cast<Gtk::Label*>(row->get_child());
            robots_in_gui.push_back(label->get_text());
        }
    }

    // Remove stale robots from the GUI and the data list
    robotList.erase(std::remove_if(robotList.begin(), robotList.end(),
        [&](const RemoteRobot& robot) {
            if (now - robot.lastSeenTime > 12) {
                // Find and remove the corresponding row from the ListBox
                for (auto* child : addressListBox->get_children()) {
                     if (auto* row = dynamic_cast<Gtk::ListBoxRow*>(child)) {
                        auto* label = static_cast<Gtk::Label*>(row->get_child());
                        if (label->get_text() == robot.tag) {
                            addressListBox->remove(*row);
                            break;
                        }
                    }
                }
                return true; // Remove from robotList
            }
            return false;
        }),
        robotList.end());

    // Add new robots to the GUI
    for (const auto& robot : robotList) {
        if (std::find(robots_in_gui.begin(), robots_in_gui.end(), robot.tag) == robots_in_gui.end()) {
            addressListBox->append(*Gtk::manage(new Gtk::Label(robot.tag)));
        }
    }
    addressListBox->show_all();
}


// --- Video Server Globals & Implementation ---

int videoSock = 0;
bool videoConnected = false;
bool isStreamingActive = false;
struct sockaddr_in video_serv_addr;
socklen_t video_addr_len = sizeof(video_serv_addr);
std::chrono::steady_clock::time_point last_packet_time;

std::vector<RemoteRobot> videoRobotList;
std::mutex videoRobotListMutex;

#define VIDEO_PORT 31338

// State Accessors
bool isVideoConnected() { return videoConnected; }
bool isVideoStreamActive() { return isStreamingActive; }

// --- State Update Functions ---
void setVideoConnectedState(VideoServerUI& ui) {
    ui.connectButton->set_label("Disconnect");
    ui.connectionStatusLabel->set_text("Connected");
    Gdk::RGBA green;
    green.set_rgba(0, 1.0, 0, 1.0);
    ui.connectionStatusLabel->override_background_color(green);
    ui.ipAddressEntry->set_can_focus(false);
    ui.ipAddressEntry->set_editable(false);
    videoConnected = true;
}

void setVideoDisconnectedState(VideoServerUI& ui) {
    ui.connectButton->set_label("Connect");
    ui.connectionStatusLabel->set_text("Not Connected");
    ui.streamButton->set_label("Not Video Streaming");
    Gdk::RGBA red;
    red.set_rgba(1.0, 0, 0, 1.0);
    ui.connectionStatusLabel->override_background_color(red);
    ui.ipAddressEntry->set_can_focus(true);
    ui.ipAddressEntry->set_editable(true);
    videoConnected = false;
    isStreamingActive = false;
}

void handleVideoDisconnect(VideoServerUI& ui) {
    setVideoDisconnectedState(ui);
}

std::atomic<ConnStatus> video_connection_status = ConnStatus::PENDING;

// --- Connection Logic ---
static void connectToVideoServer(VideoServerUI& ui, Glib::Dispatcher& dispatcher) {
    if (videoConnected) return;

    memset(&video_serv_addr, 0, sizeof(video_serv_addr));
    video_serv_addr.sin_family = AF_INET;
    video_serv_addr.sin_port = htons(VIDEO_PORT);

    if ((videoSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("\n Video UDP socket creation error \n");
        video_connection_status = ConnStatus::FAILURE;
        dispatcher.emit();
        return;
    }

    if (inet_pton(AF_INET, ui.ipAddressEntry->get_text().c_str(), &video_serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid Video IP Address" << std::endl;
        video_connection_status = ConnStatus::FAILURE;
        dispatcher.emit();
        return;
    }

    std::string hello("Hello Robot");
    sendto(videoSock, hello.c_str(), hello.length(), 0, (struct sockaddr *)&video_serv_addr, video_addr_len);

    fcntl(videoSock, F_SETFL, O_NONBLOCK);
    video_connection_status = ConnStatus::SUCCESS;
    dispatcher.emit();
}

void update_video_connection_status(VideoServerUI& ui) {
    ConnStatus status = video_connection_status;
    if (status == ConnStatus::SUCCESS) {
        setVideoConnectedState(ui);
    }
    else if (status == ConnStatus::FAILURE) {
        setVideoDisconnectedState(ui);
    }
    ui.connectButton->set_sensitive(true);
}


static void disconnectFromVideoServer(VideoServerUI& ui) {
    Gtk::MessageDialog dialog(*ui.parentWindow, "Disconnect from video server?", false, Gtk::MESSAGE_QUESTION, Gtk::BUTTONS_OK_CANCEL);
    if (dialog.run() == Gtk::RESPONSE_OK) {
        if (close(videoSock) == 0) {
            setVideoDisconnectedState(ui);
        }
        else {
            Gtk::MessageDialog errDialog(*ui.parentWindow, "Failed to close video socket", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            errDialog.run();
        }
    }
}

void videoConnectOrDisconnect(VideoServerUI& ui, Glib::Dispatcher& dispatcher) {
    if (ui.connectButton->get_label() == "Connect") {
        ui.connectButton->set_sensitive(false);
        ui.connectionStatusLabel->set_text("Connecting...");

        std::thread conn_thread(connectToVideoServer, std::ref(ui), std::ref(dispatcher));
        conn_thread.detach();
    }
    else {
        disconnectFromVideoServer(ui);
    }
}

// --- UI Interaction Functions ---
void videoStream(VideoServerUI& ui) {
    if (!videoConnected) return;
    
    uint8_t message[3];
    message[0] = 3; 
    message[1] = 1;

    if (ui.streamButton->get_label() == "Not Video Streaming") {
        message[2] = 1;
        ui.streamButton->set_label("Video Streaming");
        isStreamingActive = true;
        last_packet_time = std::chrono::steady_clock::now();
    }
    else {
        message[2] = 0;
        ui.streamButton->set_label("Not Video Streaming");
        isStreamingActive = false;
    }
    sendto(videoSock, message, sizeof(message), 0, (struct sockaddr *)&video_serv_addr, video_addr_len);
}

void videoRowActivated(Gtk::ListBoxRow* listBoxRow, VideoServerUI& ui) {
    auto label = static_cast<Gtk::Label*>(listBoxRow->get_child());
    Glib::ustring connectionString(label->get_text());
    
    size_t index = connectionString.rfind('@');
    if (index == Glib::ustring::npos) return;

    Glib::ustring addressString = connectionString.substr(index + 1);
    ui.ipAddressEntry->set_text(addressString);
}


// --- Background Threads ---

void videoBroadcastListen() {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) {
        perror("Opening video datagram socket error");
        return;
    }

    int reuse = 1;
    if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse)) < 0) {
        perror("Setting SO_REUSEADDR for video error");
        close(sd);
        return;
    }

    /* Bind to the proper port number with the IP address */
    /* specified as INADDR_ANY. */
    struct sockaddr_in localSock;
    localSock.sin_family = AF_INET;
    localSock.sin_port = htons(4322); // Video broadcast port
    localSock.sin_addr.s_addr = INADDR_ANY;
    if (bind(sd, (struct sockaddr*)&localSock, sizeof(localSock))) {
        perror("Binding video datagram socket error");
        close(sd);
        return;
    }

    /* Join the multicast group 226.1.1.1 on the local 203.106.93.94 */
    /* interface. Note that this IP_ADD_MEMBERSHIP option must be */
    /* called for each local interface over which the multicast */
    /* datagrams are to be received. */
    std::vector<std::string> addressList = getAddressList(); 
    for (const std::string& addressString : addressList) {
        struct ip_mreq group;
        group.imr_multiaddr.s_addr = inet_addr("226.1.1.1");
        group.imr_interface.s_addr = inet_addr(addressString.c_str());
        if (setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0) {
            perror("Adding multicast group for video error");
        }
    }

    char databuf[1024];
    while (true) {
        ssize_t bytesRead = read(sd, databuf, sizeof(databuf));
        if (bytesRead > 0) {
            std::string message(databuf, bytesRead);
            std::lock_guard<std::mutex> lock(videoRobotListMutex);

            bool robotExists = false;
            for (auto& robot : videoRobotList) {
                if (robot.tag == message) {
                    time(&robot.lastSeenTime);
                    robotExists = true;
                    break;
                }
            }
            if (!robotExists) {
                videoRobotList.push_back({message, time(nullptr)});
            }
        }
    }
}

void adjustVideoRobotList(Gtk::ListBox* videoAddressListBox) {
    if (!videoAddressListBox) return;
    std::lock_guard<std::mutex> lock(videoRobotListMutex);
    time_t now = time(nullptr);
    std::vector<std::string> robots_in_gui;
    
    for (auto* child : videoAddressListBox->get_children()) {
        if (auto* row = dynamic_cast<Gtk::ListBoxRow*>(child)) {
            robots_in_gui.push_back(static_cast<Gtk::Label*>(row->get_child())->get_text());
        }
    }

    videoRobotList.erase(std::remove_if(videoRobotList.begin(), videoRobotList.end(),
        [&](const RemoteRobot& robot) {
            if (now - robot.lastSeenTime > 12) {
                for (auto* child : videoAddressListBox->get_children()) {
                     if (auto* row = dynamic_cast<Gtk::ListBoxRow*>(child)) {
                        if (static_cast<Gtk::Label*>(row->get_child())->get_text() == robot.tag) {
                            videoAddressListBox->remove(*row);
                            break;
                        }
                    }
                }
                return true; 
            }
            return false;
        }),
        videoRobotList.end());

    for (const auto& robot : videoRobotList) {
        if (std::find(robots_in_gui.begin(), robots_in_gui.end(), robot.tag) == robots_in_gui.end()) {
            videoAddressListBox->append(*Gtk::manage(new Gtk::Label(robot.tag)));
        }
    }
    videoAddressListBox->show_all();
}

struct FrameChunkHeader {
    uint16_t frame_id;
    uint16_t chunk_index;
    uint16_t total_chunks;
} __attribute__((packed));

std::unordered_map<uint16_t, std::vector<std::vector<uint8_t>>> frameChunks;
std::unordered_map<uint16_t, size_t> frameSizes;
uint16_t lastFrameID = 0;

void videoMain(cv::Mat& latestFrame, std::mutex& frameMutex, std::atomic<bool>& newFrameAvailable, Glib::Dispatcher& videoDisconnectDispatcher, std::atomic<bool>& shouldVideoDisconnect) {
    // --- FFmpeg Decoder Initialization ---
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!codec) {
        std::cerr << "H.265 (HEVC) decoder not found" << std::endl;
        return;
    }

    AVCodecParserContext* parser = av_parser_init(codec->id);
    if (!parser) {
        std::cerr << "Failed to initialize H.265 parser" << std::endl;
        return;
    }

    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "Failed to allocate codec context" << std::endl;
        av_parser_close(parser);
        return;
    }

    if (avcodec_open2(codec_ctx, codec, NULL) < 0) {
        std::cerr << "Failed to open codec" << std::endl;
        avcodec_free_context(&codec_ctx);
        av_parser_close(parser);
        return;
    }

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* bgr_frame = av_frame_alloc();
    SwsContext* sws_ctx = nullptr;
    uint8_t* bgr_buffer = nullptr;

    std::vector<uint8_t> frameDataBuffer(1000000); // 1 MB buffer should be safe

    bool running = true;
    while (running) {
        if (!videoConnected || !isStreamingActive) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_packet_time).count() >= 1) {
            std::cerr << "Video stream timed out." << std::endl;
            isStreamingActive = false;
            shouldVideoDisconnect = true;
            videoDisconnectDispatcher.emit();
            continue;
        }

        ssize_t bytesRead = recvfrom(videoSock, frameDataBuffer.data(), frameDataBuffer.size(), 0, NULL, NULL);
        if (bytesRead < (ssize_t)sizeof(FrameChunkHeader))
            continue;

        last_packet_time = std::chrono::steady_clock::now();

        FrameChunkHeader hdr;
        memcpy(&hdr, frameDataBuffer.data(), sizeof(hdr));
        hdr.frame_id = ntohs(hdr.frame_id);
        hdr.chunk_index = ntohs(hdr.chunk_index);
        hdr.total_chunks = ntohs(hdr.total_chunks);

        std::vector<uint8_t> chunk(frameDataBuffer.begin() + sizeof(hdr),
                                frameDataBuffer.begin() + bytesRead);

        // Store chunk
        frameChunks[hdr.frame_id].resize(hdr.total_chunks);
        frameChunks[hdr.frame_id][hdr.chunk_index] = std::move(chunk);

        // Check if we have all chunks for this frame
        bool complete = true;
        for (size_t i = 0; i < hdr.total_chunks; ++i) {
            if (frameChunks[hdr.frame_id][i].empty()) {
                complete = false;
                break;
            }
        }

        if (complete) {
            // Combine chunks into single H.265 frame
            std::vector<uint8_t> fullFrame;
            for (auto &chunk : frameChunks[hdr.frame_id])
                fullFrame.insert(fullFrame.end(), chunk.begin(), chunk.end());

            frameChunks.erase(hdr.frame_id);

            // Decode as before
            uint8_t* data_ptr = fullFrame.data();
            size_t data_size = fullFrame.size();

            while (data_size > 0) {
                int ret = av_parser_parse2(parser, codec_ctx, &pkt->data, &pkt->size,
                                        data_ptr, data_size,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
                if (ret < 0) break;
                data_ptr += ret;
                data_size -= ret;

                if (pkt->size && avcodec_send_packet(codec_ctx, pkt) >= 0) {
                    while (avcodec_receive_frame(codec_ctx, frame) == 0) {
                        // Got a decoded frame, now convert it to BGR for OpenCV
                        
                        // Initialize SWS context for color conversion on first frame
                        if (!sws_ctx) {
                            sws_ctx = sws_getContext(codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt,
                                                     codec_ctx->width, codec_ctx->height, AV_PIX_FMT_GRAY8,
                                                     SWS_BILINEAR, NULL, NULL, NULL);
                            int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_GRAY8, codec_ctx->width, codec_ctx->height, 32);
                            bgr_buffer = (uint8_t*)av_malloc(num_bytes * sizeof(uint8_t));
                            av_image_fill_arrays(bgr_frame->data, bgr_frame->linesize, bgr_buffer, AV_PIX_FMT_GRAY8, codec_ctx->width, codec_ctx->height, 32);
                        }

                        // Perform color conversion (e.g., YUV to BGR)
                        sws_scale(sws_ctx, (uint8_t const * const *)frame->data, frame->linesize, 0, codec_ctx->height,
                                  bgr_frame->data, bgr_frame->linesize);

                        // Create an OpenCV Mat from the BGR data
                        cv::Mat decoded_mat(codec_ctx->height, codec_ctx->width, CV_8UC1, bgr_frame->data[0], bgr_frame->linesize[0]);

                        // Resize and update the GUI
                        cv::Mat display_img;
                        cv::resize(decoded_mat, display_img, cv::Size(1600, 1000), 0, 0, cv::INTER_LINEAR);
                        
                        {
                            std::lock_guard<std::mutex> lock(frameMutex);
                            latestFrame = display_img.clone(); // Clone is crucial for thread safety
                            newFrameAvailable = true;
                        }
                    }
                }
            }
            av_packet_unref(pkt);
        }
    }

    // --- Cleanup ---
    if (sws_ctx) sws_freeContext(sws_ctx);
    if (bgr_buffer) av_freep(&bgr_buffer);
    av_frame_free(&bgr_frame);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    avcodec_free_context(&codec_ctx);
    av_parser_close(parser);
}


// --- Functions to send data (Example for Joystick) ---
void insert_float(float value, uint8_t* array) {
    uint32_t as_int = *reinterpret_cast<uint32_t*>(&value);
    array[0] = (as_int >> 24) & 0xff;
    array[1] = (as_int >> 16) & 0xff;
    array[2] = (as_int >> 8) & 0xff;
    array[3] = (as_int >> 0) & 0xff;
}

void sendJoystickAxis(uint8_t which, uint8_t axis, float value) {
    if (!connected) return;
    uint8_t command = 1;
    int length = 8;
    uint8_t message[length];
    message[0] = length;
    message[1] = command;
    message[2] = which;
    message[3] = axis;
    insert_float(value, &message[4]);
    sendto(sock, message, length, 0, (struct sockaddr *)&serv_addr, addr_len);
}

int receiveRobotData(std::vector<uint8_t>& buffer) {
    if (!connected && !initialized) return -1;
    char recv_buffer[16384] = {0};
    int bytesRead = recvfrom(sock, recv_buffer, 16384, 0, (struct sockaddr *)&serv_addr, &addr_len);
    
    if (bytesRead > 0) {
        buffer.assign(recv_buffer, recv_buffer + bytesRead);
    }
    
    return bytesRead;
}

void sendKeyboardEvent(uint32_t keyval, uint8_t state) {
    if (!connected) return;
    uint8_t message[5];
    message[0] = 5;       // messageSize
    message[1] = 2;       // command (keyboard)
    message[2] = (uint8_t)((keyval >> 8) & 0xff);
    message[3] = (uint8_t)((keyval >> 0) & 0xff);
    message[4] = state;   // 1 for press, 0 for release
    sendto(sock, message, sizeof(message), 0, (struct sockaddr *)&serv_addr, addr_len);
}

void sendJoystickButton(uint8_t which, uint8_t button, uint8_t state) {
    if (!connected) return;
    uint8_t command = 5;
    int length = 5;
    uint8_t message[length];
    message[0] = length;
    message[1] = command;
    message[2] = which;
    message[3] = button;
    message[4] = state;
    sendto(sock, message, length, 0, (struct sockaddr *)&serv_addr, addr_len);
}

void sendJoystickHat(uint8_t which, uint8_t hat, uint8_t value) {
    if (!connected) return;
    uint8_t command = 6;
    int length = 5;
    uint8_t message[length];
    message[0] = length;
    message[1] = command;
    message[2] = which;
    message[3] = hat;
    message[4] = value;
    sendto(sock, message, length, 0, (struct sockaddr *)&serv_addr, addr_len);
}

void sendHeartbeat() {
    if (!connected) return;
    
    lastHeartbeatTime = std::chrono::high_resolution_clock::now();
    uint8_t message[2];
    message[0] = 2; // length
    message[1] = 0; // command (heartbeat)
    sendto(sock, message, sizeof(message), 0, (struct sockaddr *)&serv_addr, addr_len);
}

void sendVideoHeartbeat() {
    if (!videoConnected) return;
    uint8_t message[2];
    message[0] = 2; // length
    message[1] = 0; // command (heartbeat)
    sendto(videoSock, message, sizeof(message), 0, (struct sockaddr *)&video_serv_addr, video_addr_len);
}