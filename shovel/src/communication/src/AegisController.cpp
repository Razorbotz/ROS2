#include "AegisController.hpp"

AegisController::AegisController(rclcpp::Node::SharedPtr node, 
                                 HeartbeatLink& link_ref, 
                                 std::mutex& mutex_ref, 
                                 RemoteStatus& status_ref,
                                 bool& rawData_in,
                                 SystemStatus& sysStatus_in
                                 )
    : nodeHandle(node), hb_link(link_ref), comms_mutex(mutex_ref), nanoStatus(status_ref),
      sendRawData_ref(rawData_in), systemStatus_ref(sysStatus_in)
{
}


void AegisController::on_packet_received(uint16_t id, const uint8_t* data, uint16_t len) {
    RCLCPP_INFO(nodeHandle->get_logger(), "Received Message ID: %d", id);
    // Packet containing motor speed values
    switch (id) {
        // --- TELEMETRY ---
        case 001: {
            MotorSpeed msg;
            if (parse_packet(data, len, msg, "MotorSpeed")) {
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Motor " << (int)msg.motor_id << " set to " << msg.speed << std::endl;
                // publish_speed(msg);
            }
            break;
        }
        case 002: {
            MotorPosition msg;
            if (parse_packet(data, len, msg, "MotorPosition")) {
                std::lock_guard<std::mutex> lock(comms_mutex);
                std::cout << "Motor " << (int)msg.motor_id << " pos set to " << msg.position << std::endl;
            }
            break;
        }

        case 010: {
            // Joystick Axis
            JoystickAxis msg;
            if (parse_packet(data, len, msg, "JoyAxis")) {
                // Handle axis
            }
            break;
        }

        case 011: {
            // Joystick Button
            JoystickButton msg;
            if (parse_packet(data, len, msg, "JoyBtn")) {
                // Handle button
            }
            break;
        }

        case 012: {
            // Joystick Hat
            JoystickHat msg;
            if (parse_packet(data, len, msg, "JoyHat")) {
                // Handle hat
            }
            break;
        }

        case 013: {
            // Keyboard
            KeyboardEvent msg;
            if (parse_packet(data, len, msg, "Keyboard")) {
                // Handle key
            }
            break;
        }

        case 020: {
            // Complex BinaryMessage
            // 1. Reconstruct list for BinaryMessage constructor
            std::list<uint8_t> byteList;
            for(int i=0; i<len; i++) byteList.push_back(data[i]);

            // 2. Decode
            BinaryMessage receivedMsg(byteList);
            break;
        }

        // --- Control Configuration ---
        case 100:
            // Request to Nano to control specific motors 
            // This shouldn't be sent to the Orin, might need to handle the error
            RCLCPP_ERROR(nodeHandle->get_logger(), "ERROR: ID 100 received on Orin (Nano only).");
            break;
        case 101:
            // Response to control request from Nano
            break;

        // --- State & Handshake --- 
        case 200:
            // Query about who is in control
            break;

        case 201:
            // Response that the sender is in control
            break;
            
        case 202:
            // Response that the sender is not in control
            break;
            
        case 203:
            // Request from Orin to Nano to retake control
            break;
            
        case 204:
            // Response from Nano to Orin to take control
            break;
            
        case 205:
            // Response from Nano to Orin to not take control
            // This will include a message for how many seconds to delay
            break;
            
        case 206:
            //  Query if the other is alive
            break;
            
        case 207:
            // Response to alive query
            break;
            
        case 208:
            // Request from Nano to Orin to relinquish control
            break;
            
        case 209:
            // Accept control of system
            break;
            

        // --- Operational Faults & Stops ---
        case 400:
            // Message to stop immediately
            break;
            
        case 401:
            // Message to stop gracefully
            break;
            
        case 402:
            // Message from the sender that it lost control of motors
            // Include a list of lost motor IDs
            break;
            
        case 404:
            // Lost Wi-Fi connection
            // Nano lost wifi connection, need to send all received data to it
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                nanoStatus.WIFI_UP = false;
                sendRawData_ref = true;
            }
            RCLCPP_WARN(nodeHandle->get_logger(), "Nano Wi-Fi is down");
            // Send ACK with ID 406
            break;

        case 405:
            // Regained Wi-Fi connection
            // Nano regained wifi connection, no need to send all received data to it
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                nanoStatus.WIFI_UP = true;
                sendRawData_ref = false;
            }
            RCLCPP_INFO(nodeHandle->get_logger(), "Nano Wi-Fi is up");
            // Send ACK with ID 406
            break;

        case 406:
            // Wi-Fi Mode Change Confirm
            break;

        case 407: { 
            // CAN Bus Down
            if (len != 2 * sizeof(uint8_t)) { 
                RCLCPP_ERROR(nodeHandle->get_logger(), "Packet 502 wrong size. Expected 2, got %d", len);
                break;
            }

            struct {
                uint8_t interface_id;
                uint8_t error_code;
            } payload;
            
            std::memcpy(&payload, data, 2);

            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                
                if (payload.interface_id == 0) nanoStatus.CAN0_UP = false;
                if (payload.interface_id == 1) nanoStatus.CAN1_UP = false;
                
                RCLCPP_WARN(nodeHandle->get_logger(), "CRITICAL: Remote CAN Interface %d DOWN (Error: %d)", payload.interface_id, payload.error_code);
            }
            // Send ACK with ID 409
            break;
        }
            
        case 408: { 
            // CAN Bus back up
            if (len != 2 * sizeof(uint8_t)) { 
                RCLCPP_ERROR(nodeHandle->get_logger(), "Packet 503 wrong size. Expected 2, got %d", len);
                break;
            }

            struct {
                uint8_t interface_id;
                uint8_t error_code;
            } payload;
            
            std::memcpy(&payload, data, 2);

            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                
                if (payload.interface_id == 0) nanoStatus.CAN0_UP = true;
                if (payload.interface_id == 1) nanoStatus.CAN1_UP = true;
                
                RCLCPP_INFO(nodeHandle->get_logger(), "CRITICAL: Remote CAN Interface %d UP (Error: %d)", payload.interface_id, payload.error_code);
            }
            // Send ACK with ID 409
            break;
        }
        case 409:
            // CAN Bus Mode Change Confirm
            break;

        case 410:
            break;
        
        case 411:
            break;
        
        case 412:
            break;

        // --- System & Critical Hardware --- 
        case 500:
            // System shutting down
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                nanoStatus.UP = false;
            }
            systemStatus_ref = SINGLE_FC;
            RCLCPP_WARN(nodeHandle->get_logger(), "Nano shutting down");
            break;
            
        case 501:
            // System functioning again
            {
                std::lock_guard<std::mutex> lock(comms_mutex); 
                nanoStatus.UP = true;
            }
            systemStatus_ref = PRIMARY;
            RCLCPP_INFO(nodeHandle->get_logger(), "Nano rebooted");
            break;    
    
        default:
            RCLCPP_WARN(nodeHandle->get_logger(), "Received unknown Message ID: %d", id);
            break;
    }
}
