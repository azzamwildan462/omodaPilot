#include "hardware/canable2_socket_can.h"

#include <iostream>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>
#include <unistd.h>

int CANable2_SOCKET_CAN::parse_incoming_data(can_frame_t *frame)
{
    frame->timestamp = this->tick;

    if (frame->id == 0x065)
    {
        fb_relay_state = frame->data[0];
        fb_key_state = frame->data[1];
        // logger->info("Relay State: %d, Key State: %d, data2: %d, counter: %d, crc: %d",
        //              fb_relay_state, fb_key_state, frame->data[2], frame->data[3], frame->data[4]);
    }
    else if (frame->id == CHERY_CANFD_STEER_ANGLE_SENSOR_FRAME_ID)
    {
        bzero(&angle_sensor, sizeof(angle_sensor));
        chery_canfd_steer_angle_sensor_unpack(&angle_sensor, frame->data, frame->dlc);
        this->fb_steering_angle = chery_canfd_steer_angle_sensor_steer_angle_decode(angle_sensor.steer_angle) * 3.141592653589793 / 180.0; // deg to rad
    }
    else if (frame->id == CHERY_CANFD_WHEEL_SPEED_REAR_FRAME_ID)
    {
        bzero(&wheel_speed_rear, sizeof(wheel_speed_rear));
        chery_canfd_wheel_speed_rear_unpack(&wheel_speed_rear, frame->data, frame->dlc);
        this->wheel_speed_rl = chery_canfd_wheel_speed_rear_wheel_speed_rl_decode(wheel_speed_rear.wheel_speed_rl);
        this->wheel_speed_rr = chery_canfd_wheel_speed_rear_wheel_speed_rr_decode(wheel_speed_rear.wheel_speed_rr);
    }
    else if (frame->id == CHERY_CANFD_WHEEL_SPEED_FRNT_FRAME_ID)
    {
        bzero(&wheel_speed_front, sizeof(wheel_speed_front));
        chery_canfd_wheel_speed_frnt_unpack(&wheel_speed_front, frame->data, frame->dlc);
        this->wheel_speed_fl = chery_canfd_wheel_speed_frnt_wheel_speed_fl_decode(wheel_speed_front.wheel_speed_fl);
        this->wheel_speed_fr = chery_canfd_wheel_speed_frnt_wheel_speed_fr_decode(wheel_speed_front.wheel_speed_fr);
    }
    else if (frame->id == CHERY_CANFD_ENGINE_DATA_FRAME_ID)
    {
        bzero(&engine_data, sizeof(engine_data));
        chery_canfd_engine_data_unpack(&engine_data, frame->data, frame->dlc);
        this->engine_gas_pos = chery_canfd_engine_data_gas_pos_decode(engine_data.gas_pos);
        this->engine_gas = chery_canfd_engine_data_gas_decode(engine_data.gas);
        this->engine_gear = chery_canfd_engine_data_gear_decode(engine_data.gear);
        this->engine_gear_button = chery_canfd_engine_data_gear_button_decode(engine_data.gear_button);
        this->engine_brake_press = chery_canfd_engine_data_brake_press_decode(engine_data.brake_press);
        this->engine_switch_to_p = chery_canfd_engine_data_switch_to_p_decode(engine_data.switch_to_p);
    }
    else if (frame->id == CHERY_CANFD_STEER_BUTTON_FRAME_ID)
    {
        bzero(&steer_button, sizeof(steer_button));
        chery_canfd_steer_button_unpack(&steer_button, frame->data, frame->dlc);
        this->btn_acc = chery_canfd_steer_button_acc_decode(steer_button.acc);
        this->btn_cc = chery_canfd_steer_button_cc_btn_decode(steer_button.cc_btn);
        this->btn_res_plus = chery_canfd_steer_button_res_plus_decode(steer_button.res_plus);
        this->btn_res_minus = chery_canfd_steer_button_res_minus_decode(steer_button.res_minus);
        this->btn_gap_adjust_up = chery_canfd_steer_button_gap_adjust_up_decode(steer_button.gap_adjust_up);
        this->btn_gap_adjust_down = chery_canfd_steer_button_gap_adjust_down_decode(steer_button.gap_adjust_down);
    }
    else if (frame->id == CHERY_CANFD_BRAKE_DATA_FRAME_ID)
    {
        bzero(&brake_data, sizeof(brake_data));
        chery_canfd_brake_data_unpack(&brake_data, frame->data, frame->dlc);
        this->data_brake_pos = chery_canfd_brake_data_brake_pos_decode(brake_data.brake_pos);
    }
    else if (frame->id == CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID)
    {
        bzero(&lkas_cam_cmd, sizeof(lkas_cam_cmd));
        chery_canfd_lkas_cam_cmd_345_unpack(&lkas_cam_cmd, frame->data, frame->dlc);
        this->target_steering_angle = chery_canfd_steer_angle_sensor_steer_angle_decode(lkas_cam_cmd.cmd) * 3.141592653589793 / 180.0; // deg to rad
    }
    else if (frame->id == CHERY_CANFD_ACC_CMD_FRAME_ID)
    {
        bzero(&acc_cam_cmd, sizeof(acc_cam_cmd));
        chery_canfd_acc_cmd_unpack(&acc_cam_cmd, frame->data, frame->dlc);
    }
    else if (frame->id == CHERY_CANFD_ACC_FRAME_ID)
    {
        bzero(&acc_data, sizeof(acc_data));
        chery_canfd_acc_unpack(&acc_data, frame->data, frame->dlc);
    }
    else if (frame->id == CHERY_CANFD_LKAS_STATE_FRAME_ID)
    {
        bzero(&lkas_state, sizeof(lkas_state));
        chery_canfd_lkas_state_unpack(&lkas_state, frame->data, frame->dlc);
    }
    else if (frame->id == CHERY_CANFD_SETTING_FRAME_ID)
    {
        bzero(&setting_cmd_from_adas, sizeof(setting_cmd_from_adas));
        chery_canfd_setting_unpack(&setting_cmd_from_adas, frame->data, frame->dlc);
        this->speed_cc = chery_canfd_setting_cc_speed_decode(setting_cmd_from_adas.cc_speed);
        this->acc_avail = chery_canfd_setting_acc_available_decode(setting_cmd_from_adas.acc_available);
    }
    else if (frame->id == CHERY_CANFD_BCM_SIGNAL_1_FRAME_ID)
    {
        bzero(&bcm_signal_1, sizeof(bcm_signal_1));
        chery_canfd_bcm_signal_1_unpack(&bcm_signal_1, frame->data, frame->dlc);
        this->bcm_door_lock = chery_canfd_bcm_signal_1_door_lock_decode(bcm_signal_1.door_lock);
        this->bcm_rl_door_open = chery_canfd_bcm_signal_1_rl_door_open_decode(bcm_signal_1.rl_door_open);
        this->bcm_rr_door_open = chery_canfd_bcm_signal_1_rr_door_open_decode(bcm_signal_1.rr_door_open);
        this->bcm_fl_door_open = chery_canfd_bcm_signal_1_fl_door_open_decode(bcm_signal_1.fl_door_open);
        this->bcm_fr_door_open = chery_canfd_bcm_signal_1_fr_door_open_decode(bcm_signal_1.fr_door_open);
        this->bcm_sign_signal = chery_canfd_bcm_signal_1_sign_signal_decode(bcm_signal_1.sign_signal);
    }
    else if (frame->id == CHERY_CANFD_BCM_SIGNAL_2_FRAME_ID)
    {
        bzero(&bcm_signal_2, sizeof(bcm_signal_2));
        chery_canfd_bcm_signal_2_unpack(&bcm_signal_2, frame->data, frame->dlc);
        this->bcm_left_sign = chery_canfd_bcm_signal_2_left_sign_decode(bcm_signal_2.left_sign);
        this->bcm_right_sign = chery_canfd_bcm_signal_2_right_sign_decode(bcm_signal_2.right_sign);
        this->bcm_door_lock_open = chery_canfd_bcm_signal_2_door_lock_open_decode(bcm_signal_2.door_lock_open);
        this->bcm_high_beam = chery_canfd_bcm_signal_2_high_beam_decode(bcm_signal_2.high_beam);
        this->bcm_left_sign_pressed = chery_canfd_bcm_signal_2_left_sign_pressed_decode(bcm_signal_2.left_sign_pressed);
        this->bcm_right_sign_pressed = chery_canfd_bcm_signal_2_right_sign_pressed_decode(bcm_signal_2.right_sign_pressed);
        this->bcm_wiper_button = chery_canfd_bcm_signal_2_wiper_button_decode(bcm_signal_2.wiper_button);
    }
    else if (frame->id == CHERY_CANFD_STEER_SENSOR_2_FRAME_ID)
    {
        bzero(&steer_sensor_2, sizeof(steer_sensor_2));
        chery_canfd_steer_sensor_2_unpack(&steer_sensor_2, frame->data, frame->dlc);
        this->fb_steering_torq_drv = chery_canfd_steer_sensor_2_torque_driver_decode(steer_sensor_2.torque_driver);
    }
    else if (frame->id == CHERY_CANFD_BSM_RIGHT_FRAME_ID)
    {
        bzero(&bsm_right, sizeof(bsm_right));
        chery_canfd_bsm_right_unpack(&bsm_right, frame->data, frame->dlc);
        this->bsm_right_detect = chery_canfd_bsm_right_bsm_right_detect_decode(bsm_right.bsm_right_detect);
    }
    else if (frame->id == CHERY_CANFD_BSM_LEFT_FRAME_ID)
    {
        bzero(&bsm_left, sizeof(bsm_left));
        chery_canfd_bsm_left_unpack(&bsm_left, frame->data, frame->dlc);
        this->bsm_left_detect = chery_canfd_bsm_left_bsm_left_detect_decode(bsm_left.bsm_left_detect);
    }
    else
    {
        // Unknown frame ID
        return -1;
    }

    return 0;
}

int CANable2_SOCKET_CAN::dlc_to_len(uint8_t dlc)
{
    // Convert CAN FD DLC to actual data length
    switch (dlc)
    {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
        return dlc;
    case 9:
        return 12;
    case 10:
        return 16;
    case 11:
        return 20;
    case 12:
        return 24;
    case 13:
        return 32;
    case 14:
        return 48;
    case 15:
        return 64;
    default:
        return 8; // fallback
    }
}

uint8_t CANable2_SOCKET_CAN::len_to_dlc(int len)
{
    // Convert actual data length to CAN FD DLC
    if (len <= 8)
        return len;
    else if (len <= 12)
        return 9;
    else if (len <= 16)
        return 10;
    else if (len <= 20)
        return 11;
    else if (len <= 24)
        return 12;
    else if (len <= 32)
        return 13;
    else if (len <= 48)
        return 14;
    else
        return 15; // 64 bytes max
}

#include <cstdlib> // untuk system()

int CANable2_SOCKET_CAN::setup_can_interface(const std::string &interface_name, int bitrate, int fd_bitrate)
{
    logger->info("Setting up CAN interface: %s", interface_name.c_str());

    // Check if interface exists
    std::string check_cmd = "ip link show " + interface_name + " > /dev/null 2>&1";
    if (system(check_cmd.c_str()) != 0)
    {
        logger->error("CAN interface %s not found", interface_name.c_str());
        return -1;
    }

    // Bring interface down first
    std::string down_cmd = "sudo ip link set " + interface_name + " down";
    if (system(down_cmd.c_str()) != 0)
    {
        logger->warn("Failed to bring down %s (might be already down)", interface_name.c_str());
    }

    // Setup CAN FD interface
    std::string setup_cmd = "sudo ip link set " + interface_name +
                            " type can bitrate " + std::to_string(bitrate) +
                            " fd on dbitrate " + std::to_string(fd_bitrate);

    if (system(setup_cmd.c_str()) != 0)
    {
        logger->error("Failed to configure %s", interface_name.c_str());
        return -1;
    }

    // Bring interface up
    std::string up_cmd = "sudo ip link set " + interface_name + " up";
    if (system(up_cmd.c_str()) != 0)
    {
        logger->error("Failed to bring up %s", interface_name.c_str());
        return -1;
    }

    logger->info("CAN interface %s configured successfully", interface_name.c_str());
    return 0;
}

int CANable2_SOCKET_CAN::init(std::string device_name, int baudRate, int fd_baudrate)
{
    this->device_name = device_name;

    logger->info("CANable2_SOCKET_CAN initializing on %s", device_name.c_str());

    // Auto-setup CAN interface
    if (setup_can_interface(device_name, baudRate, fd_baudrate) != 0)
    {
        logger->error("Failed to setup CAN interface %s", device_name.c_str());
        return -1;
    }

    // Create socket
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0)
    {
        logger->error("Failed to create CAN socket: %s", strerror(errno));
        return -1;
    }

    // Enable CAN FD frames (optional, fallback ke classic CAN)
    int enable_canfd = 1;
    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        logger->warn("CAN FD not supported, using classic CAN: %s", strerror(errno));
        use_canfd = false;
    }
    else
    {
        use_canfd = true;
        logger->info("CAN FD enabled");
    }

    // Set socket to non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0)
    {
        logger->error("Failed to get socket flags: %s", strerror(errno));
        close(sockfd);
        return -1;
    }

    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        logger->error("Failed to set socket to non-blocking: %s", strerror(errno));
        close(sockfd);
        return -1;
    }

    // Get interface index
    struct ifreq ifr;
    strcpy(ifr.ifr_name, device_name.c_str());
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        logger->error("Failed to get interface index for %s: %s", device_name.c_str(), strerror(errno));
        close(sockfd);
        return -1;
    }

    // Bind socket to CAN interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        logger->error("Failed to bind socket to %s: %s", device_name.c_str(), strerror(errno));
        close(sockfd);
        return -1;
    }

    this->fd = sockfd;
    logger->info("CANable2_SOCKET_CAN initialized on %s successfully", device_name.c_str());
    return 0;
}

int CANable2_SOCKET_CAN::send_msg(can_frame_t *can_msg)
{
    if (sockfd < 0)
    {
        logger->error("Socket not initialized");
        return -1;
    }

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id = can_msg->id;

    // Handle CAN FD data length
    int actual_len = dlc_to_len(can_msg->dlc);
    frame.len = actual_len;

    // Set CAN FD flagss
    if (can_msg->flags & 4)
    { // FD frame
        // logger->info("CAN FD+BRS frame: ID=0x%03X", can_msg->id);
        frame.flags |= CANFD_BRS; // Enable bit rate switching
    }

    memcpy(frame.data, can_msg->data, actual_len);

    ssize_t bytes_sent = write(sockfd, &frame, sizeof(frame));
    if (bytes_sent < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // Socket buffer full, not an error
            return 1;
        }
        // logger->error("Failed to send CAN frame: %s", strerror(errno));
        return 1;
    }

    return 0;
}

can_frame_t CANable2_SOCKET_CAN::recv_msg()
{
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    return frame;
}

int CANable2_SOCKET_CAN::send_msgs(const std::vector<can_frame_t> &can_msgs)
{
    for (const auto &msg : can_msgs)
    {
        if (send_msg(const_cast<can_frame_t *>(&msg)) < 0)
        {
            return -1;
        }
    }
    return 0;
}

std::vector<can_frame_t> CANable2_SOCKET_CAN::recv_msgs()
{
    std::vector<can_frame_t> frames;

    if (sockfd < 0)
    {
        return frames;
    }

    struct canfd_frame frame;

    while (true)
    {
        ssize_t bytes_read = read(sockfd, &frame, sizeof(frame));

        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // No more data available
                break;
            }
            else
            {
                logger->error("Failed to read CAN frame: %s", strerror(errno));
                break;
            }
        }

        if (bytes_read == sizeof(struct canfd_frame))
        {
            // CAN FD frame
            can_frame_t can_frame;
            memset(&can_frame, 0, sizeof(can_frame));

            can_frame.id = frame.can_id & CAN_EFF_MASK;
            can_frame.dlc = len_to_dlc(frame.len);
            can_frame.flags = 4; // Mark as FD frame
            can_frame.timestamp = this->tick;

            memcpy(can_frame.data, frame.data, frame.len);

            // Process incoming data
            parse_incoming_data(&can_frame);

            // Check if frame should be intercepted
            bool is_intercepted = false;
            for (auto id : intercepted_can_ids)
            {
                if (can_frame.id == id)
                {
                    is_intercepted = true;
                    break;
                }
            }

            if (!is_intercepted)
            {
                frames.push_back(can_frame);
            }
        }
        else if (bytes_read == sizeof(struct can_frame))
        {
            // Classic CAN frame
            struct can_frame *classic_frame = (struct can_frame *)&frame;

            can_frame_t can_frame;
            memset(&can_frame, 0, sizeof(can_frame));

            can_frame.id = classic_frame->can_id & CAN_EFF_MASK;
            can_frame.dlc = classic_frame->can_dlc;
            can_frame.flags = 0; // Classic CAN
            can_frame.timestamp = this->tick;

            memcpy(can_frame.data, classic_frame->data, classic_frame->can_dlc);

            // Process incoming data
            parse_incoming_data(&can_frame);

            // Check if frame should be intercepted
            bool is_intercepted = false;
            for (auto id : intercepted_can_ids)
            {
                if (can_frame.id == id)
                {
                    is_intercepted = true;
                    break;
                }
            }

            if (!is_intercepted)
            {
                frames.push_back(can_frame);
            }
        }
    }

    return frames;
}

int CANable2_SOCKET_CAN::update(uint64_t tick)
{
    // Update tick
    this->tick = tick;

    // Calculate speed from wheel speeds
    float raw_kmh_longitudinal = (this->wheel_speed_rl + this->wheel_speed_rr) / 2.0f;
    this->fb_current_velocity = raw_kmh_longitudinal / 3.6f;

    // Update gear status strings
    switch (this->engine_gear)
    {
    case CHERY_CANFD_ENGINE_DATA_GEAR_P_CHOICE:
        this->gear_status = "P";
        break;
    case CHERY_CANFD_ENGINE_DATA_GEAR_R_CHOICE:
        this->gear_status = "R";
        this->fb_current_velocity *= -1;
        break;
    case CHERY_CANFD_ENGINE_DATA_GEAR_N_CHOICE:
        this->gear_status = "N";
        break;
    case CHERY_CANFD_ENGINE_DATA_GEAR_D_CHOICE:
        this->gear_status = "D";
        break;
    }

    // Update gear button status strings
    switch (this->engine_gear_button)
    {
    case CHERY_CANFD_ENGINE_DATA_GEAR_BUTTON_PARK__PRESSED_CHOICE:
        this->gear_button_status = "Park Pressed";
        break;
    case CHERY_CANFD_ENGINE_DATA_GEAR_BUTTON_REVERSE__PRESSED_CHOICE:
        this->gear_button_status = "Reverse Pressed";
        break;
    case CHERY_CANFD_ENGINE_DATA_GEAR_BUTTON_NETRAL__PRESSED_CHOICE:
        this->gear_button_status = "Netral Pressed";
        break;
    case CHERY_CANFD_ENGINE_DATA_GEAR_BUTTON_DRIVE__PRESSED_CHOICE:
        this->gear_button_status = "Drive Pressed";
        break;
    }

    return 0;
}

int CANable2_SOCKET_CAN::init_update_as_new_thread()
{
    return 0;
}

void CANable2_SOCKET_CAN::shutdown()
{
    if (sockfd >= 0)
    {
        if (close(sockfd) < 0)
        {
            logger->warn("Failed to close socket: %s", strerror(errno));
        }
        else
        {
            logger->info("CANable2_SOCKET_CAN shutdown successfully");
        }
        sockfd = -1;
        fd = -1;
    }

    if (!device_name.empty() && device_name.find("vcan") != 0)
    {
        std::string down_cmd = "sudo ip link set " + device_name + " down";
        if (system(down_cmd.c_str()) == 0)
        {
            logger->info("CAN interface %s brought down successfully", device_name.c_str());
        }
        else
        {
            logger->warn("Failed to bring down CAN interface %s", device_name.c_str());
        }
    }
}