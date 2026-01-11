#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#pragma pack(push, 1)
struct PC2CAN
{
    int16_t global_fsm;
    float target_velocity;
    float target_steering_angle;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct CAN2PC
{
    uint8_t flag_override_status;
    float current_velocity;
    float current_steering_angle;
};
#pragma pack(pop)

class UDPCANPC : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine_send;
    rclcpp::TimerBase::SharedPtr tim_routine_recv;

    // CANBUS HAL
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SUB_fb_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SUB_fb_current_velocity;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr SUB_flag_override_ctrl;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr PUB_target_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr PUB_target_velocity;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr PUB_global_fsm;

    // PC
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SUB_target_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SUB_target_velocity;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr SUB_global_fsm;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr PUB_flag_override_ctrl;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr PUB_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr PUB_fb_current_velocity;

    // Configs
    bool is_iam_PC = false;
    std::string udp_my_ip = "0.0.0.0";
    int udp_my_port = 1234;
    std::string udp_target_ip = "192.168.69.69";
    int udp_target_port = 5432;

    HelpLogger logger;

    // Vars
    int sockfd;
    struct sockaddr_in my_addr, target_addr;

    // PC
    int16_t pc_global_fsm = 0;
    float pc_target_velocity = 0.0;
    float pc_target_steering_angle = 0.0;
    uint8_t pc_flag_override_status = 0;
    float pc_current_velocity = 0.0;
    float pc_current_steering_angle = 0.0;

    // CAN
    uint8_t can_flag_override_status = 0;
    float can_current_velocity = 0.0;
    float can_current_steering_angle = 0.0;
    int16_t can_global_fsm = 0;
    float can_target_velocity = 0.0;
    float can_target_steering_angle = 0.0;

    // Struct
    PC2CAN pc2can;
    CAN2PC can2pc;

    UDPCANPC()
        : Node("udp_can_pc")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        this->declare_parameter("is_iam_PC", is_iam_PC);
        this->get_parameter("is_iam_PC", is_iam_PC);

        this->declare_parameter("udp_my_ip", udp_my_ip);
        this->get_parameter("udp_my_ip", udp_my_ip);

        this->declare_parameter("udp_my_port", udp_my_port);
        this->get_parameter("udp_my_port", udp_my_port);

        this->declare_parameter("udp_target_ip", udp_target_ip);
        this->get_parameter("udp_target_ip", udp_target_ip);

        this->declare_parameter("udp_target_port", udp_target_port);
        this->get_parameter("udp_target_port", udp_target_port);

        init_udp();

        if (is_iam_PC)
        {
            SUB_target_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
                "/master/cmd_target_steering_angle", 1, std::bind(&UDPCANPC::callback_sub_target_steering_angle, this, std::placeholders::_1));
            SUB_target_velocity = this->create_subscription<std_msgs::msg::Float32>(
                "/master/cmd_target_velocity", 1, std::bind(&UDPCANPC::callback_sub_target_velocity, this, std::placeholders::_1));
            SUB_global_fsm = this->create_subscription<std_msgs::msg::Int16>(
                "/master/global_fsm", 1, std::bind(&UDPCANPC::callback_sub_global_fsm, this, std::placeholders::_1));

            PUB_flag_override_ctrl = this->create_publisher<std_msgs::msg::UInt8>(
                "/hardware/cmd_flag_override_ctrl", 1);
            PUB_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>(
                "/hardware/fb_steering_angle", 1);
            PUB_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>(
                "/hardware/fb_current_velocity", 1);
        }
        else
        {
            SUB_fb_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
                "/hardware/fb_steering_angle", 1, std::bind(&UDPCANPC::callback_sub_fb_steering_angle, this, std::placeholders::_1));
            SUB_fb_current_velocity = this->create_subscription<std_msgs::msg::Float32>(
                "/hardware/fb_current_velocity", 1, std::bind(&UDPCANPC::callback_sub_fb_current_velocity, this, std::placeholders::_1));
            SUB_flag_override_ctrl = this->create_subscription<std_msgs::msg::UInt8>(
                "/hardware/cmd_flag_override_ctrl", 1, std::bind(&UDPCANPC::callback_sub_flag_override_ctrl, this, std::placeholders::_1));

            PUB_target_steering_angle = this->create_publisher<std_msgs::msg::Float32>(
                "/master/cmd_target_steering_angle", 1);
            PUB_target_velocity = this->create_publisher<std_msgs::msg::Float32>(
                "/master/cmd_target_velocity", 1);
            PUB_global_fsm = this->create_publisher<std_msgs::msg::Int16>(
                "/master/global_fsm", 1);
        }

        tim_routine_send = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&UDPCANPC::callback_routine_send, this));
        tim_routine_recv = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&UDPCANPC::callback_routine_recv, this));

        logger.info("UDPCANPC node initialized");
    }

    ~UDPCANPC()
    {
    }

    void callback_sub_fb_steering_angle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        can_current_steering_angle = msg->data;
    }
    void callback_sub_fb_current_velocity(const std_msgs::msg::Float32::SharedPtr msg)
    {
        can_current_velocity = msg->data;
    }
    void callback_sub_flag_override_ctrl(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        can_flag_override_status = msg->data;
    }
    void callback_sub_target_steering_angle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        pc_target_steering_angle = msg->data;
    }
    void callback_sub_target_velocity(const std_msgs::msg::Float32::SharedPtr msg)
    {
        pc_target_velocity = msg->data;
    }
    void callback_sub_global_fsm(const std_msgs::msg::Int16::SharedPtr msg)
    {
        pc_global_fsm = msg->data;
    }

    void routine_recv_as_CANBUS_HAL()
    {
        size_t data_received = recvfrom(
            sockfd,
            reinterpret_cast<uint8_t *>(&pc2can),
            sizeof(pc2can),
            MSG_DONTWAIT,
            nullptr,
            nullptr);

        if (data_received < sizeof(pc2can))
        {
            return;
        }

        // Process the received data
        std_msgs::msg::Int16 msg_global_fsm;
        msg_global_fsm.data = pc2can.global_fsm;
        PUB_global_fsm->publish(msg_global_fsm);

        std_msgs::msg::Float32 msg_target_steering_angle;
        msg_target_steering_angle.data = pc2can.target_steering_angle;
        PUB_target_steering_angle->publish(msg_target_steering_angle);

        std_msgs::msg::Float32 msg_target_velocity;
        msg_target_velocity.data = pc2can.target_velocity;
        PUB_target_velocity->publish(msg_target_velocity);
    }

    void routine_recv_as_PC()
    {
        size_t data_received = recvfrom(
            sockfd,
            reinterpret_cast<uint8_t *>(&can2pc),
            sizeof(can2pc),
            MSG_DONTWAIT,
            nullptr,
            nullptr);

        if (data_received < sizeof(can2pc))
        {
            return;
        }

        // Process the received data
        std_msgs::msg::UInt8 msg_flag_override_ctrl;
        msg_flag_override_ctrl.data = can2pc.flag_override_status;
        PUB_flag_override_ctrl->publish(msg_flag_override_ctrl);

        std_msgs::msg::Float32 msg_fb_steering_angle;
        msg_fb_steering_angle.data = can2pc.current_steering_angle;
        PUB_fb_steering_angle->publish(msg_fb_steering_angle);

        std_msgs::msg::Float32 msg_fb_current_velocity;
        msg_fb_current_velocity.data = can2pc.current_velocity;
        PUB_fb_current_velocity->publish(msg_fb_current_velocity);
    }

    void routine_send_as_CANBUS_HAL()
    {
        can2pc.current_steering_angle = can_current_steering_angle;
        can2pc.current_velocity = can_current_velocity;
        can2pc.flag_override_status = can_flag_override_status;

        size_t data_sent = sendto(
            sockfd,
            reinterpret_cast<uint8_t *>(&can2pc),
            sizeof(can2pc),
            MSG_DONTWAIT,
            (struct sockaddr *)&target_addr,
            sizeof(target_addr));

        (void)data_sent;
    }

    void routine_send_as_PC()
    {
        pc2can.target_steering_angle = pc_target_steering_angle;
        pc2can.target_velocity = pc_target_velocity;
        pc2can.global_fsm = pc_global_fsm;

        size_t data_sent = sendto(
            sockfd,
            reinterpret_cast<uint8_t *>(&pc2can),
            sizeof(pc2can),
            MSG_DONTWAIT,
            (struct sockaddr *)&target_addr,
            sizeof(target_addr));

        (void)data_sent;
    }

    void init_udp()
    {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(udp_my_port);
        my_addr.sin_addr.s_addr = inet_addr(udp_my_ip.c_str());

        if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(sockfd);
            return;
        }

        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(udp_target_port);
        target_addr.sin_addr.s_addr = inet_addr(udp_target_ip.c_str());
    }

    void callback_routine_send()
    {
        if (is_iam_PC)
        {
            routine_send_as_PC();
        }
        else
        {
            routine_send_as_CANBUS_HAL();
        }
    }
    void callback_routine_recv()
    {
        if (is_iam_PC)
        {
            routine_recv_as_PC();
        }
        else
        {
            routine_recv_as_CANBUS_HAL();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_udp_can_pc = std::make_shared<UDPCANPC>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_udp_can_pc);
    executor.spin();

    return 0;
}