#include "master/master.hpp"

Master::Master()
    : Node("master")
{
    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    pub_target_steering_angle = this->create_publisher<std_msgs::msg::Float32>("cmd_target_steering_angle", 1);
    pub_target_velocity = this->create_publisher<std_msgs::msg::Float32>("cmd_target_velocity", 1);
    pub_hw_flag = this->create_publisher<std_msgs::msg::UInt8>("cmd_hw_flag", 1);

    sub_key_pressed = this->create_subscription<std_msgs::msg::Int16>(
        "/key_pressed", 1, std::bind(&Master::callback_sub_key_pressed, this, std::placeholders::_1));

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_routine, this));

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg)
{
    current_key_pressed = msg->data;

    switch (current_key_pressed)
    {
    case 'a':
        cmd_target_steering_angle += 0.1;
        break;
    case 'd':
        cmd_target_steering_angle -= 0.1;
        break;

    case 'l':
        cmd_hw_flag |= CMD_STEER_ACTIVE;
        break;
    case 'k':
        cmd_hw_flag |= CMD_GAS_ACTIVE;
        break;

    case ' ':
        cmd_hw_flag = 0;
        cmd_target_steering_angle = 0;
        cmd_target_velocity = 0;
        break;
    }
}

void Master::callback_routine()
{
    process_transmitter();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}