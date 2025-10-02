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

    sub_fb_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
        "fb_steering_angle", 1, std::bind(&Master::callback_sub_fb_steering_angle, this, std::placeholders::_1));
    sub_fb_current_velocity = this->create_subscription<std_msgs::msg::Float32>(
        "fb_current_velocity", 1, std::bind(&Master::callback_sub_fb_current_velocity, this, std::placeholders::_1));
    sub_throttle_position = this->create_subscription<std_msgs::msg::Float32>(
        "fb_throttle_position", 1, std::bind(&Master::callback_sub_throttle_position, this, std::placeholders::_1));
    sub_brake_position = this->create_subscription<std_msgs::msg::Int16>(
        "fb_brake_position", 1, std::bind(&Master::callback_sub_brake_position, this, std::placeholders::_1));
    sub_gear_status = this->create_subscription<std_msgs::msg::UInt8>(
        "fb_gear_status", 1, std::bind(&Master::callback_sub_gear_status, this, std::placeholders::_1));
    sub_steer_torque = this->create_subscription<std_msgs::msg::Int8>(
        "fb_steer_torque", 1, std::bind(&Master::callback_sub_steer_torque, this, std::placeholders::_1));
    sub_key_pressed = this->create_subscription<std_msgs::msg::Int16>(
        "/key_pressed", 1, std::bind(&Master::callback_sub_key_pressed, this, std::placeholders::_1));

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_routine, this));

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_sub_fb_steering_angle(const std_msgs::msg::Float32::SharedPtr msg)
{
    fb_steering_angle = msg->data;
}

void Master::callback_sub_fb_current_velocity(const std_msgs::msg::Float32::SharedPtr msg)
{
    fb_current_velocity = msg->data;
}

void Master::callback_sub_throttle_position(const std_msgs::msg::Float32::SharedPtr msg)
{
    throttle_position = msg->data;
}

void Master::callback_sub_brake_position(const std_msgs::msg::Int16::SharedPtr msg)
{
    brake_position = msg->data;
}

void Master::callback_sub_gear_status(const std_msgs::msg::UInt8::SharedPtr msg)
{
    gear_status = msg->data;
}

void Master::callback_sub_steer_torque(const std_msgs::msg::Int8::SharedPtr msg)
{
    steer_torque = msg->data;
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

    case 'q':
        cmd_target_steering_angle = 4.8;
        break;
    case 'e':
        cmd_target_steering_angle = -4.8;
        break;

    case '9':
        cmd_target_steering_angle += 0.01;
        break;
    case '0':
        cmd_target_steering_angle -= 0.01;
        break;

    case 'w':
        cmd_target_velocity += 0.1;
        break;
    case 's':
        cmd_target_velocity -= 0.1;
        break;

    case 'z':
        cmd_target_velocity = -1.0;
        break;

    case '1':
        cmd_target_velocity = 1.0;
        break;

    case '2':
        cmd_target_velocity = 2.0;
        break;

    case '3':
        cmd_target_velocity = 3.0;
        break;

    case '4':
        cmd_target_velocity = 4.0;
        break;

    case '5':
        cmd_target_velocity = 5.0;
        break;

    case 'l':
        cmd_hw_flag |= CMD_STEER_ACTIVE;
        break;
    case 'k':
        cmd_hw_flag |= CMD_GAS_ACTIVE;
        break;
    case 'j':
        cmd_hw_flag |= CMD_GAS_FULL_STOP;
        break;
    case 'h':
        cmd_hw_flag |= CMD_GAS_ACCEL_ON;
        break;
    case ',':
        cmd_hw_flag &= ~CMD_STEER_ACTIVE;
        break;
    case 'm':
        cmd_hw_flag &= ~CMD_GAS_ACTIVE;
        break;
    case 'n':
        cmd_hw_flag &= ~CMD_GAS_FULL_STOP;
        break;
    case 'b':
        cmd_hw_flag &= ~CMD_GAS_ACCEL_ON;
        break;

    case 't':
        cmd_hw_flag |= CMD_ACC_BTN_PRESS;
        last_time_steer_button_press = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        break;

    case ' ':
        cmd_hw_flag = 0;
        cmd_target_steering_angle = 0;
        cmd_target_velocity = 0;
        break;
    }

    if (cmd_target_steering_angle > MAX_STEERING_ANGLE)
        cmd_target_steering_angle = MAX_STEERING_ANGLE;
    else if (cmd_target_steering_angle < MIN_STEERING_ANGLE)
        cmd_target_steering_angle = MIN_STEERING_ANGLE;
}

void Master::callback_routine()
{
    time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    // static uint16_t counter_steer_dipegang = 0;

    // if (steer_torque > 18)
    // {
    //     counter_steer_dipegang++;
    //     if (counter_steer_dipegang > 150)
    //         counter_steer_dipegang = 150;
    // }

    // if (counter_steer_dipegang > 5)
    // {
    //     cmd_hw_flag &= ~CMD_STEER_ACTIVE;
    // }

    // if ((cmd_hw_flag & CMD_STEER_ACTIVE) == 0)
    // {
    //     counter_steer_dipegang = 0;
    // }

    // last_time_steer_button_press - time_now > 2 second
    if ((cmd_hw_flag & CMD_ACC_BTN_PRESS) != 0 && (time_now - last_time_steer_button_press).seconds() > 1.0)
    {
        cmd_hw_flag &= ~CMD_ACC_BTN_PRESS;
    }

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