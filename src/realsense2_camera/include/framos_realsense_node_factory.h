// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#pragma once



#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>
#include "base_realsense_node.h"
#include "realsense_node_factory.h"
#include "dynamic_params.h"



namespace realsense2_camera
{



class FramosRealSenseNodeFactory : public rclcpp::Node
{
public:
    explicit FramosRealSenseNodeFactory(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    FramosRealSenseNodeFactory(const std::string& node_name, const std::string& ns, const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    virtual ~FramosRealSenseNodeFactory(); 
    
private:
    void init();
    void startDevice();
    void changeDeviceCallback(rs2::event_information& info);
    void getDevice(rs2::device_list list);
    void tryGetLogSeverity(rs2_log_severity& severity) const;
    static std::string parseUsbPort(std::string line);

private:
    std::string api_version_to_string(int version);
    void setRs2CliArgs();
    void setDeviceFilterList();
    uint64_t serialFromString(const std::string& serial);
    uint32_t ipFromString(const std::string& serial);
    bool isNumber(const std::string& str);

private:
    rclcpp::Node::SharedPtr _node;
    rs2::device _device;
    std::unique_ptr<BaseRealSenseNode> _realSenseNode;
    rs2::context* _ctx;
    std::string _serial_no;
    std::string _usb_port_id;
    std::string _device_type;
    double _wait_for_device_timeout;
    double _reconnect_timeout;
    bool _initial_reset;
    std::thread _query_thread;
    bool _is_alive;
    rclcpp::Logger _logger;
    std::shared_ptr<Parameters> _parameters;
    std::string _ip_address;
}; // !class FramosRealSenseNodeFactory



} // !namespace realsense2_camera
