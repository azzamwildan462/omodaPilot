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


#include <map>
#include <assert.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include "constants.h"
#include "ros_utils.h"
#include "sensor_params.h"
#include "profile_manager.h"



namespace realsense2_camera
{



class FrequencyDiagnostics
{
public:
    FrequencyDiagnostics(std::string name, int expected_frequency, std::shared_ptr<diagnostic_updater::Updater> updater);
    FrequencyDiagnostics (const FrequencyDiagnostics& other);
    ~FrequencyDiagnostics();

public:
    void Tick();

private:
    std::string _name;
    double _min_freq, _max_freq;
    diagnostic_updater::FrequencyStatusParam _freq_status_param;
    diagnostic_updater::FrequencyStatus _freq_status;
    std::shared_ptr<diagnostic_updater::Updater> _p_updater;
}; // !class FrequencyDiagnostics



class RosSensor : public rs2::sensor
{
public:
    RosSensor(
        rs2::sensor sensor,
        std::shared_ptr<Parameters> parameters, 
        std::function<void(rs2::frame)> frame_callback,
        std::function<void()> update_sensor_func,
        std::function<void()> hardware_reset_func, 
        std::shared_ptr<diagnostic_updater::Updater> diagnostics_updater,
        rclcpp::Logger logger,
        bool force_image_default_qos = false,
        bool is_rosbag_file = false
    );
    ~RosSensor();

public:
    template<class T> 
    bool is() const { return rs2::sensor::is<T>(); }

public:
    void registerSensorParameters();
    bool getUpdatedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
    void runFirstFrameInitialization();
    virtual bool start(const std::vector<rs2::stream_profile>& profiles);
    void stop();
    rmw_qos_profile_t getQOS(const stream_index_pair& sip) const;
    rmw_qos_profile_t getInfoQOS(const stream_index_pair& sip) const;

private:
    void setupErrorCallback();
    void setParameters(bool is_rosbag_file = false);
    void clearParameters();
    void set_sensor_auto_exposure_roi();
    void registerAutoExposureROIOptions();
    void UpdateSequenceIdCallback();
    
    template<class T> 
    void set_sensor_parameter_to_ros(rs2_option option);

private:
    bool profiles_equal(const rs2::stream_profile& a, const rs2::stream_profile& b);
    bool is_profiles_in_profiles(const std::vector<stream_profile>& sub_profiles, const std::vector<stream_profile>& all_profiles);
    bool compare_profiles_lists(const std::vector<stream_profile>& active_profiles, const std::vector<stream_profile>& wanted_profiles);

private:
    rclcpp::Logger _logger;
    std::function<void(rs2::frame)> _origin_frame_callback;
    std::function<void(rs2::frame)> _frame_callback;
    SensorParams _params;
    std::function<void()> _update_sensor_func, _hardware_reset_func;
    bool _is_first_frame;
    std::vector<std::function<void()> > _first_frame_functions_stack;
    std::vector<std::shared_ptr<ProfilesManager> > _profile_managers;
    rs2::region_of_interest _auto_exposure_roi;
    std::vector<std::string> _parameters_names;
    std::shared_ptr<diagnostic_updater::Updater> _diagnostics_updater;
    std::map<stream_index_pair, FrequencyDiagnostics> _frequency_diagnostics;
    bool _force_image_default_qos;
}; // !class RosSensor



} // !namespace realsense2_camera
