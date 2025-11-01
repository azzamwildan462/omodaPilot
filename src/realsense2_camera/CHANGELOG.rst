^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.55.1 (2024-05-28)
-------------------
* PR `#3106 <https://github.com/IntelRealSense/realsense-ros/issues/3106>`_ from SamerKhshiboun: Remove unused parameter _is_profile_exist
* PR `#3098 <https://github.com/IntelRealSense/realsense-ros/issues/3098>`_ from kadiredd: ROS live cam test fixes
* PR `#3094 <https://github.com/IntelRealSense/realsense-ros/issues/3094>`_ from kadiredd: ROSCI infra for live camera testing
* PR `#3066 <https://github.com/IntelRealSense/realsense-ros/issues/3066>`_ from SamerKhshiboun: Revert Foxy Build Support (From Source)
* PR `#3052 <https://github.com/IntelRealSense/realsense-ros/issues/3052>`_ from Arun-Prasad-V: Support for selecting profile for each stream_type
* PR `#3056 <https://github.com/IntelRealSense/realsense-ros/issues/3056>`_ from SamerKhshiboun: Add documentation for RealSense ROS2 Wrapper Windows installation
* PR `#3049 <https://github.com/IntelRealSense/realsense-ros/issues/3049>`_ from Arun-Prasad-V: Applying Colorizer filter to Aligned-Depth image
* PR `#3053 <https://github.com/IntelRealSense/realsense-ros/issues/3053>`_ from Nir-Az: Fix Coverity issues + remove empty warning log
* PR `#3007 <https://github.com/IntelRealSense/realsense-ros/issues/3007>`_ from Arun-Prasad-V: Skip updating Exp 1,2 & Gain 1,2 when HDR is disabled
* PR `#3042 <https://github.com/IntelRealSense/realsense-ros/issues/3042>`_ from kadiredd: Assert Fail if camera not found
* PR `#3008 <https://github.com/IntelRealSense/realsense-ros/issues/3008>`_ from Arun-Prasad-V: Renamed GL GPU enable param
* PR `#2989 <https://github.com/IntelRealSense/realsense-ros/issues/2989>`_ from Arun-Prasad-V: Dynamically switching b/w CPU & GPU processing
* PR `#3001 <https://github.com/IntelRealSense/realsense-ros/issues/3001>`_ from deep0294: Update ReadMe to run ROS2 Unit Test
* PR `#2998 <https://github.com/IntelRealSense/realsense-ros/issues/2998>`_ from SamerKhshiboun: fix calibration intrinsic fail
* PR `#2987 <https://github.com/IntelRealSense/realsense-ros/issues/2987>`_ from SamerKhshiboun: Remove D465 SKU
* PR `#2984 <https://github.com/IntelRealSense/realsense-ros/issues/2984>`_ from deep0294: Fix All Profiles Test
* PR `#2956 <https://github.com/IntelRealSense/realsense-ros/issues/2956>`_ from Arun-Prasad-V: Extending LibRS's GL support to RS ROS2
* PR `#2953 <https://github.com/IntelRealSense/realsense-ros/issues/2953>`_ from Arun-Prasad-V: Added urdf & mesh files for D405 model
* PR `#2940 <https://github.com/IntelRealSense/realsense-ros/issues/2940>`_ from Arun-Prasad-V: Fixing the data_type of ROS Params exposure & gain
* PR `#2948 <https://github.com/IntelRealSense/realsense-ros/issues/2948>`_ from Arun-Prasad-V: Disabling HDR during INIT
* PR `#2934 <https://github.com/IntelRealSense/realsense-ros/issues/2934>`_ from Arun-Prasad-V: Disabling hdr while updating exposure & gain values
* PR `#2946 <https://github.com/IntelRealSense/realsense-ros/issues/2946>`_ from gwen2018: fix ros random crash with error hw monitor command for asic temperature failed
* PR `#2865 <https://github.com/IntelRealSense/realsense-ros/issues/2865>`_ from PrasRsRos: add live camera tests
* PR `#2891 <https://github.com/IntelRealSense/realsense-ros/issues/2891>`_ from Arun-Prasad-V: revert PR2872
* PR `#2853 <https://github.com/IntelRealSense/realsense-ros/issues/2853>`_ from Arun-Prasad-V: Frame latency for the '/topic' provided by user
* PR `#2872 <https://github.com/IntelRealSense/realsense-ros/issues/2872>`_ from Arun-Prasad-V: Updating _camera_name with RS node's name
* PR `#2878 <https://github.com/IntelRealSense/realsense-ros/issues/2878>`_ from Arun-Prasad-V: Updated ros2 examples and readme
* PR `#2841 <https://github.com/IntelRealSense/realsense-ros/issues/2841>`_ from SamerKhshiboun: Remove Dashing, Eloquent, Foxy, L500 and SR300 support
* PR `#2868 <https://github.com/IntelRealSense/realsense-ros/issues/2868>`_ from Arun-Prasad-V: Fix Pointcloud topic frame_id
* PR `#2849 <https://github.com/IntelRealSense/realsense-ros/issues/2849>`_ from Arun-Prasad-V: Create /imu topic only when motion streams enabled
* PR `#2847 <https://github.com/IntelRealSense/realsense-ros/issues/2847>`_ from Arun-Prasad-V: Updated rs_launch param names
* PR `#2839 <https://github.com/IntelRealSense/realsense-ros/issues/2839>`_ from Arun-Prasad: Added ros2 examples
* PR `#2861 <https://github.com/IntelRealSense/realsense-ros/issues/2861>`_ from SamerKhshiboun: fix readme and nodefactory for ros2 run
* PR `#2859 <https://github.com/IntelRealSense/realsense-ros/issues/2859>`_ from PrasRsRos: Fix tests (topic now has camera name)
* PR `#2857 <https://github.com/IntelRealSense/realsense-ros/issues/2857>`_ from lge-ros2: Apply camera name in topics
* PR `#2840 <https://github.com/IntelRealSense/realsense-ros/issues/2840>`_ from SamerKhshiboun: Support Depth, IR and Color formats in ROS2
* PR `#2764 <https://github.com/IntelRealSense/realsense-ros/issues/2764>`_ from lge-ros2 : support modifiable camera namespace
* PR `#2830 <https://github.com/IntelRealSense/realsense-ros/issues/2830>`_ from SamerKhshiboun: Add RGBD + reduce changes between hkr and development
* PR `#2811 <https://github.com/IntelRealSense/realsense-ros/issues/2811>`_ from Arun-Prasad-V: Exposing stream formats params to user
* PR `#2825 <https://github.com/IntelRealSense/realsense-ros/issues/2825>`_ from SamerKhshiboun: Fix align_depth + add test
* PR `#2822 <https://github.com/IntelRealSense/realsense-ros/issues/2822>`_ from Arun-Prasad-V: Updated rs_launch configurations
* PR `#2726 <https://github.com/IntelRealSense/realsense-ros/issues/2726>`_ from PrasRsRos: Integration test template
* PR `#2742 <https://github.com/IntelRealSense/realsense-ros/issues/2742>`_ from danielhonies:Update rs_launch.py
* PR `#2806 <https://github.com/IntelRealSense/realsense-ros/issues/2806>`_ from Arun-Prasad-V: Enabling RGB8 Infrared stream
* PR `#2799 <https://github.com/IntelRealSense/realsense-ros/issues/2799>`_ from SamerKhshiboun: Fix overriding frames on same topics/CV-images due to a bug in PR2759
* PR `#2759 <https://github.com/IntelRealSense/realsense-ros/issues/2759>`_ from SamerKhshiboun: Cleanups and name fixes
* Contributors: (=YG=) Hyunseok Yang, Arun Prasad, Arun-Prasad-V, Daniel Honies, Hyunseok, Madhukar Reddy Kadireddy, Nir, Nir Azkiel, PrasRsRos, Samer Khshiboun, SamerKhshiboun, deep0294, gwen2018, nairps

4.54.1 (2023-06-27)
-------------------
* Applying AlignDepth filter after Pointcloud
* Publish /aligned_depth_to_color topic only when color frame present
* Support Iron distro
* Protect empty string dereference
* Fix: /tf and /static_tf topics' inconsistencies
* Revamped the TF related code
* Fixing TF frame links b/w multi camera nodes when using custom names
* Updated TF descriptions in launch py and readme
* Fixing /tf topic has only TFs of last started sensor
* add D430i support
* Fix Swapped TFs Axes
* replace stereo module with depth module
* use rs2_to_ros to replace stereo module with depth moudle
* calculate extriniscs twice in two opposite ways to save inverting rotation matrix
* fix matrix rotation
* Merge branch 'ros2-development' into readme_fix
* invert translation
* Added 'publish_tf' param in rs launch files
* Indentation corrections
* Fix: Don't publish /tf when publish_tf is false
* use playback device for rosbags
* Avoid configuring dynamic_tf_broadcaster within tf_publish_rate param's callback
* Fix lower FPS in D405, D455
* update rs_launch.py to support enable_auto_exposure and manual exposure
* fix timestamp calculation metadata header to be aligned with metadata json timestamp
* Expose USB port in DeviceInfo service
* Use latched QoS for Extrinsic topic when intra-process is used
* add cppcheck to GHA
* Fix Apache License Header and Intel Copyrights
* apply copyrights and license on project
* Enable intra-process communication for point clouds
* Fix ros2 parameter descriptions and range values
* T265 clean up
* fix float_to_double method
* realsense2_camera/src/sensor_params.cpp
* remove T265 device from ROS Wrapper - step1
* Enable D457
* Fix hdr_merge filter initialization in ros2 launch
* if default profile is not defined, take the first available profile as default
* changed to static_cast and added descriptor name and type
* remove extra ';'
* remove unused variable format_str
* publish point cloud via unique shared pointer
* make source backward compatible to older versions of cv_bridge and rclcpp
* add hdr_merge.enable and depth_module.hdr_enabled to rs_launch.py
* fix compilation errors
* fix tabs
* if default profile is not defined, take the first available profile as default
* Fix ros2 sensor controls steps and add control default value to param description
* Publish static transforms when intra porocess communication is enabled
* Properly read camera config files in rs_launch.py
* fix deprecated API
* Add D457
* Windows bring-up
* publish actual IMU optical frame ID in IMU messages
* Publish static tf for IMU frames
* fix extrinsics calculation
* fix ordered_pc arg prefix
* publish IMU frames only if unite/sync imu method is not none
* Publish static tf for IMU frames
* add D430i support
* Contributors: Arun Prasad, Arun Prasad V, Arun-Prasad-V, Christian Rauch, Daniel Honies, Gilad Bretter, Nir Azkiel, NirAz, Pranav Dhulipala, Samer Khshiboun, SamerKhshiboun, Stephan Wirth, Xiangyu, Yadunund, nvidia

4.51.1 (2022-09-13)
-------------------
* Fix crash when activating IMU & aligned depth together
* Fix rosbag device loading by preventing set_option to HDR/Gain/Exposure
* Support ROS2 Humble
* Publish real frame rate of realsense camera node topics/publishers
* No need to start/stop sensors for align depth changes
* Fix colorizer filter which returns null reference ptr
* Fix align_depth enable/disable
* Add colorizer.enable to rs_launch.py
* Add copyright and license to all ROS2-beta source files
* Fix CUDA suffix for pointcloud and align_depth topics
* Add ROS build farm pre-release to ci

* Contributors: Eran, NirAz, SamerKhshiboun

4.0.4 (2022-03-20)
------------------
* fix required packages for building debians for ros2-beta branch

* Contributors: NirAz

4.0.3 (2022-03-16)
------------------
* Support intra-process zero-copy
* Update README
* Fix Galactic deprecated-declarations compilation warning
* Fix Eloquent compilation error

* Contributors: Eran, Nir-Az, SamerKhshiboun

4.0.2 (2022-02-24)
------------------
* version 4.4.0 changed to 4.0.0 in CHANGELOG
* add frequency monitoring to /diagnostics topic.
* fix topic_hz.py to recognize message type from topic name. (Naive)
* move diagnostic updater for stream frequencies into the RosSensor class.
* add frequency monitoring to /diagnostics topic.
* fix galactic issue with undeclaring parameters
* fix to support Rolling.
* fix dynamic_params syntax.
* fix issue with Galactic parameters set by default to static which prevents them from being undeclared.

* Contributors: Haowei Wen, doronhi, remibettan

4.0.1 (2022-02-01)
------------------
* fix reset issue when multiple devices are connected
* fix /rosout issue
* fix PID for D405 device
* fix bug: frame_id is based on camera_name
* unite_imu_method is now changeable in runtime.
* fix motion module default values.
* add missing extrinsics topics
* fix crash when camera disconnects.
* fix header timestamp for metadata messages.

* Contributors: nomumu, JamesChooWK, benlev, doronhi

4.0.0 (2021-11-17)
-------------------
* changed parameters: 
  - "stereo_module", "l500_depth_sensor" are replaced by "depth_module"
  - for video streams: <module>.profile replaces <stream>_width, <stream>_height, <stream>_fps
  - removed paramets <stream>_frame_id, <stream>_optical_frame_id. frame_ids are defined by camera_name
  - "filters" is removed. All filters (or post-processing blocks) are enabled/disabled using "<filter>.enable"
  - "align_depth" is replaced with "align_depth.enable"
  - "allow_no_texture_points", "ordered_pc" replaced by "pointcloud.allow_no_texture_points", "pointcloud.ordered_pc"
  - "pointcloud_texture_stream", "pointcloud_texture_index" are replaced by "pointcloud.stream_filter", "pointcloud.stream_index_filter"

* Allow enable/disable of sensors in runtime.
* Allow enable/disable of filters in runtime.

3.2.3 (2021-11-11)
------------------
* add parameter reconnect_timeout
* default frame_id includes namespace.
* Added dummy transformation for multi camera example
* add parameter wait_for_device_timeout
* Fix deprecation warnings when building on Rolling
* Make pointcloud_qos a configurable parameter
* show warning when requested profile cannot be selected.
* send only 4 distortion coeffs when using equidistant
* fixed missing std namespace
* Add ros2 github actions.
* add temperature diagnostics
* Add a parameter, diagnostics_period, to control if and how often will messages be published on the diagnostic topic.
* publish diagnostics topic for Asic and Projector temperature
* Support spaces in the filters Parameter string.
* publish metadata
* Add echo_metadada.py - An example script for subscribing and parsing metadata topics.
* Add service: device_info
* fixed device_name value to snake case
* Add device name.
* Contributors: Guillaume Doisy, Marenix, Nathan Brooks, Jacco van der Spek, doronhi

3.2.2 (2021-07-01)
------------------
* Support Galactic and Rolling
* Fix reading yaml config file
* No tf broadcaster object if publish_tf is set to false
* Add udev-rules installation to debian
* update ros2 launch examples (demo_t265_launch.py, rs_t265_launch.py, demo_pointcloud_launch.py)
* fix rs_multi_camera_launch.py to include default separate node names.
* Add support for L535
* QoS parameters to be applied for all publishers
* Imu_default QoS changed to hid_default
* Pointcloud_default and info_default QoS changed to qos_default
* Contributors: Guillaume Doisy, TSC21, anaelle, doronhi

3.2.1 (2021-05-10)
------------------
* Add build dependency **ros_environment**
* Contributors: doronhi

3.2.0 (2021-05-05)
------------------
* Added filling correct Tx, Ty values in projection matrix of right camera.
* Fixed frame_id of right sensor to match left sensor in a stereo pair.pair
* Contributors: Pavlo Kolomiiets, doronhi

3.1.6 (2021-05-05)
------------------
* Fix pointcloud message size when no texture is added.
* migration toggle_sensors feature to ros2
* Add option for aligning pointcloud to color image.
* removed option to align to other streams other then color.
* Contributors: anaelle, doronhi

3.1.5 (2021-03-24)
------------------
* Support Eloquent and Dashing.
* Add filter: HDR_merge
* fix initialization of colorizer image if user specified negative image size (as is in default launch file)
* Remove the following tests for known playback issue with librealsense2 version 2.43.0: align_depth_color_1, align_depth_ir1_1, align_depth_ir1_decimation_1.
* Remove wrong dependency
* changed default image QOS to SYSTEM_DEFAULT
* Add missing librealsense2 dependency from package.xml
* fix bug: selection of profile disregarded stream index.
* Contributors: changseung.yu, doronhi

3.1.4 (2021-02-18)
------------------
* fix reading json file with device other than D400 series.
* Publish depth confidence image for supporting devices (L515)
* Add selecting QoS option
* Import unit-tests
* fix timestamp domain issues
  - Add offset to ros_time only if device uses hardware-clock. Otherwise use device time - either system_time or global_time.
  - Warn of a hardware timestamp possible loop.
* Choose the default profile in case of an invalid request.
* Avoid aligning confidence image.
* Add an option for an Ordered PointCloud.
* Contributors: Gabriel Urbain, Isaac I.Y. Saito, Itamar Eliakim, Marc Alban, doronhi

3.1.3 (2020-12-28)
------------------
* Publish depth confidence image for supporting devices (L515)
* fix bug: dynamic tf publisher.
* fix realsense2_description's dependency to realsense2_camera_msgs
* remove boost dependency.
* rename node_namespace to namespace
* rename node_executable to executable
* Contributors: doronhi

3.1.2 (2020-12-10)
------------------
* upgrade librealsense2 version to 2.40.0
* Added pointcloud attributes, when RS2_STREAM_ANY is enabled
* colorize the aligned depth image.
  fix missing parameters description.
* add infra_rgb option: enable streaming D415's infra stream as RGB.
  Expose stereo extrinsics
  Avoid currently unsupported Z16H depth format.
  Enable launch cmd line params
  Add notification if connected using USB2.1 port.
* Fix README.md
* Add launch option: output:=[screen|log]: enable sending logs to ros log file.
* rename rs.d400_and_t265.launch.py to rs_d400_and_t265_launch.py
  add rs_multi_camera_launch.py on top of rs_launch.py
* Remove '_' prefix from parameter serial_no. Allow to pass serial_no which is sometimes a string containing only digits.
* remove redundant nav_msgs dependency (caused build error)
* remove node package, change msg package name
* update README.md
* Contributors: Ryan Shim, doronhi

3.1.1 (2020-09-23)
------------------
* fix bug: Conversion from milliseconds to nanoseconds.
  enable use of parameter: use_sim_time.
* various fixes for canonical ROS2
* Contributors: AustinDeric, doronhi

3.1.0 (2020-09-16)
------------------
* port support of T265 from ROS1.
* Contributors: doronhi

3.0.0 (2020-09-16)
------------------
* Update README.md
* Enable recovery from reconnect event.
* Add an example config file: d435i.yaml.
* fix launch file installation. Allow running the following command: 'ros2 launch realsense2_camera rs.launch.py'
* enable set_auto_exposure_roi options.
* enable sensors and filters dynamic parameters.
* clean
* RealSenseNodeFactory inherits from rclcpp::Node
* Add project: realsense2_node
* Add support for D455
* README.md: update installation process.
