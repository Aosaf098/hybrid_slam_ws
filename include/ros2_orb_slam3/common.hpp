/*
* Originally adapted from ORB-SLAM3: Examples/ROS/include/common.h
* Author: Azmyin Md. Kamal (Modified by aosaf)
* Version: 2.0 (Hybrid Split)
* Date: June 2026
* Compatible for ROS2 Humble & custom feature messaging layout
*/

#ifndef COMMON_HPP
#define COMMON_HPP

// System Includes
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS2 Core Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

// Custom Interface Messages
#include "hybrid_slam_interfaces/msg/feature_frame.hpp"

// ORB-SLAM3 Core Engine Headers
#include "System.h"

// Placeholder bindings shortcut namespaces
using std::placeholders::_1;

// Global constants relative path from home directory
const std::string packagePath = "hybrid_slam_ws/src/ros2_orb_slam3/";

class MonocularMode : public rclcpp::Node
{
public:
    // Constructor and Destructor
    MonocularMode();
    ~MonocularMode();

    // Configuration Callback Routine
    void experimentSetting_callback(const std_msgs::msg::String& msg);

    // Timeline Callback Routine
    void Timestep_callback(const std_msgs::msg::Float64& time_msg);

    // UPDATED SUBSCRIPTION ENTRY: Receives decoupled feature coordinate/descriptor arrays
    void Feature_callback(const hybrid_slam_interfaces::msg::FeatureFrame::SharedPtr msg);

private:
    // Core engine worker bindings initializer method
    void initializeVSLAM(std::string& configString);

    // Subscription Objects Handles
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
    rclcpp::Subscription<hybrid_slam_interfaces::msg::FeatureFrame>::SharedPtr subFeatureMsg_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

    // Publisher Object Handle
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;

    // Internal Environment Properties Strings
    std::string homeDir;
    std::string nodeName;
    std::string vocFilePath;
    std::string settingsFilePath;
    std::string experimentConfig;
    std::string receivedConfig;

    // Direct string handles for targeted subscription topic routes
    std::string subexperimentconfigName;
    std::string pubconfigackName;
    std::string subImgMsgName;
    std::string subTimestepMsgName;

    // Environment Flags
    bool bSettingsFromPython = false;
    double timeStep;

    // Local pointers pointing cleanly into the underlying unmanaged C++ SLAM core engine
    ORB_SLAM3::System::eSensor sensorType;
    ORB_SLAM3::System *pAgent;
    bool enablePangolinWindow;
    bool enableOpenCVWindow;
};

#endif // COMMON_HPP