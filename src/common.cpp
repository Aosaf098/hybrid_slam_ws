/*
A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3
Decoupled/Hybrid Split Modification

Author: Azmyin Md. Kamal (Adapted by aosaf)
Date: 01/01/24 (Modified: June 2026)

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file
*/

//* Includes
#include "ros2_orb_slam3/common.hpp"
#include "hybrid_slam_interfaces/msg/feature_frame.hpp" //* Custom lightweight feature message structure

//* Latency instrumentation includes (Metric 3)
#include <chrono>
#include <fstream>

//* Constructor
MonocularMode::MonocularMode() : Node("mono_node_cpp")
{
    // Declare parameters to be passed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED (HYBRID MODE)");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overridden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; 
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; 
    
    //* UPDATED ARCHITECTURE: Topic changed from full images to pre-extracted features channel
    subImgMsgName = "/mono_py_driver/feature_msg"; 
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; 

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* UPDATED SUBSCRIBER: Changes sensor_msgs::msg::Image to custom hybrid_slam_interfaces::msg::FeatureFrame
    subFeatureMsg_subscription_ = this->create_subscription<hybrid_slam_interfaces::msg::FeatureFrame>(subImgMsgName, 1, std::bind(&MonocularMode::Feature_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    pAgent->Shutdown();
}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout << "Sent response: " << message.data.c_str() << std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);
}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); 

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; 
    enableOpenCVWindow = true; 
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; 
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    timeStep = time_msg.data;
}

//* UPDATED CALLBACK ENGINE: Processes raw memory buffers back into structured keypoint vectors and matrices
void MonocularMode::Feature_callback(const hybrid_slam_interfaces::msg::FeatureFrame::SharedPtr msg)
{
    //* ---------------- Latency instrumentation (Metric 3) ----------------
    //* Initialized once on first call; persists across callbacks via 'static'.
    static std::ofstream latency_csv(std::string(getenv("HOME")) + "/hybrid_slam_ws/decoupled_latency_mh01.csv");
    static bool header_written = false;
    static int frames_received = 0;   // every frame entering the callback
    static int frames_tracked  = 0;   // frames that completed tracking
    if (!header_written) {
        latency_csv << "frame_idx,num_features,transport_ms,tracking_ms,e2e_ms\n";
        header_written = true;
    }
    frames_received++;

    //* Transport latency: publish time (header.stamp, set in Python driver) -> callback entry.
    //* Computed in raw nanoseconds to avoid clock-source mismatch exceptions.
    double transport_ms =
        (this->now().nanoseconds() - rclcpp::Time(msg->header.stamp).nanoseconds()) / 1.0e6;

    int num_features = msg->num_features;
    if (num_features == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty feature frame, skipping tracking.");
        return; 
    }
    
    // 1. Reconstruct the vector of OpenCV KeyPoints from sequential message coordinate arrays
    std::vector<cv::KeyPoint> vKeyPoints;
    vKeyPoints.reserve(num_features);
    for (int i = 0; i < num_features; i++) {
        cv::KeyPoint kp;
        kp.pt.x  = msg->kps_x[i];
        kp.pt.y  = msg->kps_y[i];
        kp.size  = msg->kps_size[i];
        kp.angle = msg->kps_angle[i];
        vKeyPoints.push_back(kp);
    }
    
    // 2. Map the flat uint8 vector straight back into a row-major matrix of descriptors (num_features rows x 32 columns)
    // CV_8UC1 means 8-bit unsigned single-channel characters (bytes)
    cv::Mat mDescriptors(num_features, 32, CV_8UC1, const_cast<uint8_t*>(&msg->descriptors[0]));
    
    // 3. Perform tracking operations using pre-extracted parameters (TIMED)
    // Note: If your current pAgent instance hasn't had its core Tracking.cc headers modified to accept keypoint signatures natively yet, 
    // uncomment the line below after editing Tracking.cc to host your custom Entry Point API!
    
    auto t0 = std::chrono::steady_clock::now();
    Sophus::SE3f Tcw = pAgent->TrackMonocularFeatures(vKeyPoints, mDescriptors, timeStep); 
    auto t1 = std::chrono::steady_clock::now();
    double tracking_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    //* ---------------- Log per-frame latency (Metric 3) ----------------
    frames_tracked++;
    double e2e_ms = transport_ms + tracking_ms;
    latency_csv << frames_tracked << "," << num_features << ","
                << transport_ms << "," << tracking_ms << "," << e2e_ms << "\n";
    latency_csv.flush();
}