#!/usr/bin/env python3

"""
Python node for the MonocularMode cpp node.
HYBRID DECOUPLED ARCHITECTURE: Extracts ORB Features before publishing.

Author: Azmyin Md. Kamal (Modified by aosaf)
Date: 01/01/2024 (Modified June 2026)
"""

import sys
import os
import time
import natsort
import numpy as np
import cv2 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from hybrid_slam_interfaces.msg import FeatureFrame 
from pathlib import Path

class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name)

        self.declare_parameter("settings_name","EuRoC")
        self.declare_parameter("image_seq","NULL")

        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.image_seq = str(self.get_parameter('image_seq').value)

        self.home_dir = str(Path.home()) + "/hybrid_slam_ws/src/ros2_orb_slam3" 
        self.parent_dir = "TEST_DATASET" 
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq 

        self.node_name = "mono_py_driver"
        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(self.image_sequence_dir, "mav0") 

        # Initialize the OpenCV ORB Extractor
        self.orb = cv2.ORB_create(nfeatures=1000)

        # ROS2 Configurations
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_feature_to_agent_name = "/mono_py_driver/feature_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.send_config = True 
        
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1) 
        self.subscribe_exp_ack_ = self.create_subscription(String, self.sub_exp_ack_name, self.ack_callback, 10)
        self.publish_feature_msg_ = self.create_publisher(FeatureFrame, self.pub_feature_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)

        self.start_frame = 0 
        self.end_frame = -1 
        self.frame_stop = -1 
        self.frame_id = 0 

        # --- Latency instrumentation (Metric 3): extraction timing log ---
        self.extract_log = open(str(Path.home()) + "/hybrid_slam_ws/decoupled_extract_mh01.csv", "w")
        self.extract_log.write("frame_id,num_features,extract_ms\n")
        self.extract_log.flush()

    def get_image_dataset_asl(self, exp_dir, agent_name = "mav0"):
        imgz_file_list = []
        time_list = []
        agent_cam0_fld = exp_dir + "/" + agent_name + "/" + "cam0"
        imgz_file_dir = agent_cam0_fld + "/" + "data" + "/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir),reverse=False)

        for iox in imgz_file_list:
            time_step = iox.split(".")[0]
            time_list.append(time_step)

        return imgz_file_dir, imgz_file_list, time_list

    def ack_callback(self, msg):
        if(msg.data == "ACK"):
            self.send_config = False
    
    def handshake_with_cpp_node(self):
        if (self.send_config == True):
            msg = String()
            msg.data = self.settings_name
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    
    def run_py_node(self, idx, imgz_name):
        img_look_up_path = self.imgz_seqz_dir  + imgz_name
        timestep = float(imgz_name.split(".")[0]) 
        self.frame_id += 1  

        # 1. Read Image
        cv_img = cv2.imread(img_look_up_path, cv2.IMREAD_GRAYSCALE)
        
        # 2. Extract Features locally (TIMED - Metric 3)
        t_ex0 = time.perf_counter()
        keypoints, descriptors = self.orb.detectAndCompute(cv_img, None)
        t_extract_ms = (time.perf_counter() - t_ex0) * 1000.0

        # 3. Pack ROS Message
        feature_msg = FeatureFrame()
        feature_msg.header.stamp = self.get_clock().now().to_msg()
        feature_msg.header.frame_id = "hybrid_cam_frame"

        if keypoints is not None and descriptors is not None:
            feature_msg.num_features = len(keypoints)
            feature_msg.kps_x = [float(kp.pt[0]) for kp in keypoints]
            feature_msg.kps_y = [float(kp.pt[1]) for kp in keypoints]
            feature_msg.kps_size = [float(kp.size) for kp in keypoints]
            feature_msg.kps_angle = [float(kp.angle) for kp in keypoints]

            # Fast C-level byte packing prevents ROS 2 silent crashes
            feature_msg.descriptors = descriptors.tobytes()
        else:
            feature_msg.num_features = 0

        # 4. Stream Data
        timestep_msg = Float64()
        timestep_msg.data = timestep

        self.publish_timestep_msg_.publish(timestep_msg) 
        self.publish_feature_msg_.publish(feature_msg)

        # --- Log extraction latency (Metric 3) ---
        self.extract_log.write(f"{self.frame_id},{feature_msg.num_features},{t_extract_ms:.4f}\n")
        self.extract_log.flush()
        
        print(f"Streamed Frame {self.frame_id}: {feature_msg.num_features} features published.")

def main(args = None):
    rclpy.init(args=args) 
    n = MonoDriver("mono_py_node") 
    rate = n.create_rate(20) 
    
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        
    print(f"Handshake complete. Commencing decoupled feature stream...")

    # ---------------------------------------------------------
    # NEW FIX: Give the C++ engine 5 seconds to load the Vocabulary!
    print("Waiting 5 seconds for C++ backend to boot...")
    time.sleep(5)
    # ---------------------------------------------------------

    # Safe slicing logic
    frames_to_process = n.imgz_seqz[n.start_frame:] if n.end_frame == -1 else n.imgz_seqz[n.start_frame:n.end_frame]
    print(f"Total images found: {len(frames_to_process)}")

    for idx, imgz_name in enumerate(frames_to_process):
        try:
            rclpy.spin_once(n) 
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            if (n.frame_id > n.frame_stop and n.frame_stop != -1):
                break
        
        except KeyboardInterrupt:
            break

    # --- Close instrumentation log cleanly (Metric 3) ---
    try:
        n.extract_log.close()
    except Exception:
        pass

    cv2.destroyAllWindows() 
    n.destroy_node() 
    rclpy.shutdown()

if __name__=="__main__":
    main()