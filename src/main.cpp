#include "orbslam3_dense_ros2/ros2_slam_publisher.h"
#include "SlamDataProcess.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <signal.h>
#include <cstdlib>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Global variables for graceful shutdown
std::shared_ptr<ORB_SLAM3::ROS2SlamPublisher> g_ros2_publisher;
ORB_SLAM3::SlamDataProcess* g_slam_data_processor = nullptr;
ORB_SLAM3::System* g_slam_system = nullptr;

void signal_handler(int signal) {
    std::cout << "Interrupt signal (" << signal << ") received. Shutting down..." << std::endl;
    
    if (g_ros2_publisher) {
        g_ros2_publisher->RequestFinish();
    }
    
    if (g_slam_data_processor) {
        g_slam_data_processor->RequestFinish();
    }
    
    if (g_slam_system) {
        g_slam_system->Shutdown();
    }
    
    rclcpp::shutdown();
}

// Main ROS2 node class
class ORBSlamNode : public rclcpp::Node
{
public:
    ORBSlamNode(int argc, char** argv) : Node("orb_slam3_ros2_node")
    {
        // Initialize parameters from command line arguments
        std::string voc_file = "/home/zgf/test/src/orbslam3_dense_ros2/orb_slam3/Vocabulary/ORBvoc.txt.bin";
        std::string settings_file = "/home/zgf/test/src/orbslam3_dense_ros2/orb_slam3/config/RGB-D/RealSense_D435i.yaml";
        bool enable_pangolin = true;
        std::string rgb_topic = "/camera/camera/color/image_raw";
        std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";
        
        // Parse command line arguments
        if (argc >= 2) voc_file = argv[1];
        if (argc >= 3) settings_file = argv[2];
        if (argc >= 4) enable_pangolin = (std::string(argv[3]) == "true");
        if (argc >= 5) rgb_topic = argv[4];
        if (argc >= 6) depth_topic = argv[5];
        
        // Declare parameters
        this->declare_parameter("vocabulary_file", voc_file);
        this->declare_parameter("settings_file", settings_file);
        this->declare_parameter("enable_pangolin", enable_pangolin);
        this->declare_parameter("rgb_topic", rgb_topic);
        this->declare_parameter("depth_topic", depth_topic);
        
        // Get parameters (allow override via ROS parameters)
        voc_file = this->get_parameter("vocabulary_file").as_string();
        settings_file = this->get_parameter("settings_file").as_string();
        enable_pangolin = this->get_parameter("enable_pangolin").as_bool();
        rgb_topic = this->get_parameter("rgb_topic").as_string();
        depth_topic = this->get_parameter("depth_topic").as_string();
        
        // Validate required parameters
        if (voc_file.empty() || settings_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide vocabulary_file and settings_file parameters");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", voc_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings file: %s", settings_file.c_str());
        RCLCPP_INFO(this->get_logger(), "RGB topic: %s", rgb_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic.c_str());
        
        try {
            // Initialize ORB-SLAM3 System
            RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3 system...");
            slam_system_ = new ORB_SLAM3::System(
                voc_file, 
                settings_file, 
                ORB_SLAM3::System::RGBD, 
                enable_pangolin
            );
            g_slam_system = slam_system_;
            
            RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 system initialized successfully");
            
            // Get system components using public getter functions
            frame_drawer_ = slam_system_->GetFrameDrawer();
            map_drawer_ = slam_system_->GetMapDrawer();
            tracking_ = slam_system_->GetTracker();
            
            // Initialize SLAM data processor (pure data processing, no ROS2)
            // Use getter functions to access previously private members
            slam_data_processor_ = new ORB_SLAM3::SlamDataProcess(
                slam_system_, frame_drawer_, map_drawer_, tracking_, settings_file, slam_system_->GetAtlas()
            );
            g_slam_data_processor = slam_data_processor_;
            
            // 可选：启动SlamDataPub线程（如有需要）
            slam_data_pub_thread_ = std::make_unique<std::thread>(&ORB_SLAM3::SlamDataProcess::Run, slam_data_processor_);
            
            // 设置Tracking与SlamDataPub的关联（如接口存在）
            if (tracking_) {
                tracking_->SetSlamDataPub(slam_data_processor_);
            }
            
            // Initialize ROS2 publisher (ROS2-only, interfaces with SlamDataProcess)
            ros2_publisher_ = std::make_shared<ORB_SLAM3::ROS2SlamPublisher>(slam_data_processor_);
            g_ros2_publisher = ros2_publisher_;
            
            // Create synchronized RGB and Depth subscribers
            rgb_subscriber_.subscribe(this, rgb_topic);
            depth_subscriber_.subscribe(this, depth_topic);
            
            // Create synchronizer
            sync_policy_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(1), rgb_subscriber_, depth_subscriber_);
            sync_policy_->registerCallback(
                std::bind(&ORBSlamNode::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            
            // Start the ROS2 publisher
            ros2_publisher_->Run();
            
            RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 node started successfully");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    ~ORBSlamNode()
    {
        if (ros2_publisher_) {
            ros2_publisher_->RequestFinish();
        }
        
        if (slam_system_) {
            slam_system_->Shutdown();
            delete slam_system_;
        }
        
        if (slam_data_processor_) {
            delete slam_data_processor_;
        }
    }
    
private:
    void rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, 
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        try {
            // Convert ROS images to OpenCV images
            cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

            // Use RGB timestamp as reference (they should be synchronized)
            double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;
            
            // Track RGBD frame
            Sophus::SE3f pose = slam_system_->TrackRGBD(rgb_ptr->image, depth_ptr->image, timestamp);
            
            // // Update SLAM data processor with new pose
            // if (!pose.matrix().isZero()) {
            //     // Convert Sophus pose to cv::Mat for SlamDataProcess
            //     cv::Mat cv_pose = cv::Mat::eye(4, 4, CV_32F);
            //     Eigen::Matrix4f eigen_pose = pose.matrix();
            //     for (int i = 0; i < 4; i++) {
            //         for (int j = 0; j < 4; j++) {
            //             cv_pose.at<float>(i, j) = eigen_pose(i, j);
            //         }
            //     }
            //     slam_data_processor_->SetCurrentCameraPose(cv_pose);
            // }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGBD images: %s", e.what());
        }
    }
    
    // ORB-SLAM3 components
    ORB_SLAM3::System* slam_system_;
    ORB_SLAM3::FrameDrawer* frame_drawer_;
    ORB_SLAM3::MapDrawer* map_drawer_;
    ORB_SLAM3::Tracking* tracking_;
    ORB_SLAM3::SlamDataProcess* slam_data_processor_;
    
    // ROS2 components
    std::shared_ptr<ORB_SLAM3::ROS2SlamPublisher> ros2_publisher_;
    std::unique_ptr<std::thread> slam_data_pub_thread_;
    
    // Synchronization components
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_policy_;
};

int main(int argc, char** argv)
{
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        // Create and run the node with command line arguments
        auto node = std::make_shared<ORBSlamNode>(argc, argv);
        
        RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 ROS2 system started. Waiting for synchronized RGB+Depth images...");
        
        // Spin the node
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "ORB-SLAM3 ROS2 system shut down cleanly." << std::endl;
    return 0;
}
