#ifndef ROS2_SLAM_PUBLISHER_H
#define ROS2_SLAM_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <thread>
#include <mutex>

#include "SlamDataProcess.h"


namespace ORB_SLAM3 {

class SlamDataProcess;  // Forward declaration

class ROS2SlamPublisher
{
public:
    ROS2SlamPublisher(SlamDataProcess* slam_data_processor);
    ~ROS2SlamPublisher();

    // Initialize ROS2 publishers and parameters
    void Initialize();
    
    // Start publishing threads
    void Run();
    
    // Control functions
    void RequestFinish();
    bool IsFinished();
    
    // Get the ROS2 node for external use
    std::shared_ptr<rclcpp::Node> GetNode() const { return node_; }

private:
    // ROS2 node and publishers
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr camera_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher frame_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> map_to_slam_map_broadcaster_;
    
    // Data processor reference
    SlamDataProcess* slam_data_processor_;
    
    // Threading
    std::thread pose_thread_;
    std::thread pointcloud_thread_;
    std::thread pointcloud_mapping_thread_;
    std::thread frame_thread_;
    std::thread trajectory_thread_;
    
    // Control
    bool finish_requested_;
    bool finished_;
    std::mutex finish_mutex_;
    
    // ROS2 Parameters
    double publish_rate_hz_;
    std::string camera_frame_;
    std::string ground_frame_;
    std::string vehicle_frame_;
    
    // Publishing thread functions
    void PublishPoseData();
    void PublishPointCloudData();
    void PublishFrameData();
    void PublishPointCloudMappingData();
    void PublishTrajectoryData();
    void ROS2SlamMapToSlamMapStaticTransform();
    
    // Helper functions
    void LoadROSParameters();
    geometry_msgs::msg::PoseStamped EigenMatrixToPoseStamped(const Eigen::Matrix4f& matrix, 
                                                             const std::string& frame_id);
    sensor_msgs::msg::PointCloud2 PCLToROS(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
                                           const std::string& frame_id);
    // sensor_msgs::msg::PointCloud2 PCLToROS_RGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
    //                                        const std::string& frame_id);
    nav_msgs::msg::Path TrajectoryToPath(const std::vector<Eigen::Matrix4f>& trajectory,
                                        const std::string& frame_id);
};

} // namespace ORB_SLAM3

#endif // ROS2_SLAM_PUBLISHER_H
