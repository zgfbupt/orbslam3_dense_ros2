#include "orbslam3_dense_ros2/ros2_slam_publisher.h"
#include <chrono>

namespace ORB_SLAM3 {

ROS2SlamPublisher::ROS2SlamPublisher(SlamDataProcess* slam_data_processor)
    : slam_data_processor_(slam_data_processor)
    , finish_requested_(false)
    , finished_(true)
    , publish_rate_hz_(30.0)
    , camera_frame_("camera")
    , ground_frame_("map")
    , vehicle_frame_("vehicle")
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("orbslam3_ros2_publisher");
    
    LoadROSParameters();
}

ROS2SlamPublisher::~ROS2SlamPublisher()
{
    RequestFinish();
    
    if (pose_thread_.joinable()) pose_thread_.join();
    if (pointcloud_thread_.joinable()) pointcloud_thread_.join();
    if (frame_thread_.joinable()) frame_thread_.join();
    if (trajectory_thread_.joinable()) trajectory_thread_.join();
}

void ROS2SlamPublisher::LoadROSParameters()
{
    // Declare and get ROS2 parameters
    node_->declare_parameter<double>("publish_rate_hz", 30.0);
    node_->declare_parameter<std::string>("camera_frame", "camera_link");
    node_->declare_parameter<std::string>("ground_frame", "odom");
    node_->declare_parameter<std::string>("vehicle_frame", "base_footprint");
    
    publish_rate_hz_ = node_->get_parameter("publish_rate_hz").as_double();
    camera_frame_ = node_->get_parameter("camera_frame").as_string();
    ground_frame_ = node_->get_parameter("ground_frame").as_string();
    vehicle_frame_ = node_->get_parameter("vehicle_frame").as_string();
    
    RCLCPP_INFO(node_->get_logger(), "ROS2 Publisher initialized with rate: %.1f Hz", publish_rate_hz_);
}

void ROS2SlamPublisher::Initialize()
{
    // Initialize publishers
    camera_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    vehicle_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose", 10);
    camera_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
    vehicle_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("vehicle_path", 10);
    all_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_all", 10);
    ref_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_ref", 10);
    map_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("cloudPointMapping", 10);
    
    // Initialize image transport and TF broadcaster
    image_transport_ = std::make_shared<image_transport::ImageTransport>(node_);
    frame_pub_ = image_transport_->advertise("current_frame", 1);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);


    ROS2SlamMapToSlamMapStaticTransform();

    RCLCPP_INFO(node_->get_logger(), "ROS2 Publishers initialized");
}

void ROS2SlamPublisher::Run()
{
    finished_ = false;
    
    Initialize();
    
    // Start publishing threads
    pose_thread_ = std::thread(&ROS2SlamPublisher::PublishPoseData, this);
    pointcloud_thread_ = std::thread(&ROS2SlamPublisher::PublishPointCloudData, this);
    pointcloud_mapping_thread_ = std::thread(&ROS2SlamPublisher::PublishPointCloudMappingData, this);
    frame_thread_ = std::thread(&ROS2SlamPublisher::PublishFrameData, this);
    trajectory_thread_ = std::thread(&ROS2SlamPublisher::PublishTrajectoryData, this);
    
    RCLCPP_INFO(node_->get_logger(), "ROS2 Publisher threads started");
}

void ROS2SlamPublisher::ROS2SlamMapToSlamMapStaticTransform()
{
    map_to_slam_map_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node_->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ground_frame_;

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.065;

    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    map_to_slam_map_broadcaster_->sendTransform(transformStamped);
}

void ROS2SlamPublisher::PublishPoseData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto pose_data = slam_data_processor_->GetCurrentPoseData();
        
        if (pose_data.has_new_pose)
        {
            // Publish camera pose
            auto camera_pose_msg = EigenMatrixToPoseStamped(pose_data.cam_pose_to_ground, ground_frame_);
            camera_pose_pub_->publish(camera_pose_msg);
            
            // Publish vehicle pose
            auto vehicle_pose_msg = EigenMatrixToPoseStamped(pose_data.vehicle_pose_to_ground, ground_frame_);
            vehicle_pose_pub_->publish(vehicle_pose_msg);
            
            // Publish TF transform
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = node_->now();
            transform_stamped.header.frame_id = ground_frame_;
            transform_stamped.child_frame_id = vehicle_frame_;

            transform_stamped.transform.translation.x = vehicle_pose_msg.pose.position.x;
            transform_stamped.transform.translation.y = vehicle_pose_msg.pose.position.y;
            transform_stamped.transform.translation.z = vehicle_pose_msg.pose.position.z;
            transform_stamped.transform.rotation = vehicle_pose_msg.pose.orientation;
            
            tf_broadcaster_->sendTransform(transform_stamped);
        }
        
        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishPointCloudMappingData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    while (!finish_requested_ && slam_data_processor_)
    {
        auto pc_data = slam_data_processor_->GetCurrentPointCloudMappingData();

        if (pc_data.has_new_map)
        {
            if (pc_data.map_points && pc_data.map_points->points.size() > 0)
            {
                auto map_points_msg = PCLToROS(pc_data.map_points, ground_frame_);
                map_points_pub_->publish(map_points_msg);
            }
        }

        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishPointCloudData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(2000.0 / publish_rate_hz_)); // Slower rate for point clouds
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto pc_data = slam_data_processor_->GetCurrentPointCloudData();
        
        if (pc_data.has_new_cloud)
        {
            if (pc_data.all_points && pc_data.all_points->points.size() > 0)
            {
                auto all_points_msg = PCLToROS(pc_data.all_points, ground_frame_);
                all_points_pub_->publish(all_points_msg);
            }
            
            if (pc_data.ref_points && pc_data.ref_points->points.size() > 0)
            {
                auto ref_points_msg = PCLToROS(pc_data.ref_points, ground_frame_);
                ref_points_pub_->publish(ref_points_msg);
            }
        }
        
        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishFrameData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto frame_data = slam_data_processor_->GetCurrentFrameData();

        if (frame_data.has_new_frame && !frame_data.current_frame.empty())
        {
            cv_bridge::CvImage cv_image;
            cv_image.header.frame_id = camera_frame_;
            cv_image.header.stamp = node_->now();
            cv_image.encoding = "bgr8";
            cv_image.image = frame_data.current_frame;
            
            sensor_msgs::msg::Image ros_image;
            cv_image.toImageMsg(ros_image);
            frame_pub_.publish(ros_image);
        }
        
        std::this_thread::sleep_for(period);
    }
}

void ROS2SlamPublisher::PublishTrajectoryData()
{
    auto period = std::chrono::milliseconds(static_cast<int>(5000.0 / publish_rate_hz_)); // Even slower for trajectories
    
    while (!finish_requested_ && slam_data_processor_)
    {
        auto traj_data = slam_data_processor_->GetCurrentTrajectoryData();
        
        if (traj_data.has_new_trajectory)
        {
            if (!traj_data.camera_trajectory.empty())
            {
                auto camera_path = TrajectoryToPath(traj_data.camera_trajectory, ground_frame_);
                camera_path_pub_->publish(camera_path);
            }
            
            if (!traj_data.vehicle_trajectory.empty())
            {
                auto vehicle_path = TrajectoryToPath(traj_data.vehicle_trajectory, ground_frame_);
                vehicle_path_pub_->publish(vehicle_path);
            }
        }
        
        std::this_thread::sleep_for(period);
    }
}

geometry_msgs::msg::PoseStamped ROS2SlamPublisher::EigenMatrixToPoseStamped(const Eigen::Matrix4f& matrix, 
                                                                            const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    
    pose_msg.pose.position.x = matrix(0, 3);
    pose_msg.pose.position.y = matrix(1, 3);
    pose_msg.pose.position.z = matrix(2, 3);
    
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    pose_msg.header.frame_id = frame_id;
    pose_msg.header.stamp = node_->now();
    
    return pose_msg;
}

sensor_msgs::msg::PointCloud2 ROS2SlamPublisher::PCLToROS(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
                                                          const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = frame_id;
    ros_cloud.header.stamp = node_->now();
    return ros_cloud;
}

nav_msgs::msg::Path ROS2SlamPublisher::TrajectoryToPath(const std::vector<Eigen::Matrix4f>& trajectory,
                                                       const std::string& frame_id)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = node_->now();
    
    for (const auto& pose_matrix : trajectory)
    {
        auto pose_stamped = EigenMatrixToPoseStamped(pose_matrix, frame_id);
        path_msg.poses.push_back(pose_stamped);
    }
    
    return path_msg;
}

void ROS2SlamPublisher::RequestFinish()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    finish_requested_ = true;
}

bool ROS2SlamPublisher::IsFinished()
{
    std::unique_lock<std::mutex> lock(finish_mutex_);
    return finished_;
}

} // namespace ORB_SLAM3
