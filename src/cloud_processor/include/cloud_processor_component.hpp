#ifndef CLOUD_PROCESSOR_HPP
#define CLOUD_PROCESSOR_HPP

// Standard library includes
#include <chrono>
using namespace std::chrono_literals;

#include <functional>
#include <memory>
#include <string>
#include <sstream>

// Third-party libraries for point cloud processing and transformations
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// ROS application/library includes for node management, messaging, and transformations
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>

// Custom includes
#include "visibility_control.hpp"
#include "utilities.h"

namespace zion
{
  /**
   * @brief CloudProcessor class responsible for processing point cloud data in ROS 2.
   * The class handles incoming point cloud data, applies transformations, voxel downsampling, and cropping.
   * The processed data is then published to another topic.
   */
  class CloudProcessor : public rclcpp::Node
  {
  public:

    /**
     * @brief Constructor for the CloudProcessor class.
     * Initializes the node, sets up parameters, subscribers, and initializes transformation utilities.
     * @param options Node options for initializing the ROS 2 node.
     */
    ZION_COMPONENTS_PUBLIC
    explicit CloudProcessor(const rclcpp::NodeOptions & options);
    
    /**
     * @brief Destructor for the CloudProcessor class.
     * Handles cleanup when the node goes out of scope or is destroyed.
     */
    virtual ~CloudProcessor();

  protected:

    /**
     * @brief Callback function for handling incoming point cloud messages.
     * Processes the point cloud data by applying transformations, voxel downsampling, and cropping.
     * Publishes the processed data to another topic.
     * @param pcl_msg Shared pointer to the incoming point cloud message.
     */
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg);
    
    /**
     * @brief Loads parameters related to voxel filtering, cropping, topic names, and frame IDs.
     * Each parameter is logged after loading.
     */
    void loadParams();

  private:

    // Buffer and listener for handling transformation data between different coordinate frames
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // Subscription to the input point cloud topic
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;

    // Publisher for the processed point cloud data
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_buffer_; 

    // Topic names for the input and processed point cloud data
    std::string input_point_cloud_topic_;
    std::string filtered_point_cloud_topic_;

    // Frame IDs for the input and output point clouds
    std::string input_frame_;
    std::string output_frame_;

    // Transformation matrix.
    Eigen::Affine3d c2cp;
    
    // Parameters for voxel filtering
    double leaf_size_xy_;
    double leaf_size_z_;
    
    // Parameters defining the crop box's dimensions
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double min_z_;
    double max_z_;
  };
}  // namespace zion

#endif //  CLOUD_PROCESSOR
