#ifndef CLOUD_PROCESSOR_HPP
#define CLOUD_PROCESSOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "visibility_control.hpp"
#include "utilities.h"


using namespace std::chrono_literals;

#include <functional>
#include <memory>
#include <sstream>
#include <string>

namespace zion
{
  class CloudProcessor : public rclcpp::Node
  {
  public:
    ZION_COMPONENTS_PUBLIC
    explicit CloudProcessor(const rclcpp::NodeOptions & options);
    virtual ~CloudProcessor();

  protected:
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg);

    void loadParams();

  private:

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;

    //publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_; ///< Publisher for the filtered pcl.
    std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_buffer_;

    Eigen::Affine3d c2cp;

    std::string input_point_cloud_topic_;
    std::string filtered_point_cloud_topic_;


    std::string input_frame_;
    std::string output_frame_;

    double leaf_size_xy_;
    double leaf_size_z_;
    
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double min_z_;
    double max_z_;
  };
}  // namespace composition

#endif //  CLOUD_PROCESSOR
