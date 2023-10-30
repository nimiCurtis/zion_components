#ifndef ZION_BROADCASTER_HPP
#define ZION_BROADCASTER_HPP

// Standard library includes
#include <chrono>
using namespace std::chrono_literals;

#include <functional>
#include <memory>
#include <string>
#include <sstream>

// Third party libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// ROS application/library includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom includes
#include "visibility_control.hpp"

namespace zion
{
  /**
   * @brief ZionBroadcaster class responsible for broadcasting transformation data in ROS 2.
   * 
   * The class handles transformation data and broadcasts a "projected" version of a transformation.
   * It subscribes to the /tf topic for transformation data and processes it to broadcast a new transformation.
   */
  class ZionBroadcaster : public rclcpp::Node
  {
    public:

      /**
       * @brief Constructor for the ZionBroadcaster class.
       * Initializes the node, sets up parameters, subscribers, and initializes transformation utilities.
       * @param options Node options for initializing the ROS 2 node.
       */
      ZION_COMPONENTS_PUBLIC
      explicit ZionBroadcaster(const rclcpp::NodeOptions & options);
      
      /**
       * @brief Destructor for the ZionBroadcaster class.
       * Handles cleanup when the node goes out of scope or is destroyed.
       */
      virtual ~ZionBroadcaster();

    protected:

      /**
       * @brief Callback function for handling incoming transformation messages on the /tf topic.
       * Processes the transformation message, checks for available transforms, and broadcasts a new "projected" transform.
       * @param tf_msg Shared pointer to the transformation message received on the /tf topic.
       */
      void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg);
      
      /**
       * @brief Loads frame IDs from the ROS 2 parameters.
       * Frame IDs include map_frame, base_frame, input_frame, and output_frame.
       */
      void loadParams();

    private:

      // Buffer for handling transformation data.
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      
      // Listener for transformations.
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      
      // Broadcaster for sending out transformations.
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      
      // Subscription to the /tf topic for transformation messages.
      rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
      
      // Frame IDs for transformations.
      std::string map_frame_;
      std::string base_frame_;
      std::string input_frame_;
      std::string output_frame_;
  };
}
#endif //  ZION_BROADCASTER
