// Custom includes
#include "cloud_processor_component.hpp"
namespace zion
{
  CloudProcessor::CloudProcessor(const rclcpp::NodeOptions &options)
      : Node("cloud_processor",options)
  { 
      // Initialization log messages
      RCLCPP_INFO(get_logger(), "********************************");
      RCLCPP_INFO(get_logger(), "      Zion Component ");
      RCLCPP_INFO(get_logger(), "********************************");
      RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
      RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
      RCLCPP_INFO(get_logger(), "********************************");

      loadParams();

      // Initialize the tf instances for transforming data between coordinate frames
      tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Setting up subscribers and publishers
      rclcpp::QoS qos_profile_pcl(1);
      qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_point_cloud_topic_, qos_profile_pcl,
          std::bind(&CloudProcessor::pclCallback, this, std::placeholders::_1));

      pcl_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud_filtered",10);
      pcl_buffer_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    }

  CloudProcessor::~CloudProcessor() {
      RCLCPP_DEBUG(get_logger(), "Destroying node");
  }

  void CloudProcessor::loadParams()
  {
      RCLCPP_INFO(get_logger(),"************   Parameters   ************");

      // Voxel Filter Parameters
      this->declare_parameter("voxel_filter.leaf_size_xy", 0.025);
      this->get_parameter("voxel_filter.leaf_size_xy", leaf_size_xy_);
      RCLCPP_INFO(get_logger(),"* leaf_size_xy: %f", leaf_size_xy_);

      this->declare_parameter("voxel_filter.leaf_size_z", 0.08);
      this->get_parameter("voxel_filter.leaf_size_z", leaf_size_z_);
      RCLCPP_INFO(get_logger(),"* leaf_size_z: %f", leaf_size_z_);

      // Crop Box Parameters
      this->declare_parameter("crop_box.min_x", 0.0);
      this->get_parameter("crop_box.min_x", min_x_);
      RCLCPP_INFO(get_logger(),"* min_x: %f", min_x_);

      this->declare_parameter("crop_box.max_x", 2.0);
      this->get_parameter("crop_box.max_x", max_x_);
      RCLCPP_INFO(get_logger(),"* max_x: %f", max_x_);

      this->declare_parameter("crop_box.min_y", -0.8);
      this->get_parameter("crop_box.min_y", min_y_);
      RCLCPP_INFO(get_logger(),"* min_y: %f", min_y_);

      this->declare_parameter("crop_box.max_y", 0.8);
      this->get_parameter("crop_box.max_y", max_y_);
      RCLCPP_INFO(get_logger(),"* max_y: %f", max_y_);

      this->declare_parameter("crop_box.min_z", -3.0);
      this->get_parameter("crop_box.min_z", min_z_);
      RCLCPP_INFO(get_logger(),"* min_z: %f", min_z_);

      this->declare_parameter("crop_box.max_z", 1.0);
      this->get_parameter("crop_box.max_z", max_z_);
      RCLCPP_INFO(get_logger(),"* max_z: %f", max_z_);

      // Topic Names
      this->declare_parameter("topic_names.input_point_cloud_topic", "/zedm/zed_node/point_cloud/cloud_registered");
      this->get_parameter("topic_names.input_point_cloud_topic", input_point_cloud_topic_);
      RCLCPP_INFO(get_logger(),"* input_point_cloud_topic: %s", input_point_cloud_topic_.c_str());

      // Frame IDs
      this->declare_parameter("frame_ids.input_cloud_frame", "zedm_left_camera_frame");
      this->get_parameter("frame_ids.input_cloud_frame", input_frame_);
      RCLCPP_INFO(get_logger(),"* input_cloud_frame: '%s'", input_frame_.c_str());

      this->declare_parameter("frame_ids.output_cloud_frame", "zedm_base_link_projected");
      this->get_parameter("frame_ids.output_cloud_frame", output_frame_);
      RCLCPP_INFO(get_logger(),"* output_cloud_frame: '%s'", output_frame_.c_str());
  }

  void CloudProcessor::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
  {
      geometry_msgs::msg::TransformStamped base_projected2camera;

      // Check if transformation between frames is available
      if (tf_buffer_->canTransform(output_frame_, input_frame_, tf2::TimePointZero))
      {
          try {
            // Lookup for the transformation
            base_projected2camera = tf_buffer_->lookupTransform(
                  output_frame_, input_frame_,
                  tf2::TimePointZero);

            // Convert ROS transform to Eigen transform
            c2cp = tf2::transformToEigen(base_projected2camera);

            // Initialize point clouds
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);

            // Convert ROS point cloud message to PCL point cloud
            pcl::fromROSMsg(*pcl_msg, *input_cloud);

            // Process the input cloud
            Utilities::voxelizingDownsample(input_cloud, input_cloud,0.025,0.08);
            Utilities::transformCloud(c2cp,input_cloud,input_cloud);
            Utilities::cropping(input_cloud, output_cloud,
                              min_x_,max_x_,
                              min_y_,max_y_,
                              min_z_,max_z_);

            // Convert processed PCL point cloud to ROS message
            pcl::toROSMsg(*output_cloud, *pcl_buffer_);
            pcl_buffer_->header.stamp = this->get_clock()->now();
            pcl_buffer_->header.frame_id = output_frame_;
            pcl_pub_->publish(*pcl_buffer_);
          }
          catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              input_frame_.c_str(), output_frame_.c_str(), ex.what());
            return;
          }
      }
  }
}// namespace


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zion::CloudProcessor)

