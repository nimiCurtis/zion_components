#ifndef STAIR_MODELING_HPP
#define STAIR_MODELING_HPP

#include "utilities.h"
#include "plane.h"
#include "stair.h"
#include "visibility_control.hpp"

// Include necessary libraries and dependencies
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//When using customn ros2 meseges, use onliu .hpp and small leters with '_' as a separation between words
#include <zion_msgs/msg/stair_stamped.hpp>


namespace zion
{
    /**
     * @brief The StairModeling class is responsible for processing and modeling stairs
     * in a 3D environment using point cloud data.
     */
    class StairModeling : public rclcpp::Node 
    {
    public:
        /**
         * @brief Constructor that initializes the StairModeling object with a given NodeHandle.
         * @param nh NodeHandle used for initializing subscribers, publishers, etc.
         */
        ZION_COMPONENTS_PUBLIC
        explicit StairModeling(const rclcpp::NodeOptions & options);
        virtual ~StairModeling();

    
    protected:
        /**
         * @brief Callback function for processing received point clouds.
         * @param pcl_msg Point cloud message received from a subscribed topic.
         */
        void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg);

        /**
         * @brief Publishes the convex hulls of detected planes as a MarkerArray.
         * @param cloud_frame The frame ID of the point cloud.
         * @param secs The seconds part of the timestamp.
         * @param nsecs The nanoseconds part of the timestamp.
         */
        void publishHullsAsMarkerArray(const std::string& cloud_frame);

        /**
         * @brief Publishes the the env_geo msgs.
         * @param cloud_frame The frame ID of the point cloud.
         * @param secs The seconds part of the timestamp.
         * @param nsecs The nanoseconds part of the timestamp.
         */
        void publishStair(const std::string& cloud_frame);

        void publishStairPose(const std::string &cloud_frame);
        void getPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

        void findFloor();
        void calcPlaneSlope();
        void checkForValidStair();
        void getStair();
        void getStairPose();

        void reset();
        void loadParams();
        void printDebug();
        void setStairTf();

    private:

        

        std::vector<double> colors_; ///< Vector storing colors for visualization.

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;

        //publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_marker_array_pub_; ///< Publisher to the hull topic.
        rclcpp::Publisher<zion_msgs::msg::StairStamped>::SharedPtr stair_pub_; ///< Publisher to the stair topic.
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; ///< Publisher to the pose topic.

        geometry_msgs::msg::Pose::SharedPtr stair_pose_;

        // tf2_ros
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        Eigen::Affine3d c2cp; ///< camera to camera projected transformation
        Eigen::Affine3d cp2c; ///< camera projected to camera transformation

        Stair Stair_; ///< Stair object representing the detected stair.
        std::vector<Plane> Planes_; ///< Vector of Plane objects representing the detected planes.

        // PointCloud objects and flags
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_; ///< Input point cloud.
        // sensor_msgs::msg::PointCloud2::Ptr pcl_buffer_; ///< Buffer for storing the processed point cloud.
        std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_buffer_;
        bool stair_detected_; ///< Flag indicating whether a stair has been detected.
        int floor_index_ ;
        int level_index_;
        
        // ROS parameters

        double distance_threshold_;
        int max_iterations_;
        double angle_threshold_;
        double cluster_tolerance_;
        int min_cluster_size_;
        int max_cluster_size_;
        int mean_k_;
        double stddev_mul_thresh_;

        int k_neighbors_;
        int x_neighbors_;
        double y_threshold_;

        std::string input_point_cloud_topic_;
        std::string filtered_point_cloud_topic_;

        std::string map_frame_;
        std::string base_frame_;
        std::string input_frame_;
        std::string output_frame_;

        bool debug_;
        std::string debug_msg_;
    };
} // namespace

#endif // STAIR_MODELING_H
