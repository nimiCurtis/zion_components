#ifndef STAIR_MODELING_HPP
#define STAIR_MODELING_HPP

// Standard library includes
#include <sys/resource.h>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <math.h>
#include <iostream>

// Third party libraries includes
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>


// ROS application/library includes
#include <rclcpp/rclcpp.hpp>
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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

// Custom includes
#include "utilities.h"
#include "plane.h"
#include "stair.h"
#include "visibility_control.hpp"
#include <zion_msgs/msg/stair.hpp>
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

        /**
         * @brief Destructor of the StairModeling object.
         */
        virtual ~StairModeling();

    protected:
        /**
         * @brief Callback function for processing received point clouds.
         * @param pcl_msg Point cloud message received from a subscribed topic.
         */
        void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg);

        /**
         * @brief Callback function for processing received detection_msg.
         * @param det_msg Detection2D message received from a subscribed topic.
         */
        void detCallback(const vision_msgs::msg::Detection2D::SharedPtr pcl_msg);


        /**
         * @brief Publishes the convex hulls of detected planes as a MarkerArray.
         * @param cloud_frame The frame ID of the point cloud.
         */
        void publishHullsAsMarkerArray(const std::string& cloud_frame);

        /**
         * @brief Publishes the the Stair msgs.
         * @param cloud_frame The frame ID of the point cloud.
         */
        void publishStair(const std::string& cloud_frame);

        /**
         * @brief Publishes the detected stair properties to a ROS topic.
         * This function takes the properties of the detected stair and constructs a message which 
         * is then published to a ROS topic for further use or visualization.
         * @param cloud_frame The frame ID associated with the detected stair.
         */
        void publishStairPose(const std::string &cloud_frame);

        /**
         * @brief Loads parameters from the parameter server.
         * This function fetches parameters related to segmentation, clustering, floor finding, 
         * average X calculation, topic names, and frame IDs from the parameter server 
         * and sets the corresponding class members.
         */
        void loadParams();
        
        /**
         * @brief Logs the debug message to the ROS info stream.
         * Appends a line separator to the debug message and logs it. 
         * This is useful for debugging purposes.
         */
        void printDebug();

        /**
         * @brief Resets the state of the stair modeling component.
         * Clears detected planes, resets debug message, and initializes
         * indices and flags related to stair detection.
         */
        void reset();

        /**
         * @brief Extracts the main planes from the input point cloud using RANSAC segmentation.
         * This function segments the cloud to identify the main planes. The planes are then
         * processed and stored for further analysis. This function aims to identify planes 
         * representing stairs and their landings in the input point cloud.
         * @param input_cloud Input point cloud from which the planes are to be extracted.
         */
        void getPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

        /**
         * @brief Determines which of the detected planes represent the floor and the level.
         * This function evaluates the planes detected in the point cloud to identify 
         * which of them corresponds to the floor and which corresponds to the level.
         * The function utilizes the average 'X' values of points to determine the proximity 
         * of the plane to the sensor, assuming the floor is closest.
        */
        void findFloor();

        /**
         * @brief Checks if there is a valid stair detected.
         * This function validates if the detected planes correspond to a stair structure 
         * based on the number of detected planes and the length of the level.
         */
        void checkForValidStair();
        
        /**
         * @brief Constructs a Stair object from the detected planes.
         * This function initializes the Stair object based on the properties of the detected planes.
         * It determines the type (ascending/descending), dimensions, and other characteristics of the stair.
         */
        void getStair();
        
        /**
         * @brief Computes the pose of the detected stair.
         * This function determines the position and orientation of the detected stair 
         * based on the transition point and the principal directions of the plane representing the stair level.
         */
        void getStairPose();

        // Not in use. 
        void calcPlaneSlope();
        void setStairTf();

    private:
        std::vector<double> colors_; ///< Vector storing colors for visualization.

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_; ///< Subscription to the point cloud topic.
        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr obj_det_sub_; ///< Subscription to the detection topic.

        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_marker_array_pub_; ///< Publisher to the hull topic.
        rclcpp::Publisher<zion_msgs::msg::StairStamped>::SharedPtr stair_pub_; ///< Publisher to the stair topic.
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; ///< Publisher to the stair pose topic.
        geometry_msgs::msg::Pose::SharedPtr stair_pose_; ///< Pose of the detected stair.

        // tf2_ros
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_; ///< Buffer to store transforms for the tf2 listener.
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; ///< Listener for transform updates.

        Stair Stair_; ///< Stair object representing the detected stair.
        std::vector<Plane> Planes_; ///< Vector of Plane objects representing the detected planes.

        // PointCloud objects and flags
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;   ///< Input point cloud.
        std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_buffer_;  ///< Buffer for storing the processed point cloud.
        std::shared_ptr<vision_msgs::msg::Detection2D> det_buffer_;
        bool stair_detected_; ///< Flag indicating whether a stair has been detected.
        int floor_index_; ///< Index of the detected floor plane.
        int level_index_; ///< Index of the detected level plane.

        // ROS parameters
        double distance_threshold_; ///< Distance threshold for the plane segmentation.
        int max_iterations_; ///< Maximum number of iterations for plane segmentation.
        double angle_threshold_; ///< Angle threshold for plane segmentation.
        double cluster_tolerance_; ///< Tolerance for clustering.
        int min_cluster_size_; ///< Minimum size of the clusters.
        int max_cluster_size_; ///< Maximum size of the clusters.
        int mean_k_; ///< Mean K for segmentation.
        double stddev_mul_thresh_; ///< Standard deviation multiplier threshold for segmentation.

        int k_neighbors_; ///< Number of neighbors for k-means clustering.
        int x_neighbors_; ///< Number of neighbors in the x-direction.
        double y_threshold_; ///< Threshold in the y-direction.

        std::string input_point_cloud_topic_; ///< Topic name for input point cloud.
        std::string filtered_point_cloud_topic_; ///< Topic name for filtered point cloud.

        std::string output_frame_; ///< Name of the output frame for the processed data.

        bool use_det_; ///< Using the stair detection model for predictiong single stair.
        bool planes_empty_; ///< No planes are found. True == there is no planes found from segmentation.
        bool det_found_; ///< Effective only if use_det is true. Detection found boolean. True == there is a detection. 
        bool debug_; ///< Flag for debugging mode.
        std::string debug_msg_; ///< Message string for debugging.
    };
} // namespace

#endif // STAIR_MODELING_H
