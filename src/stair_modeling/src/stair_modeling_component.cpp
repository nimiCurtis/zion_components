// Copyright 2023 Nimrod Curtis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Custom includes
#include "stair_modeling_component.hpp"

namespace zion
{
    StairModeling::StairModeling(const rclcpp::NodeOptions &options)
            : Node("stair_modeling",options)
    {   
        RCLCPP_INFO(get_logger(), "********************************");
        RCLCPP_INFO(get_logger(), "      Zion Component ");
        RCLCPP_INFO(get_logger(), "********************************");
        RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
        RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
        RCLCPP_INFO(get_logger(), "********************************");

        //load params
        loadParams();

        // init pcl and det buffer
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        stair_pose_.reset(new geometry_msgs::msg::Pose);

        // Subscribers and Publishers

        // Callback Groups
        rclcpp::SubscriptionOptions options1;
        rclcpp::CallbackGroup::SharedPtr cbg1 = 
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        options1.callback_group = cbg1;

        rclcpp::SubscriptionOptions options2;
        rclcpp::CallbackGroup::SharedPtr cbg2 = 
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);        
        options2.callback_group = cbg2;

        // Pcl sub
        rclcpp::QoS qos_profile_pcl(10);
        qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        // qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_point_cloud_topic_,qos_profile_pcl,
            std::bind(&StairModeling::pclCallback, this, std::placeholders::_1),
            options1);

        rclcpp::QoS qos_profile_hull(10);
        qos_profile_hull.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        hull_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/planes_hull",qos_profile_hull);

        rclcpp::QoS qos_profile_pose(10);
        qos_profile_pose.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose",qos_profile_pose);

        rclcpp::QoS qos_profile_stair(10);
        qos_profile_stair.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        stair_pub_ = this->create_publisher<zion_msgs::msg::StairStamped>("~/stair",qos_profile_stair);

        rclcpp::QoS qos_profile_pcl_pub(10);
        qos_profile_pcl_pub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        pcl_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud_filtered",qos_profile_pcl_pub);
        pcl_buffer_ = std::make_shared<sensor_msgs::msg::PointCloud2>();


        // init tf instances
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock(),tf2::durationFromSec(5.0));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // Initialize to an invalid index
        floor_index_ = -1;  
        level_index_ = -1;

        planes_empty_ = true;

        colors_={
        255., 0.0, 0.0, // red  
        0.0, 255., 0.0, // green
        0.0, 0.0, 255., // blue
        };

        stairs_arr_ = {};
        stairs_counts_arr_ = {};
        stair_detected_ = false;

    }


    StairModeling::~StairModeling() {
        RCLCPP_DEBUG(get_logger(), "Destroying node");
    }


    void StairModeling::loadParams()
    {
        RCLCPP_INFO(get_logger(),"************   Parameters   ************");

        // Debug
        this->declare_parameter("debug", false);
        this->get_parameter("debug", debug_);
        RCLCPP_INFO(get_logger(),"* debug: %s", debug_ ? "true" : "false");

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

        // Segmentation Parameters
        this->declare_parameter("segmentation.distance_threshold", 0.05);
        this->get_parameter("segmentation.distance_threshold", distance_threshold_);
        RCLCPP_INFO(get_logger(),"* segmentation.distance_threshold: %f", distance_threshold_);

        this->declare_parameter("segmentation.max_iterations", 600);
        this->get_parameter("segmentation.max_iterations", max_iterations_);
        RCLCPP_INFO(get_logger(),"* segmentation.max_iterations: %d", max_iterations_);

        this->declare_parameter("segmentation.angle_threshold", 5.);
        this->get_parameter("segmentation.angle_threshold", angle_threshold_);
        RCLCPP_INFO(get_logger(),"* segmentation.angle_threshold: %f", angle_threshold_);

        // Clustering Parameters
        this->declare_parameter("clustering.cluster_tolerance", 0.08);
        this->get_parameter("clustering.cluster_tolerance", cluster_tolerance_);
        RCLCPP_INFO(get_logger(),"* clustering.cluster_tolerance: %f", cluster_tolerance_);

        this->declare_parameter("clustering.min_cluster_size", 50);
        this->get_parameter("clustering.min_cluster_size", min_cluster_size_);
        RCLCPP_INFO(get_logger(),"* clustering.min_cluster_size: %d", min_cluster_size_);

        // Floor Finding Parameters
        this->declare_parameter("floor_finding.k_neighbors", 50);
        this->get_parameter("floor_finding.k_neighbors", k_neighbors_);
        RCLCPP_INFO(get_logger(),"* floor_finding.k_neighbors: %d", k_neighbors_);

        // Average X Calculation Parameters
        this->declare_parameter("avg_x_calculation.x_neighbors", 20);
        this->get_parameter("avg_x_calculation.x_neighbors", x_neighbors_);
        RCLCPP_INFO(get_logger(),"* avg_x_calculation.x_neighbors: %d", x_neighbors_);

        this->declare_parameter("avg_x_calculation.y_threshold", 0.05);
        this->get_parameter("avg_x_calculation.y_threshold", y_threshold_);
        RCLCPP_INFO(get_logger(),"* avg_x_calculation.y_threshold: %f", y_threshold_);

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
    }

    void StairModeling::printDebug()
    {
        debug_msg_ = debug_msg_ + "\n-----";
        RCLCPP_INFO( this->get_logger(), "%s", debug_msg_.c_str());
    }

    void StairModeling::reset()
    {
        Planes_.clear();
        debug_msg_ = "debug";
        floor_index_ = -1;  
        level_index_ = -1;
        planes_empty_ = true;
    }

    void StairModeling::getPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
    {       
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_points(new pcl::PointCloud<pcl::PointXYZRGB>);
            *outlier_points=*input_cloud;
            int i = 0;
            int n_points = static_cast<int>(outlier_points->points.size());

            // set segmenter
            pcl::SACSegmentation<pcl::PointXYZRGB> plane_segmenter;
            plane_segmenter.setOptimizeCoefficients(true);
            plane_segmenter.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
            plane_segmenter.setMethodType(pcl::SAC_RANSAC);
            plane_segmenter.setMaxIterations(max_iterations_);
            plane_segmenter.setAxis(Eigen::Vector3f(1, 0, 0));
            plane_segmenter.setDistanceThreshold(distance_threshold_);
            plane_segmenter.setEpsAngle(Utilities::deg2rad(angle_threshold_));

            while ((outlier_points->points.size() > 0.15 * n_points) && i<2) {
                i++;
                debug_msg_ = debug_msg_ + "\nRansac iteration: " + std::to_string(i);

                // get the segmented plane cloud
                pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
                pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
                
                plane_segmenter.setInputCloud(outlier_points);
                plane_segmenter.segment(*plane_indices, *plane_coefficients);

                // if cloud is too small break the loop 
                if (static_cast<int>(plane_indices->indices.size()) < min_cluster_size_){
                    RCLCPP_INFO(this->get_logger(), 
                        "Could not estimate a planar model for plane because it too small .\n");
                    break;
                }
                else{ 
                    // get the inliers and remove points using euclidean clustering
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);

                    // get the plane points
                    Utilities::getCloudByInliers(outlier_points, plane_points, plane_indices, false, false); 
                    Utilities::euclideanClustering(plane_points,plane_points_clustered);

                    /// keep outlier
                    Utilities::getCloudByInliers(outlier_points, outlier_points, plane_indices, true, false); 
                    debug_msg_ = debug_msg_ + "\nPlane " + std::to_string(i) + " has: " + std::to_string(plane_points->points.size()) + " points";
                    
                    if (!plane_points_clustered->empty()){
                        // get the first plane 
                        if (Planes_.size()==0){
                            Plane plane = Plane(plane_points_clustered,plane_coefficients);
                            plane.calcPlaneSlope();
                            Planes_.push_back(plane);
                        }
                        // get the second plane
                        else{
                            // check for the height validation
                            // if height difference is in valid range , pushback the second plane
                            if (Stair::checkValidHeight(Planes_[0].plane_coefficients_->values[3],plane_coefficients->values[3])){
                                Plane plane = Plane(plane_points_clustered,plane_coefficients);
                                plane.calcPlaneSlope();
                                Planes_.push_back(plane);
                            }
                            // if not, keep in while loop
                            else{
                                i--;
                            } 
                        }
                    }
                    else{
                        // Handle the case when there are no valid indices
                        RCLCPP_WARN(get_logger(), "No valid indices provided for projection.");
                        break;
                    }
            } // while


            if (!Planes_.empty()){
                planes_empty_ = false;
                // planes features for debug
                for (size_t i = 0; i < Planes_.size(); ++i) {
                                debug_msg_ = debug_msg_ + "\n------- Plane: " + std::to_string(i+1) + "-------";

                                debug_msg_ = debug_msg_ +  
                                "\ncoefficients: [" + std::to_string(Planes_[i].plane_coefficients_->values[0]) + " "
                                                    + std::to_string(Planes_[i].plane_coefficients_->values[1]) + " "
                                                    + std::to_string(Planes_[i].plane_coefficients_->values[2]) + " "
                                                    + std::to_string(Planes_[i].plane_coefficients_->values[3]) + "]";

                                debug_msg_ = debug_msg_ + 
                                "\ncentroid: " +"x: " + std::to_string(Planes_[i].centroid_.x) + " | "
                                            +"y: " + std::to_string(Planes_[i].centroid_.y) + " | "
                                            +"z: " + std::to_string(Planes_[i].centroid_.z);
                                
                                debug_msg_ = debug_msg_ +
                                "\nPCA rotation matrix: " +" \n[" + std::to_string(Planes_[i].plane_dir_.col(0)[0]) + " " + std::to_string(Planes_[i].plane_dir_.col(1)[0])+ " " + std::to_string(Planes_[i].plane_dir_.col(2)[0])
                                                        +"\n " + std::to_string(Planes_[i].plane_dir_.col(0)[1]) + " " + std::to_string(Planes_[i].plane_dir_.col(1)[1])+ " " + std::to_string(Planes_[i].plane_dir_.col(2)[1])
                                                        +"\n " + std::to_string(Planes_[i].plane_dir_.col(0)[2]) + " " + std::to_string(Planes_[i].plane_dir_.col(1)[2])+ " " + std::to_string(Planes_[i].plane_dir_.col(2)[2])+"]";

                                debug_msg_ = debug_msg_ + 
                                "\nbounding rect center: " +"x: " + std::to_string(Planes_[i].center_.x) + " | "
                                            +"y: " + std::to_string(Planes_[i].center_.y) + " | "
                                            +"z: " + std::to_string(Planes_[i].center_.z);
                                
                                debug_msg_ = debug_msg_ + 
                                "\narea: " + std::to_string(Planes_[i].width_ * Planes_[i].length_);
                                
                                debug_msg_ = debug_msg_ + "\n-----";
                }
            }
            else{
                planes_empty_ = true;
                debug_msg_ = debug_msg_ + "\nNo planes!" + "\n-----";
            }
        }
    }


    void StairModeling::findFloor() 
    {

            // If there is more then 1 planes.
            if (Planes_.size()>1){

                // If not using stair detection -> find the floor based on the x's values
                    float min_avg_x = std::numeric_limits<float>::max();

                    // Loop through each plane
                    for (size_t i = 0; i < Planes_.size(); ++i) {
                        const auto& plane = Planes_[i];

                        // Sort the points based on their x-values
                        std::sort(plane.cloud_->points.begin(), plane.cloud_->points.end(),
                                [](const pcl::PointXYZRGB &a, const pcl::PointXYZRGB &b) {
                                    return a.x < b.x;
                                });

                        // Calculate the average 'X' value of the k_neighbors_ smallest x-values
                        float avg_x = 0.0;
                        for (int j = 0; j < k_neighbors_; ++j) {
                            avg_x += plane.cloud_->points[j].x;
                        }
                        avg_x /= k_neighbors_;

                        // Update the closest plane index based on the avg_x
                        ///// continueee from here
                        if (avg_x < min_avg_x) {
                            min_avg_x = avg_x;
                            floor_index_ = static_cast<int>(i);
                        }
                    }
                    if (floor_index_==0){ level_index_ = 1;}
                    else{ level_index_ = 0;}
            }
            // In case of single plane -> It is the floor!
            else{
                floor_index_=0;
            }
        
        // set planes type 
        if (floor_index_!=-1){Planes_[floor_index_].type_ = 0;}
        if (level_index_!=-1){Planes_[level_index_].type_ = 1;}
        
        // for debug
        debug_msg_ = debug_msg_ + "\nFloor index: " + std::to_string(floor_index_) + " | Level index: " + std::to_string(level_index_); 
    }

    bool StairModeling::checkForValidCandidate(Stair& stair)
    {   
        // check the level length is more then the min
            if(
            // && Stair_.step_length_>=k_length_min --> change to volume
            stair.step_height_>=k_height_min 
            && stair.step_height_<=k_height_max
            && (stair.step_length_*stair.step_width_)>=k_area_min){
            return true;
        }else{
            return false;
        }
    }

// Function to calculate the Euclidean distance between two poses
    double StairModeling::calculatePositionError(const Stair& stair1, const Stair& stair2)
    {

        double x1 = stair1.transition_point_.x, x2 = stair2.transition_point_.x;
        double y1 = stair1.transition_point_.y, y2 = stair2.transition_point_.y;
        double z1 = stair1.transition_point_.z, z2 = stair2.transition_point_.z;

        // Calculate the error as the Euclidean distance
        double error = std::sqrt(std::pow(x1 - x2, 2) 
            + std::pow(y1 - y2, 2) 
            + std::pow(z1 - z2, 2));
        return error;
    }

    Stair StairModeling::avgStair(Stair& stair1, Stair& stair2)
    {

        stair1.transition_point_.x = (stair1.transition_point_.x + stair2.transition_point_.x)/2;
        stair1.transition_point_.y = (stair1.transition_point_.y + stair2.transition_point_.y)/2;
        stair1.transition_point_.z = (stair1.transition_point_.z + stair2.transition_point_.z)/2;

        stair1.step_distance_ = (stair1.step_distance_ + stair2.step_distance_)/2;
        stair1.step_height_ = (stair1.step_height_ + stair2.step_height_)/2;
        stair1.step_width_ = (stair1.step_width_ + stair2.step_width_)/2;
        stair1.step_length_ = (stair1.step_length_ + stair2.step_length_)/2;
        stair1.step_angle_ = (stair1.step_angle_ + stair2.step_angle_)/2;
        stair1.Planes_ = stair2.Planes_;

        return stair1;
    }

    void StairModeling::stairDetectionFilter(Stair& stair){
        int buffer_size = stairs_arr_.size();
        RCLCPP_INFO_STREAM(this->get_logger(),"buffer size: " << buffer_size);

        int i = 0; // init running pointer
        int min_count_thresh = 3;
        int max_count_thresh = 10; // maximum counts that stair can get
        int max_count_i = -1; // the index maximum count in the buffer 
        int max_count_val = 0;
        double err_thresh = 0.05;
        double err;
        // iterate over the stairs candidates
        while(i<buffer_size){
                RCLCPP_INFO_STREAM(get_logger(),"index i: " << i);

                                // check if coounts is below counts thresh -> then remove the stair from buffer             
                if (stairs_counts_arr_[i]<=-min_count_thresh){
                    RCLCPP_INFO_STREAM(this->get_logger(),"removig stair number: " << i);

                    auto iter_stairs = stairs_arr_.begin() + i;
                    auto iter_counts = stairs_counts_arr_.begin() + i;

                    // erase elements
                    stairs_arr_.erase(iter_stairs);
                    stairs_counts_arr_.erase(iter_counts);
                }else if (stairs_counts_arr_[i] > max_count_val)
                {
                max_count_val = std::min(stairs_counts_arr_[i], max_count_thresh); 
                max_count_i = i;                
                }

                // check the error between poses
                err = calculatePositionError(stairs_arr_[i], stair);
                RCLCPP_INFO_STREAM(this->get_logger(),"position error: " << err);
                
                if(err< err_thresh && stairs_arr_[i].type_ == stair.type_){
                    stairs_counts_arr_[i]++;
                    stairs_arr_[i] = avgStair(stairs_arr_[i],stair);
                    RCLCPP_INFO_STREAM(this->get_logger(),"stair matched to stair number: " << i 
                                        << " and has total counts of: " << stairs_counts_arr_[i]);
                    break;
                }
                else{
                    stairs_counts_arr_[i]--;
                    RCLCPP_INFO_STREAM(this->get_logger(),"error is too big so decrement count of index:" << i 
                                        << " to " << stairs_counts_arr_[i]);
                }

                i++; // increment running pointer
        } // while

        if (max_count_val>= min_count_thresh){
            RCLCPP_INFO_STREAM(this->get_logger(),"stair detected!!!!!! stair index: " << max_count_i);
            detected_stair_filtered_ = stairs_arr_[max_count_i];
            stair_detected_ = true;
        }else{
            RCLCPP_INFO_STREAM(this->get_logger(),"stair still doesn't detected!! index of stair with max count is: " << max_count_i);
            stair_detected_ = false;
        }

        // if stair is not matching to other stair in the buffer 
        // *or* buffer lenght is zero -> push the stair to the buffer
        if (i == buffer_size){
            RCLCPP_INFO_STREAM(this->get_logger(),"push back stair to buffer! ");
            stairs_arr_.push_back(stair);
            stairs_counts_arr_.push_back(1);
        }


    }

    void StairModeling::getStair()
    {
        // init stair
        Stair_ = Stair(Planes_);  
        float floor_h = Stair_.Planes_[floor_index_].plane_coefficients_->values[3];
        float level_h = Stair_.Planes_[level_index_].plane_coefficients_->values[3];
        
        // set the stair type based on the planes height
        // if floor height is high than the level height (farther from the camera) -> stair ascending
        if (floor_h>level_h){
            Stair_.type_ = 0; // 0 = stair is upward
            Stair_.step_length_ = Stair_.Planes_[level_index_].length_;
            Stair_.step_width_ = Stair_.Planes_[level_index_].width_;
            Stair_.transition_point_.y = Stair_.Planes_[level_index_].center_.y;
            Stair_.transition_point_.z = Stair_.Planes_[level_index_].center_.z;
            // If upwards, calculate the distance by finding the average x-coordinate
            // of the points below yThreshold in the level plane's cloud
            float x_distance = Utilities::findAvgXForPointsBelowYThreshold(Planes_[level_index_].cloud_, y_threshold_, x_neighbors_, true);
            Stair_.transition_point_.x = x_distance;
            debug_msg_ = debug_msg_ + "\nStair type: Up";
            Stair_.step_distance_ = x_distance;

        }else{ // -> stair descending
            Stair_.type_ = 1; // 1 = downwards
            Stair_.step_length_ = Stair_.Planes_[floor_index_].length_;
            Stair_.step_width_ = Stair_.Planes_[floor_index_].width_;
            Stair_.transition_point_.y = Stair_.Planes_[floor_index_].center_.y;
            Stair_.transition_point_.z = Stair_.Planes_[floor_index_].center_.z;

            // compute distance
            // If downwards, calculate the distance by finding the average x-coordinate
            // of the points below yThreshold in the second plane's cloud
            float x_distance = Utilities::findAvgXForPointsBelowYThreshold(Planes_[floor_index_].cloud_, y_threshold_, x_neighbors_, false);
            Stair_.transition_point_.x = x_distance;
            debug_msg_ = debug_msg_ + "\nStair type: Down";
            Stair_.step_distance_ = x_distance;
        }
        Stair_.transition_point_.x = Stair_.step_distance_;
        // compute height
        Stair_.step_height_ = fabs(floor_h-level_h);
        // compute angle
        Stair_.step_angle_ = Utilities::rad2deg(std::atan2(Stair_.transition_point_.y,Stair_.transition_point_.x));

        // debug
        debug_msg_ = debug_msg_ + "\nHeight is:  " + std::to_string(Stair_.step_height_ );
        debug_msg_ = debug_msg_ + "\nDistance is:  " + std::to_string(Stair_.step_distance_);
    }


    void StairModeling::getStairPose()
    {
        geometry_msgs::msg::Pose pose;

        stair_pose_->position.x = detected_stair_filtered_.transition_point_.x;
        stair_pose_->position.y = detected_stair_filtered_.transition_point_.y;
        stair_pose_->position.z = detected_stair_filtered_.transition_point_.z;

        tf2::Matrix3x3 tf_rotation;
        tf2::Quaternion tf_quaternion;
        geometry_msgs::msg::Quaternion ros_quaternion;
        tf_rotation.setValue(static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(0, 0)), static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(0, 1)), static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(0, 2)),
                        static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(1, 0)), static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(1, 1)), static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(1, 2)),
                        static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(2, 0)), static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(2, 1)), static_cast<double>(detected_stair_filtered_.Planes_[level_index_].plane_dir_(2, 2)));

        double roll ; double pitch ; double yaw;
        tf_rotation.getEulerYPR(yaw,pitch,roll);
        tf2::Quaternion q_rotation;
        if(pitch>=0){q_rotation.setRPY(0.,0.,pitch - M_PI/2);}
        else{q_rotation.setRPY(0.,0.,pitch + M_PI/2);}
        q_rotation.normalize();

        stair_pose_->orientation = tf2::toMsg(q_rotation);
    //     RCLCPP_INFO(this->get_logger(),
    //                 "Received Pose message:\n"
    //                 "Position (x, y, z): %.2f, %.2f, %.2f\n"
    //                 "Orientation (x, y, z, w): %.2f, %.2f, %.2f, %.2f",
    //                 stair_pose_->position.x, stair_pose_->position.y, stair_pose_->position.z,
    //                 stair_pose_->orientation.x, stair_pose_->orientation.y, stair_pose_->orientation.z, stair_pose_->orientation.w);
    }

    // Not in use for now
    void StairModeling::setStairTf()
    {
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped transformStamped;

        // set transform msg
        tf2::Transform tf2_transform;

        tf2_transform.setIdentity();    
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = output_frame_;
        transformStamped.child_frame_id = "stair";

        //translation
        transformStamped.transform.translation.x = Stair_.transition_point_.x;
        transformStamped.transform.translation.y = Stair_.transition_point_.y;
        transformStamped.transform.translation.z = Stair_.transition_point_.z;

        //rotation
        tf2::Quaternion q_rotation;
        q_rotation.setRPY(0.,0.,0.);
        q_rotation.normalize();

        transformStamped.transform.rotation = tf2::toMsg(q_rotation);
        // sending transform
        tf_broadcaster->sendTransform(transformStamped);
    }

    // Not in use
    void StairModeling::calcPlaneSlope()
    {
        Planes_[0].calcPlaneSlope();
        debug_msg_ = debug_msg_ + "\nSlope is:  " + std::to_string(Planes_[0].slope_);
        printDebug();
    }

    void StairModeling::publishStair(const std::string& cloud_frame)
    {   
        
        zion_msgs::msg::StairStamped stair_stamped_msg; //with or without ::msg ?
        zion_msgs::msg::Stair stair_msg;
        
        // declare stair_stamped_msg
        stair_stamped_msg.header.frame_id = cloud_frame;
        stair_stamped_msg.header.stamp = this->get_clock()->now();

        // declare stair_msg

        stair_stamped_msg.stair.id = detected_stair_filtered_.type_;
        stair_stamped_msg.stair.distance = detected_stair_filtered_.step_distance_;
        stair_stamped_msg.stair.height = detected_stair_filtered_.step_height_;
        stair_stamped_msg.stair.angle = detected_stair_filtered_.step_angle_;
        stair_stamped_msg.stair.length = detected_stair_filtered_.step_length_;
        stair_stamped_msg.stair.width = detected_stair_filtered_.step_width_;
        stair_stamped_msg.stair.pose = *stair_pose_; /////////////// change it

        stair_pub_->publish(stair_stamped_msg);
    }


    void StairModeling::publishHullsAsMarkerArray(const std::string& cloud_frame)
    {

    geometry_msgs::msg::Point point;
    std_msgs::msg::ColorRGBA point_color;
    visualization_msgs::msg::MarkerArray ma;

    if(Planes_.size()>0){
        for (size_t i = 0; i < Planes_.size(); i++){

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = cloud_frame;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "hull_" + std::to_string(i);
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;
            marker.color.a = 1.0;

            const int nColor = i % (colors_.size()/3);
            const double r = colors_[nColor*3]*255.0;
            const double g = colors_[nColor*3+1]*255.0;
            const double b = colors_[nColor*3+2]*255.0;
            marker.points.reserve(Planes_[i].hull_->points.size());
            marker.colors.reserve(Planes_[i].hull_->points.size());

            for (size_t j = 1; j < Planes_[i].hull_->points.size(); j++){
            point.x = Planes_[i].hull_->points[j-1].x;
            point.y = Planes_[i].hull_->points[j-1].y;
            point.z = Planes_[i].hull_->points[j-1].z;
            point_color.r = r;
            point_color.g = g;
            point_color.b = b;
            point_color.a = 1.0;
            marker.colors.push_back(point_color);
            marker.points.push_back(point);

            point.x = Planes_[i].hull_->points[j].x;
            point.y = Planes_[i].hull_->points[j].y;
            point.z = Planes_[i].hull_->points[j].z;
            point_color.r = r;
            point_color.g = g;
            point_color.b = b;
            point_color.a = 1.0;
            marker.colors.push_back(point_color);
            marker.points.push_back(point);
            }

            // start to end line:
            point.x = Planes_[i].hull_->points[0].x;
            point.y = Planes_[i].hull_->points[0].y;
            point.z = Planes_[i].hull_->points[0].z;
            point_color.r = r;
            point_color.g = g;
            point_color.b = b;
            point_color.a = 1.0;
            marker.colors.push_back(point_color);
            marker.points.push_back(point);

            point.x = Planes_[i].hull_->points[ Planes_[i].hull_->points.size()-1 ].x;
            point.y = Planes_[i].hull_->points[ Planes_[i].hull_->points.size()-1 ].y;
            point.z = Planes_[i].hull_->points[ Planes_[i].hull_->points.size()-1 ].z;
            point_color.r = r;
            point_color.g = g;
            point_color.b = b;
            point_color.a = 1.0;
            marker.colors.push_back(point_color);
            marker.points.push_back(point);

            marker.frame_locked = false;
            ma.markers.push_back(marker);
        }

        hull_marker_array_pub_->publish(ma);
        }
    }

    void StairModeling::publishStairPose(const std::string &cloud_frame)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = cloud_frame;
        pose_stamped.header.stamp = this->get_clock()->now();;
        pose_stamped.pose = *stair_pose_;
        pose_pub_->publish(pose_stamped);
    }


    void StairModeling::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    {

        reset();

        geometry_msgs::msg::TransformStamped base_projected2camera;

        RCLCPP_INFO_ONCE(get_logger(),"pcl Callback is running");

        // Check if transformation between frames is available
        if (tf_buffer_->canTransform(output_frame_, input_frame_, tf2::TimePointZero))
        {
            try {
                // Lookup for the transformation
                base_projected2camera = tf_buffer_->lookupTransform(
                    output_frame_, input_frame_,
                    tf2::TimePointZero,
                    50ms);

                // Convert ROS transform to Eigen transform
                c2cp = tf2::transformToEigen(base_projected2camera);


                // Initialize point clouds
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);

                // Convert ROS point cloud message to PCL point cloud
                pcl::fromROSMsg(*pcl_msg, *input_cloud);

                // Process the input cloud
                Utilities::voxelizingDownsample(input_cloud, input_cloud,
                                                leaf_size_xy_,leaf_size_z_);


                Utilities::transformCloud(c2cp,input_cloud,input_cloud);


                Utilities::cropping(input_cloud, cloud_,
                                min_x_,max_x_,
                                min_y_,max_y_,
                                min_z_,max_z_);


                // Convert processed PCL point cloud to ROS message
                pcl::toROSMsg(*cloud_, *pcl_buffer_);
                pcl_buffer_->header.stamp = this->get_clock()->now();
                pcl_buffer_->header.frame_id = output_frame_;
                pcl_pub_->publish(*pcl_buffer_);



                // RANSAC-based plane segmentation
                // cloud_ = output_cloud;
                getPlanes(cloud_);

                if(!planes_empty_){
                    // find the floor plane
                    RCLCPP_INFO(get_logger(),"planes not empty!");

                    findFloor();
                    RCLCPP_INFO(get_logger(),"find floor!");

                    bool is_valid_candidate = false;
                    if(Planes_.size()>1){
                        // get the raw stair properties
                        getStair();
                        RCLCPP_INFO(get_logger(),"getStair!");

                        // chack if its really proper candidate
                        is_valid_candidate = checkForValidCandidate(Stair_);
                        RCLCPP_INFO_STREAM(get_logger(),"checkForValidCandidate! is_valid_candidate = " 
                                                        << is_valid_candidate);

                        }

                        // if its true candidate apply get the stair pose and filter the stairs detections
                        if(is_valid_candidate){
                            RCLCPP_INFO_STREAM(get_logger(),"getStairPose !");
                            stairDetectionFilter(Stair_);
                        }

                    // if stair detected set the stair instance and compute geometric parameters
                    if(stair_detected_){
                        // setStairTf();
                        // Publish the processed point cloud and custom msgs
                        RCLCPP_INFO_STREAM(this->get_logger(),"publish!!!!!! ");
                        getStairPose();
                        publishHullsAsMarkerArray(output_frame_);
                        publishStairPose(output_frame_);
                        publishStair(output_frame_);
                    }
                }
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                input_frame_.c_str(), output_frame_.c_str(), ex.what());
                return;
            }
            // debug msg
            if(debug_){
                printDebug();
            }
        }
    }


    // void StairModeling::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    // {
    //     // RCLCPP_INFO(get_logger(),"Pcl msg recieved!");

    //     reset();


    //             // Convert ROS point cloud message to PCL point cloud
    //             pcl::fromROSMsg(*pcl_msg, *cloud_);

    //             // RANSAC-based plane segmentation
    //             // cloud_ = output_cloud;
    //             getPlanes(cloud_);

    //             if(!planes_empty_){
    //                 // find the floor plane
                    
    //                 findFloor();
    //                 if(Planes_.size()>1){getStair();}
    //                 checkForValidStair();

    //                 // if stair detected set the stair instance and compute geometric parameters
    //                 if(stair_detected_){
    //                     // getStair();
    //                     // setStairTf();
    //                     getStairPose();
    //                     // Publish the processed point cloud and custom msgs
    //                     publishHullsAsMarkerArray(output_frame_);
    //                     publishStairPose(output_frame_);
    //                     publishStair(output_frame_);
    //                 }
    //             }

    //         // debug msg
    //         if(debug_){
    //             printDebug();
    //         }
        
    // }

    //     void StairModeling::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    // {
    //     // RCLCPP_INFO(get_logger(),"Pcl msg recieved!");

    //     reset();

    //     geometry_msgs::msg::TransformStamped base_projected2camera;

    //     // Check if transformation between frames is available
    //     if (tf_buffer_->canTransform(output_frame_, input_frame_, tf2::TimePointZero))
    //     {
    //         try {
    //             // Lookup for the transformation
    //             base_projected2camera = tf_buffer_->lookupTransform(
    //                 output_frame_, input_frame_,
    //                 tf2::TimePointZero,
    //                 50ms);

    //             // Convert ROS transform to Eigen transform
    //             c2cp = tf2::transformToEigen(base_projected2camera);

    //             // Initialize point clouds
    //             pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
    //             // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);

    //             // Convert ROS point cloud message to PCL point cloud
    //             pcl::fromROSMsg(*pcl_msg, *input_cloud);

    //             // Process the input cloud
    //             Utilities::voxelizingDownsample(input_cloud, input_cloud,
    //                                             leaf_size_xy_,leaf_size_z_);

    //             Utilities::transformCloud(c2cp,input_cloud,input_cloud);

    //             Utilities::cropping(input_cloud, cloud_,
    //                             min_x_,max_x_,
    //                             min_y_,max_y_,
    //                             min_z_,max_z_);

    //             // Convert processed PCL point cloud to ROS message
    //             pcl::toROSMsg(*cloud_, *pcl_buffer_);
    //             pcl_buffer_->header.stamp = this->get_clock()->now();
    //             pcl_buffer_->header.frame_id = output_frame_;
    //             pcl_pub_->publish(*pcl_buffer_);

                
    //         }
    //         catch (const tf2::TransformException & ex) {
    //             RCLCPP_INFO(
    //             this->get_logger(), "Could not transform %s to %s: %s",
    //             input_frame_.c_str(), output_frame_.c_str(), ex.what());
    //             return;
    //         }
    //         // debug msg
    //         if(debug_){
    //             printDebug();
    //         }
    //     }
    // }

} // namespace

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zion::StairModeling)
