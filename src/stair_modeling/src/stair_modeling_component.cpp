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
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock(),tf2::durationFromSec(10));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
        this->declare_parameter("frame_ids.map_frame", "map");
        this->get_parameter("frame_ids.map_frame", map_frame_);
        RCLCPP_INFO(get_logger(),"* map_frame: '%s'", map_frame_.c_str());

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

        // Stair filter params
        this->declare_parameter("stair_filter.filter_min_limit", 3);
        this->get_parameter("stair_filter.filter_min_limit", filter_min_limit_);
        RCLCPP_INFO(get_logger(),"* filter_min_limit: %d", filter_min_limit_);

        this->declare_parameter("stair_filter.filter_max_limit", 10);
        this->get_parameter("stair_filter.filter_max_limit", filter_max_limit_);
        RCLCPP_INFO(get_logger(),"* filter_max_limit: %d", filter_max_limit_);

        this->declare_parameter("stair_filter.pos_err_thresh", 0.05);
        this->get_parameter("stair_filter.pos_err_thresh", pos_err_thresh_);
        RCLCPP_INFO(get_logger(),"* pos_err_thresh: %f", pos_err_thresh_);

        this->declare_parameter("stair_filter.w", 0.5);
        this->get_parameter("stair_filter.w", w_);
        RCLCPP_INFO(get_logger(),"* w factor: %f", w_);

        RCLCPP_INFO(get_logger(),"***************************************");
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
                            float diff = std::abs(Planes_[0].plane_coefficients_->values[3] - plane_coefficients->values[3]);
                            if (k_height_min <= diff && diff <= k_height_max){
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
            size_t floor_index  = -1;
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
                            floor_index = i;
                        }
                    }
            }
            // In case of single plane -> It is the floor!
            else{
                floor_index = 0;
            }

        // set planes type 
        Planes_[floor_index].type_ = 0; // set as floor

    }

    bool StairModeling::checkForValidCandidate(Stair& stair)
    {   
        // TODO // 
        // add connectivity distance 

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
        double x1 = stair1.stair_pose_in_map_.position.x, x2 = stair2.stair_pose_in_map_.position.x;
        double y1 = stair1.stair_pose_in_map_.position.y, y2 = stair2.stair_pose_in_map_.position.y;
        double z1 = stair1.stair_pose_in_map_.position.z, z2 = stair2.stair_pose_in_map_.position.z;
        
        // Calculate the error as the Euclidean distance
        double error = std::sqrt(std::pow(x1 - x2, 2) 
            + std::pow(y1 - y2, 2) 
            + std::pow(z1 - z2, 2));
        // TODO // transform poses to map frame type_
        return error;
    }


    Stair StairModeling::updateStair(Stair& stair1, Stair& stair2)
    {
        // TODO // avg properties
                // avg the map pose only
        stair1.stair_pose_in_map_.position.x = ((1-w_)*stair1.stair_pose_in_map_.position.x + w_*stair2.stair_pose_in_map_.position.x);
        stair1.stair_pose_in_map_.position.y = ((1-w_)*stair1.stair_pose_in_map_.position.y + w_*stair2.stair_pose_in_map_.position.y);
        stair1.stair_pose_in_map_.position.z = ((1-w_)*stair1.stair_pose_in_map_.position.z + w_*stair2.stair_pose_in_map_.position.z);

        stair1.stair_pose_in_map_.orientation = stair2.stair_pose_in_map_.orientation; // for now
        
        stair1.step_distance_ = ((1-w_)*stair1.step_distance_ + w_*stair2.step_distance_);
        stair1.step_height_ = ((1-w_)*stair1.step_height_ + w_*stair2.step_height_);
        stair1.step_width_ = ((1-w_)*stair1.step_width_ + w_*stair2.step_width_);
        stair1.step_length_ = ((1-w_)*stair1.step_length_ + w_*stair2.step_length_);
        stair1.step_angle_ = ((1-w_)*stair1.step_angle_ + w_*stair2.step_angle_);
        stair1.Planes_ = stair2.Planes_;

        return stair1;
    }

    void StairModeling::decrementCounter(std::vector<int>& counter_buffer){
        RCLCPP_INFO_STREAM(this->get_logger(),"Decrementing Counter | buffer size: " << counter_buffer.size());
        // decrement every counts
        for (size_t j=0; j<counter_buffer.size(); j++){
            
            stairs_counts_arr_[j]--;
            RCLCPP_INFO_STREAM(this->get_logger(),"Decrement count of stair index: " << j 
                                        << " to: " << stairs_counts_arr_[j]);
        } // for
    }

    void StairModeling::removeBelowThresh(std::vector<Stair>& stairs_buffer,
                                            std::vector<int>& counter_buffer)
    {
        for (size_t i=0; i<counter_buffer.size(); i++){
            // check if coounts is below counts thresh -> then remove the stair from buffer             
            if (counter_buffer[i]<=-filter_min_limit_){
                        RCLCPP_INFO_STREAM(this->get_logger(),"removing stair number: " << i);
                        auto iter_stairs = stairs_buffer.begin() + i;
                        auto iter_counts = counter_buffer.begin() + i;
                        // erase elements
                        stairs_buffer.erase(iter_stairs);
                        counter_buffer.erase(iter_counts);
            }
        }
        RCLCPP_INFO_STREAM(this->get_logger(),"buffer cleaning | (post) buffer size: " << counter_buffer.size());
    }

    void StairModeling::pushToBuffers(std::vector<Stair>& stairs_buffer,
                                            std::vector<int>& counter_buffer,
                                            Stair& stair)
    {
        stairs_buffer.push_back(stair);
        counter_buffer.push_back(1);
        RCLCPP_INFO_STREAM(this->get_logger(),"push to buffer | (post) buffer size: " << counter_buffer.size());
    }

    void StairModeling::updateBuffers(std::vector<Stair>& stairs_buffer,
                                            std::vector<int>& counter_buffer,
                                            Stair& stair)
    {
        double err;
        bool is_match = false;
        size_t i;

        RCLCPP_INFO_STREAM(this->get_logger(),"update buffer | (pre) buffer size: " << counter_buffer.size());
        for (i=0;i<counter_buffer.size(); i++){
            err = calculatePositionError(stairs_buffer[i], stair);
            RCLCPP_INFO_STREAM(this->get_logger(),"position error: " << err
                                << " | stair index: " << i);
            if(err< pos_err_thresh_ && stairs_buffer[i].type_ == stair.type_ && !is_match){
                    counter_buffer[i]++;
                    updateStair(stairs_buffer[i],stair); 
                    is_match = true;
                    RCLCPP_INFO_STREAM(this->get_logger(),"stair matched to stair number: " << i 
                                        << " and has total counts of: " << counter_buffer[i]);
            }
            else{
                counter_buffer[i]--;
                RCLCPP_INFO_STREAM(this->get_logger(),"error is too big or type is discorrect!! so decrement count of index: " << i 
                                    << " to: " << counter_buffer[i]);
            }
        }
        if (i==counter_buffer.size()){
            pushToBuffers(stairs_buffer, counter_buffer, stair);
        }
    }

    bool StairModeling::isDetectedStair(std::vector<Stair>& stairs_buffer,
                                            std::vector<int>& counter_buffer,
                                            Stair& detect_stair)
    {
        
        int max_count = -filter_min_limit_;
        size_t max_count_i;
        size_t i;

        RCLCPP_INFO_STREAM(this->get_logger(),"check for detected stair | (pre) buffer size:" << counter_buffer.size());
        
        for (i=0;i<counter_buffer.size(); i++){
            if (counter_buffer[i] > max_count){
                counter_buffer[i] = std::min(counter_buffer[i], filter_max_limit_);
                max_count =  counter_buffer[i];
                max_count_i = i;                
                }
        }

        if (max_count >= filter_min_limit_){

                detect_stair = stairs_buffer[max_count_i].get();
                
                RCLCPP_INFO_STREAM(this->get_logger(),"stair detected index: " << max_count_i);
                return true;
        }
        // else false stair detection
        else{
                return false;
        }
    }

    void StairModeling::setStair()
    {
        Stair_ = Stair(Planes_);
    }

    // Not in use
    void StairModeling::calcPlaneSlope()
    {
        Planes_[0].calcPlaneSlope();
        debug_msg_ = debug_msg_ + "\nSlope is:  " + std::to_string(Planes_[0].slope_);
        printDebug();
    }


    void StairModeling::publishStair(const Stair& stair,
                            const std::string& cloud_frame,
                            rclcpp::Time& now)
    {   
        zion_msgs::msg::StairStamped stair_stamped_msg; //with or without ::msg ?
        zion_msgs::msg::Stair stair_msg;

        // declare stair_stamped_msg
        stair_stamped_msg.header.frame_id = cloud_frame;
        stair_stamped_msg.header.stamp = now;

        // declare stair_msg
        stair_stamped_msg.stair.id = stair.type_;
        stair_stamped_msg.stair.distance = stair.step_distance_;
        stair_stamped_msg.stair.height = stair.step_height_;
        stair_stamped_msg.stair.angle = stair.step_angle_;
        stair_stamped_msg.stair.length = stair.step_length_;
        stair_stamped_msg.stair.width = stair.step_width_;
        stair_stamped_msg.stair.pose = stair.stair_pose_; /////////////// change it

        stair_pub_->publish(stair_stamped_msg);
    }


    // void StairModeling::publishPlanesHulls(const std::vector<Plane>& planes,
    //                             const std::string& cloud_frame,
    //                             rclcpp::Time& now)
    // {   
    //     geometry_msgs::msg::Point point;
    //     std_msgs::msg::ColorRGBA point_color;
    //     visualization_msgs::msg::MarkerArray ma;
    //     visualization_msgs::msg::MarkerArray temp;

    //     const std::vector<double> floor_color = {0.,0.,255.}; ///< Vector storing colors for visualization.
        
    //     const std::vector<double> level_color = {0.,255.,0.}; ///< Vector storing colors for visualization.

    //     for (size_t i = 0; i < planes.size(); i++){

    //         double r;
    //         double g;
    //         double b;
    //         std::string ns;

    //         if(planes[i].type_==0){
    //                 ns = "Floor" ;
    //                 r = floor_color[0]/255.;
    //                 g = floor_color[1]/255.;
    //                 b = floor_color[2]/255.;
    //         }else{
    //                 ns = "Level";
    //                 r = level_color[0]/255.;
    //                 g = level_color[1]/255.;
    //                 b = level_color[2]/255.;
    //         }

    //         visualization_msgs::msg::Marker marker;
    //         marker.header.frame_id = cloud_frame;
    //         marker.header.stamp = now;
    //         marker.ns = "ns" + i;
    //         marker.id = i;
    //         marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    //         marker.action = visualization_msgs::msg::Marker::ADD;
    //         marker.pose.position.x = 0;
    //         marker.pose.position.y = 0;
    //         marker.pose.position.z = 0;
    //         marker.pose.orientation.x = 0.0;
    //         marker.pose.orientation.y = 0.0;
    //         marker.pose.orientation.z = 0.0;
    //         marker.pose.orientation.w = 1.0;
    //         marker.scale.x = 0.03;
    //         marker.scale.y = 0.03;
    //         marker.scale.z = 0.03;
    //         marker.color.a = 1.0;

    //         marker.points.reserve(planes[i].hull_->points.size());
    //         marker.colors.reserve(planes[i].hull_->points.size());

    //         for (size_t j = 1; j < planes[i].hull_->points.size(); j++){
    //             point.x = planes[i].hull_->points[j-1].x;
    //             point.y = planes[i].hull_->points[j-1].y;
    //             point.z = planes[i].hull_->points[j-1].z;
    //             point_color.r = 1.;
    //             point_color.g = 0.5;
    //             point_color.b = 0.3;
    //             point_color.a = 1.0;
    //             marker.colors.push_back(point_color);
    //             marker.points.push_back(point);
    //         }

    //         // start to end line:
    //         point.x = planes[i].hull_->points[0].x;
    //         point.y = planes[i].hull_->points[0].y;
    //         point.z = planes[i].hull_->points[0].z;
    //         point_color.r = 1.;
    //         point_color.g = 0.5;
    //         point_color.b = 0.3;
    //         point_color.a = 1.0;
    //         marker.colors.push_back(point_color);
    //         marker.points.push_back(point);

    //         point.x = planes[i].hull_->points[ planes[i].hull_->points.size()-1 ].x;
    //         point.y = planes[i].hull_->points[ planes[i].hull_->points.size()-1 ].y;
    //         point.z = planes[i].hull_->points[ planes[i].hull_->points.size()-1 ].z;
    //         point_color.r = 1.;
    //         point_color.g = 0.5;
    //         point_color.b = 0.3;
    //         point_color.a = 1.0;
    //         point_color.a = 1.0;
    //         marker.colors.push_back(point_color);
    //         marker.points.push_back(point);

    //         marker.frame_locked = false;

    //         // // Create an additional marker for the label
    //         // visualization_msgs::msg::Marker text_marker;
    //         // text_marker.header.frame_id = cloud_frame;
    //         // text_marker.header.stamp = now;
    //         // text_marker.ns = ns + "_text_";
    //         // text_marker.id = i + planes.size(); // Unique ID, different from the hull marker
    //         // text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    //         // text_marker.action = visualization_msgs::msg::Marker::ADD;

    //         // // Position the text marker (adjust as needed)
    //         // // For example, place it at the first point of the hull
    //         // text_marker.pose.position.x = planes[i].hull_->points[0].x;
    //         // text_marker.pose.position.y = planes[i].hull_->points[0].y;
    //         // text_marker.pose.position.z = planes[i].hull_->points[0].z + 0.2; // Slightly above the hull

    //         // // Set the orientation of the text marker
    //         // text_marker.pose.orientation.x = 0.0;
    //         // text_marker.pose.orientation.y = 0.0;
    //         // text_marker.pose.orientation.z = 0.0;
    //         // text_marker.pose.orientation.w = 1.0;

    //         // // Set the scale (size) of the text marker
    //         // text_marker.scale.z = 0.1; // Adjust text size as needed

    //         // // Set the color of the text marker
    //         // text_marker.color.r = 1.0;
    //         // text_marker.color.g = 1.0;
    //         // text_marker.color.b = 1.0;
    //         // text_marker.color.a = 1.0;

    //         // // Set the text of the text marker
    //         // text_marker.text = ns;

    //         // ma.markers->values
    //         // Add the text marker to the MarkerArray
    //         // ma.markers[index] = marker;
    //         // ma.markers[index+1] = text_marker;
    //         // ma.markers.push_back(text_marker);
    //         ma.markers.push_back(marker);
    //     }

    //     hull_marker_array_pub_->publish(ma);

    // }

    void StairModeling::publishPlanesHulls(const std::vector<Plane>& planes,
                                const std::string& cloud_frame,
                                rclcpp::Time& now)
    {   
        geometry_msgs::msg::Point point;
        std_msgs::msg::ColorRGBA point_color;
        visualization_msgs::msg::MarkerArray ma;

        if(planes.size()>0){
            for (size_t i = 0; i < planes.size(); i++){

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


                const double r = 1.;
                const double g = 0.5;
                const double b = 0.3;
                marker.points.reserve(planes[i].hull_->points.size());
                marker.colors.reserve(planes[i].hull_->points.size());

                for (size_t j = 1; j < planes[i].hull_->points.size(); j++){
                point.x = planes[i].hull_->points[j-1].x;
                point.y = planes[i].hull_->points[j-1].y;
                point.z = planes[i].hull_->points[j-1].z;
                point_color.r = r;
                point_color.g = g;
                point_color.b = b;
                point_color.a = 1.0;
                marker.colors.push_back(point_color);
                marker.points.push_back(point);

                point.x = planes[i].hull_->points[j].x;
                point.y = planes[i].hull_->points[j].y;
                point.z = planes[i].hull_->points[j].z;
                point_color.r = r;
                point_color.g = g;
                point_color.b = b;
                point_color.a = 1.0;
                marker.colors.push_back(point_color);
                marker.points.push_back(point);
                }

                // start to end line:
                point.x = planes[i].hull_->points[0].x;
                point.y = planes[i].hull_->points[0].y;
                point.z = planes[i].hull_->points[0].z;
                point_color.r = r;
                point_color.g = g;
                point_color.b = b;
                point_color.a = 1.0;
                marker.colors.push_back(point_color);
                marker.points.push_back(point);

                point.x = planes[i].hull_->points[ planes[i].hull_->points.size()-1 ].x;
                point.y = planes[i].hull_->points[ planes[i].hull_->points.size()-1 ].y;
                point.z = planes[i].hull_->points[ planes[i].hull_->points.size()-1 ].z;
                point_color.r = r;
                point_color.g = g;
                point_color.b = b;
                point_color.a = 1.0;
                marker.colors.push_back(point_color);
                marker.points.push_back(point);

                marker.frame_locked = false;
                ma.markers.push_back(marker);
            }

        }
        hull_marker_array_pub_->publish(ma);

    }



    void StairModeling::publishStairPose(const geometry_msgs::msg::Pose& pose,
                                        const std::string &cloud_frame,
                                        rclcpp::Time& now)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = cloud_frame;
        pose_stamped.header.stamp = now;
        pose_stamped.pose = pose;
        pose_pub_->publish(pose_stamped);
    }


    void StairModeling::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    {

        reset();

        Eigen::Affine3d m2bp;
        Eigen::Affine3d bp2m;

        RCLCPP_INFO_ONCE(get_logger(),"pcl Callback is running");
        RCLCPP_INFO_STREAM(this->get_logger()," " );

        // Check if transformation between frames is available
        if (tf_buffer_->canTransform(output_frame_, input_frame_, tf2::TimePointZero) &&
            tf_buffer_->canTransform(output_frame_, map_frame_, tf2::TimePointZero)) // &&
            //tf_buffer_->canTransform(map_frame_, output_frame_, tf2::TimePointZero))
        {
            try {
                
                geometry_msgs::msg::TransformStamped camera2base_projected;
                geometry_msgs::msg::TransformStamped map2base_projected;
                geometry_msgs::msg::TransformStamped base_projected2map;

                // Lookup for the transformation
                camera2base_projected = tf_buffer_->lookupTransform(
                    output_frame_, input_frame_,
                    tf2::TimePointZero);//,
                    //50ms);



                base_projected2map = tf_buffer_->lookupTransform(
                    map_frame_,output_frame_,
                    tf2::TimePointZero);//,
                // TODO //// Lookup transform target_frame = map, source_frame = output_frame


                // Convert ROS transform to Eigen transform
                c2bp = tf2::transformToEigen(camera2base_projected);
                bp2m = tf2::transformToEigen(base_projected2map);
                m2bp = bp2m.inverse();

                // Initialize point clouds
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);

                // Convert ROS point cloud message to PCL point cloud
                pcl::fromROSMsg(*pcl_msg, *input_cloud);

                // Process the input cloud
                Utilities::voxelizingDownsample(input_cloud, input_cloud,
                                                leaf_size_xy_,leaf_size_z_);


                Utilities::transformCloud(c2bp,input_cloud,input_cloud);


                Utilities::cropping(input_cloud, cloud_,
                                min_x_,max_x_,
                                min_y_,max_y_,
                                min_z_,max_z_);


                // Convert processed PCL point cloud to ROS message
                // Publish pointcloud msg frequently
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
                        setStair();
                        Stair_.TransformPoseToMap(bp2m);

                        // chack if its really proper candidate
                        is_valid_candidate = checkForValidCandidate(Stair_);
                        RCLCPP_INFO_STREAM(get_logger(),"checkForValidCandidate! is_valid_candidate = " 
                                                        << is_valid_candidate);
                        if(!is_valid_candidate){
                            if (stairs_arr_.size()>0){
                                decrementCounter(stairs_counts_arr_);
                                removeBelowThresh(stairs_arr_,stairs_counts_arr_); // here there is a possible of 0 elements after cleaning 
                            }
                        }
                        // is valid candidate
                        else{
                            if(stairs_arr_.size()==0){
                                pushToBuffers(stairs_arr_,stairs_counts_arr_, Stair_);
                            }
                            // is valid and buffers got more then 1 element
                            else{
                                updateBuffers(stairs_arr_,stairs_counts_arr_, Stair_);
                                removeBelowThresh(stairs_arr_,stairs_counts_arr_); // here there is a possible of 0 elements after cleaning 
                            }
                        }

                    // if there is single plane -> keep decrementing
                    }else{
                        if (stairs_arr_.size()>0){
                                decrementCounter(stairs_counts_arr_);
                                removeBelowThresh(stairs_arr_,stairs_counts_arr_); // here there is a possible of 0 elements after cleaning 
                        }
                    }

                    if (stairs_arr_.size()>0){
                            stair_detected_ = isDetectedStair(stairs_arr_,
                                                            stairs_counts_arr_,
                                                            detected_stair_filtered_); //

                    }else{
                        stair_detected_ = false;
                    }

                    RCLCPP_INFO_STREAM(this->get_logger(),"is stair detected: " << stair_detected_);

                    if(stair_detected_){
                    // if stair detected set the stair instance and compute geometric parameters
                        // setStairTf();
                        // Publish the processed point cloud and custom msgs

                        detected_stair_filtered_.TransformPoseToBase(m2bp);

                        RCLCPP_INFO_STREAM(this->get_logger(),"publish!!!!!! ");
                        // getStairPose();

                        rclcpp::Time now = this->get_clock()->now();
                        // publish stair hulls as marker array
                        publishPlanesHulls(detected_stair_filtered_.Planes_,
                                            output_frame_,
                                            now); 


                        publishStair(detected_stair_filtered_,
                                            output_frame_,
                                            now); 
                        
                        // publish detected_stair pose
                        publishStairPose(detected_stair_filtered_.stair_pose_,
                                            output_frame_,
                                            now); 
                    }

                    else if (Planes_.size()==1) //just plane detected
                    {   
                        rclcpp::Time now = this->get_clock()->now();
                        publishPlanesHulls(Planes_,
                                            output_frame_,
                                            now); 
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

} // namespace

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zion::StairModeling)
