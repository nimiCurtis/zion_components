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
        det_buffer_ = std::shared_ptr<vision_msgs::msg::Detection2D>();

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
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(filtered_point_cloud_topic_,qos_profile_pcl,
            std::bind(&StairModeling::pclCallback, this, std::placeholders::_1),
            options1);

        rclcpp::QoS qos_profile_det(10);
        qos_profile_det.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        obj_det_sub_  = this->create_subscription<vision_msgs::msg::Detection2D>("/zion/stair_detection/detection",qos_profile_det,
            std::bind(&StairModeling::detCallback, this, std::placeholders::_1),
            options2);

        rclcpp::QoS qos_profile_hull(10);
        qos_profile_hull.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        hull_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/planes_hull",qos_profile_hull);

        rclcpp::QoS qos_profile_pose(10);
        qos_profile_pose.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose",qos_profile_pose);

        rclcpp::QoS qos_profile_stair(10);
        qos_profile_stair.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        stair_pub_ = this->create_publisher<zion_msgs::msg::StairStamped>("~/stair",qos_profile_stair);

        rclcpp::QoS qos_profile_stair_fused(10);
        qos_profile_stair_fused.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        stair_det_fused_pub_ = this->create_publisher<zion_msgs::msg::StairDetStamped>("~/stair_fused",qos_profile_stair_fused);

        // init tf instances
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // Initialize to an invalid index
        floor_index_ = -1;  
        level_index_ = -1;

        det_found_ = false;
        planes_empty_ = true;

        colors_={
        255., 0.0, 0.0, // red  
        0.0, 255., 0.0, // green
        0.0, 0.0, 255., // blue
        };
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

        // Debug
        this->declare_parameter("use_det", false);
        this->get_parameter("use_det", use_det_);
        RCLCPP_INFO(get_logger(),"* use_det: %s", use_det_ ? "true" : "false");

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

        // Topic Names
        this->declare_parameter("topic_names.filtered_point_cloud_topic", "/stair_modeling_ros/point_cloud/cloud_filtered");
        this->get_parameter("topic_names.filtered_point_cloud_topic", filtered_point_cloud_topic_);
        RCLCPP_INFO(get_logger(),"* filtered_point_cloud_topic: %s", filtered_point_cloud_topic_.c_str());

        // Frame IDs
        this->declare_parameter("frame_ids.output_cloud_frame", "zedm_base_link_projected");
        this->get_parameter("frame_ids.output_cloud_frame", output_frame_);
        RCLCPP_INFO(get_logger(),"* output_cloud_frame: '%s'", output_frame_.c_str());
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
        stair_detected_ = false;
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

            while ((outlier_points->points.size() > 0.2 * n_points) && i<2) {
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
                                "\nwidth: " + std::to_string(Planes_[i].width_) + " | "
                                +"length: " + std::to_string(Planes_[i].length_);
                                
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
                if (!use_det_){
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
                // Else find the floor based on the stair detection result
                else{
                    float min_height = std::numeric_limits<float>::max();
                    int min_index = -1;
                    // Loop through each plane
                    // And find the index of the lowest plane
                    for (size_t i = 0; i < Planes_.size(); ++i) {
                        const auto& plane = Planes_[i];
                        if (plane.plane_coefficients_->values[3]<min_height){
                        min_index=i;
                        min_height = plane.plane_coefficients_->values[3];
                        }
                    }
                    std::string stair_id = det_buffer_->results.begin()->id;
                    // If stair ascending -> the lowest plane is the floor
                    if(stair_id=="SSA"){
                        floor_index_ = min_index;
                        if (floor_index_==0){ level_index_ = 1;}
                        else{level_index_ = 0;}
                    }
                    // If stair descending -> the lowest plane is the level
                    else{
                        level_index_ = min_index;
                        if (level_index_==0){ floor_index_ = 1;}
                        else{floor_index_ = 0;}
                    }
                }
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

    void StairModeling::checkForValidStair()
    {   
        // check there is more then 1 plane
        // and the level length is more then the min
        // if(Planes_.size()>1 && Planes_[level_index_].length_>=k_length_min){
            if(Planes_.size()>1 
            && Stair_.step_length_>=k_length_min 
            && Stair_.step_height_>=k_height_min 
            && Stair_.step_height_<=k_height_max){
            stair_detected_ = true;
        }
    }

    void StairModeling::getStair()
    {
        // init stair
        Stair_ = Stair(Planes_);  
        // Stair_.step_length_ = Stair_.Planes_[level_index_].length_;
        // Stair_.step_width_ = Stair_.Planes_[level_index_].width_;
        // Stair_.transition_point_.y = Stair_.Planes_[level_index_].center_.y;
        // Stair_.transition_point_.z = Stair_.Planes_[level_index_].center_.z;


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
            // Stair_.transition_point_.x = Stair_.Planes_[level_index_].center_.x - (Stair_.step_length_/2);

            debug_msg_ = debug_msg_ + "\nStair type: Up";
            // If upwards, calculate the distance by finding the average x-coordinate
            // of the points below yThreshold in the level plane's cloud
            Stair_.step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_[level_index_].cloud_, y_threshold_, x_neighbors_, true);
        }else{ // -> stair descending
            Stair_.type_ = 1; // 1 = downwards
            Stair_.step_length_ = Stair_.Planes_[floor_index_].length_;
            Stair_.step_width_ = Stair_.Planes_[floor_index_].width_;
            Stair_.transition_point_.y = Stair_.Planes_[floor_index_].center_.y;
            Stair_.transition_point_.z = Stair_.Planes_[floor_index_].center_.z;
            // Stair_.transition_point_.x = Stair_.Planes_[floor_index_].center_.x + (Stair_.step_length_/2);

            debug_msg_ = debug_msg_ + "\nStair type: Down";
            // compute distance
            // If downwards, calculate the distance by finding the average x-coordinate
            // of the points below yThreshold in the second plane's cloud
            Stair_.step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_[floor_index_].cloud_, y_threshold_, x_neighbors_, false);
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

        stair_pose_->position.x = Stair_.transition_point_.x;
        stair_pose_->position.y = Stair_.transition_point_.y;
        stair_pose_->position.z = Stair_.transition_point_.z;

        tf2::Matrix3x3 tf_rotation;
        tf2::Quaternion tf_quaternion;
        geometry_msgs::msg::Quaternion ros_quaternion;
        tf_rotation.setValue(static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(0, 0)), static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(0, 1)), static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(0, 2)),
                        static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(1, 0)), static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(1, 1)), static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(1, 2)),
                        static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(2, 0)), static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(2, 1)), static_cast<double>(Stair_.Planes_[level_index_].plane_dir_(2, 2)));

        // tf_rotation.getRotation(tf_quaternion);
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

        stair_stamped_msg.stair.id = Stair_.type_;
        stair_stamped_msg.stair.distance = Stair_.step_distance_;
        stair_stamped_msg.stair.height = Stair_.step_height_;
        stair_stamped_msg.stair.angle = Stair_.step_angle_;
        stair_stamped_msg.stair.length = Stair_.step_length_;
        stair_stamped_msg.stair.width = Stair_.step_width_;
        stair_stamped_msg.stair.pose = *stair_pose_;

        stair_pub_->publish(stair_stamped_msg);
    }

        void StairModeling::publishStairWithDet(const std::string& cloud_frame)
    {   
        
        zion_msgs::msg::StairDetStamped stair_det_stamped_msg; //with or without ::msg ?
        
        // declare stair_stamped_msg
        stair_det_stamped_msg.header.frame_id = cloud_frame;
        stair_det_stamped_msg.header.stamp = this->get_clock()->now();

        // declare stair_msg

        stair_det_stamped_msg.stair.id = Stair_.type_;
        stair_det_stamped_msg.stair.distance = Stair_.step_distance_;
        stair_det_stamped_msg.stair.height = Stair_.step_height_;
        stair_det_stamped_msg.stair.angle = Stair_.step_angle_;
        stair_det_stamped_msg.stair.length = Stair_.step_length_;
        stair_det_stamped_msg.stair.width = Stair_.step_width_;
        stair_det_stamped_msg.stair.pose = *stair_pose_;

        stair_det_stamped_msg.bbox = det_buffer_->bbox;

        stair_det_fused_pub_->publish(stair_det_stamped_msg);
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

    void StairModeling::detCallback(const vision_msgs::msg::Detection2D::SharedPtr det_msg)
    {
        det_buffer_ = det_msg;
        // RCLCPP_INFO(get_logger(),"Got detection msg!");
        if (det_buffer_->results.size() == 0){
            debug_msg_ = debug_msg_ + "\nDetection empty" ;
            det_found_ = false;
        }
        else{
            debug_msg_ = debug_msg_ + "\nDetection is: " +  det_buffer_->results.begin()->id.c_str();
            det_found_ = true;
        }
    }

    void StairModeling::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    {
            reset();

            // from ros msg 
            pcl::fromROSMsg(*pcl_msg, *cloud_);

            // RCLCPP_INFO(get_logger(),"Got pcl msg!");

            if (!use_det_){
                // RANSAC-based plane segmentation
                getPlanes(cloud_);

                if(!planes_empty_){
                    // find the floor plane
                    
                    findFloor();
                    if(Planes_.size()>1){getStair();}
                    checkForValidStair();

                    // if stair detected set the stair instance and compute geometric parameters
                    if(stair_detected_){
                        // getStair();
                        // setStairTf();
                        getStairPose();
                                    // Publish the processed point cloud and custom msgs
                        publishHullsAsMarkerArray(output_frame_);
                        publishStairPose(output_frame_);
                        publishStair(output_frame_);

                    }
                }
            }
            else{
                // RCLCPP_INFO(get_logger(), "Detection found = %s", det_found_ ? "true" : "false");
                if (det_found_){
                    debug_msg_ = debug_msg_ + "\nUsing Stair detection Node -> Detection found";   

                    // RANSAC-based plane segmentation
                    getPlanes(cloud_);
                                // debug msg

                    if(!planes_empty_){
                        // find the floor plane
                        findFloor();
                        if(Planes_.size()>1){getStair();}
                        checkForValidStair();

                        // if stair detected set the stair instance and compute geometric parameters
                        if(stair_detected_){
                            // getStair();
                            // setStairTf();
                            getStairPose();
                            // Publish the processed point cloud and custom msgs
                            publishHullsAsMarkerArray(output_frame_);
                            publishStairPose(output_frame_);
                            publishStairWithDet(output_frame_);
                        }
                    }
                }
                else{
                    debug_msg_ = debug_msg_ + "\nUsing Stair detection Node -> No detection found yet";
                }   
            }

            det_found_ = false;

            // debug msg
            if(debug_){
                printDebug();
            }
    }



} // namespace

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zion::StairModeling)
