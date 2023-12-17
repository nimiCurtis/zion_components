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

#ifndef STAIR_H
#define STAIR_H

// Third party libraries
#include <vector>

// ROS application/library includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>

// Custom includes
#include "plane.h"
#include "utilities.h"

// Parameters for height of the stairs, given by the regulations:
const float k_height_min = 0.07f;  // Min height 
const float k_height_max = 1.f;    //1.f; 0.2f Max height 
// const float k_length_min = 0.1f;   // Min length is 20 cm (no max length)
const float k_area_min = 0.2f; // Min step area

class Stair
{   
    /**
    * TODO: 
    * 1.set the public protected private better
    */
    public:

        /**
         * @brief Constructor for the Stair class.
         *
         * This constructor initializes a Stair object using a vector of Plane objects.
         * It sorts the planes by height, calculates stair properties, and determines
         * whether it represents an upward or downward stair.
         *
         * @param planes A vector of Plane objects representing the stair steps.
         */
        Stair(const std::vector<Plane>& planes)
        {

            for (const Plane& plane : planes){
                Planes_.push_back(plane);
            } 

            auto compareByHeight = [](const Plane& plane1, const Plane& plane2) {
                return plane1.plane_coefficients_->values[3] < plane2.plane_coefficients_->values[3];
            };

            // Sorting by height 
            // The minimum height is at the beginning of the vector.
            std::sort(Planes_.begin(), Planes_.end(), compareByHeight);

            step_height_ = fabs(Planes_.front().plane_coefficients_->values[3] 
                                - Planes_.back().plane_coefficients_->values[3]);

            if(Planes_.back().type_ == 0){
                type_ = 0;
                step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_.front().cloud_,
                                                                        0.055, 30, true);
                step_length_ = planes.front().length_;
                step_width_ = planes.front().width_;

            }else{
                type_ = 1;
                step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_.front().cloud_,
                                                                        0.055, 30, false);
                step_length_ = planes.back().length_;
                step_width_ = planes.back().width_;
            }

            setStairPose();
            step_angle_ = Utilities::rad2deg(std::atan2(stair_pose_.position.y,
                                                                    stair_pose_.position.x));                                             
        }

        /**
         * TODO: 
         * 1.Refacor this constructor
         * @brief Default constructor. Initializes stair parameters to zero.
         */
        Stair(){
                step_distance_ = 0.;
                step_height_ = 0.;
                step_width_ = 0.;
                step_length_ = 0.;
                step_angle_ = 0.;
            }

        /**
         * @brief Destructor.
         */
        ~Stair(){}

        /**
         * @brief Get a reference to the current Stair object.
         * @return A reference to the current Stair object.
         */
        Stair& get(){
            return *this;
        }

        /**
         * @brief Get the orientation of the stair in quaternion representation.
         *
         * @return The orientation of the stair in quaternion.
         */
        geometry_msgs::msg::Quaternion getStairOrientation() const;

        /**
         * @brief Set the orientation of the stair.
         *
         * This function calculates and sets the orientation of the stair based on the planes
         * associated with the stair.
         */
        void setStairOrientation();

        /**
         * @brief Get the pose (position and orientation) of the stair.
         *
         * @return The pose of the stair.
         */
        geometry_msgs::msg::Pose getStairPose() const;

        /**
         * @brief Set the pose (position and orientation) of the stair.
         *
         * This function calculates and sets the pose of the stair based on the planes
         * associated with the stair. The position is determined by the step distance and the
         * centroid of the first plane in the stair, and the orientation is set by the
         * setStairOrientation() function.
         */
        void setStairPose();

        /**
         * @brief Transform the stair pose to the map frame.
         *
         * @param bp2m An Eigen Affine3d transformation representing the base to map transform.
         */
        void TransformPoseToMap(Eigen::Affine3d& bp2m);

        /**
         * @brief Transform the stair pose to the base frame.
         *
         * @param m2bp An Eigen Affine3d transformation representing the map to base transform.
         */
        void TransformPoseToBase(Eigen::Affine3d& m2bp);

        // Members:
        std::vector<Plane> Planes_; // Step candidates given by the detection process
        int type_;                  // 0 = upwards, 1 = downwards
        float step_width_;          // Width of the step
        float step_length_;         // Length of the step
        float step_height_;         // Height of the step
        float step_distance_;       // Distance between steps
        float step_angle_;          // Angle of the stair

        geometry_msgs::msg::Pose stair_pose_; ///< Pose of the detected stair.
        geometry_msgs::msg::Pose stair_pose_in_map_; ///< Pose of the detected stair in map frame.

    protected:

    private:
    // Currently no private members or methods are defined
};

#endif // STAIR_H
