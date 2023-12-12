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
#include "stair.h"
#include <rclcpp/rclcpp.hpp>

// type = 0 -> floor is the lower = begin level = end
// type = 1 -> floor is the upper = end level = begin
void Stair::setStairPose()
{   
    Plane pose_plane = Planes_.front().get();;

    stair_pose_.position.x = step_distance_;
    stair_pose_.position.y = pose_plane.centroid_.y;
    stair_pose_.position.z = pose_plane.centroid_.z;

    setStairOrientation();
}

geometry_msgs::msg::Pose Stair::getStairPose() const
{
    return stair_pose_;
}

void Stair::setStairOrientation(){
    Plane level_plane;
    if (type_ == 0){
        level_plane = Planes_.front().get();
    }
    else{
        level_plane = Planes_.back().get();
    }

    tf2::Matrix3x3 tf_rotation;
    tf2::Quaternion tf_quaternion;
    geometry_msgs::msg::Quaternion ros_quaternion;
    tf_rotation.setValue(static_cast<double>(level_plane.plane_dir_(0, 0)), static_cast<double>(level_plane.plane_dir_(0, 1)), static_cast<double>(level_plane.plane_dir_(0, 2)),
                    static_cast<double>(level_plane.plane_dir_(1, 0)), static_cast<double>(level_plane.plane_dir_(1, 1)), static_cast<double>(level_plane.plane_dir_(1, 2)),
                    static_cast<double>(level_plane.plane_dir_(2, 0)), static_cast<double>(level_plane.plane_dir_(2, 1)), static_cast<double>(level_plane.plane_dir_(2, 2)));

    double roll ; double pitch ; double yaw;
    tf_rotation.getEulerYPR(yaw,pitch,roll);
    tf2::Quaternion q_rotation;
    if(pitch>=0){q_rotation.setRPY(0.,0.,pitch - M_PI/2);}
    else{q_rotation.setRPY(0.,0.,pitch + M_PI/2);}
    q_rotation.normalize();

    ros_quaternion = tf2::toMsg(q_rotation);
    stair_pose_.orientation = ros_quaternion;
}

geometry_msgs::msg::Quaternion Stair::getStairOrientation() const
{      
    return stair_pose_.orientation;
}

void Stair::TransformPoseToMap(Eigen::Affine3d& bp2m)
{
    Eigen::Affine3d pose_eigen;
    Eigen::Affine3d pose_eigen_in_map;

    pose_eigen = Utilities::PoseToEigen(stair_pose_);
    pose_eigen_in_map = bp2m * pose_eigen; 

    stair_pose_in_map_ = Utilities::EigenToPose(pose_eigen_in_map);

}

void Stair::TransformPoseToBase(Eigen::Affine3d& m2bp)
{
    Eigen::Affine3d pose_eigen;
    Eigen::Affine3d pose_eigen_in_map;

    pose_eigen_in_map = Utilities::PoseToEigen(stair_pose_in_map_);
    pose_eigen = m2bp * pose_eigen_in_map; 

    stair_pose_ = Utilities::EigenToPose(pose_eigen);
}


