// #include "zion_components/plane.h"
#include "plane.h"
// #include <zion_components/plane.h>

// Compute the convex hull of the projected plane
void Plane::computePlaneHull()
{ 
  // Project the plane to 2D
  projectPlaneTo2D();

  // Initialize the ConvexHull object
  pcl::ConvexHull<pcl::PointXYZRGB> hull;
  hull.setInputCloud(cloud_projected_);

  // Reconstruct the convex hull for the projected plane
  hull.reconstruct(*hull_);
}

// Project the plane to 2D using the plane coefficients
void Plane::projectPlaneTo2D()
{
  // Initialize the ProjectInliers object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_);
  proj.setModelCoefficients(plane_coefficients_);
  
  // Perform the projection of the plane to 2D
  proj.filter(*cloud_projected_);
}

// Calculate the slope of the plane
void Plane::calcPlaneSlope()
{
  // Convert the first plane coefficient to degrees to get the slope
  slope_ = Utilities::rad2deg(plane_coefficients_->values[0]);
}

// Process the plane by setting its features and computing its convex hull
void Plane::processPlane()
{
    getCentroid();
    getPrincipalDirections();
    getMeasurements();
    computePlaneHull();
}

void Plane::getCentroid() 
{
    Eigen::Vector4f vector_centroid;
    pcl::compute3DCentroid(*cloud_,vector_centroid);
    centroid_.x = vector_centroid[0];
    centroid_.y = vector_centroid[1];
    centroid_.z = vector_centroid[2];
}

void Plane::getPrincipalDirections() {
	Eigen::Matrix3f covariance;
	Eigen::Vector4f eigen_centroid(centroid_.x,centroid_.y,centroid_.z,1);
	pcl::computeCovarianceMatrixNormalized(*cloud_,eigen_centroid,covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
  plane_dir_ = eigen_solver.eigenvectors();
  plane_dir_.col(2) = plane_dir_.col(0).cross(plane_dir_.col(1));  // Notice that column 0 has smaller eigenvalues, and thus direction perpendicular to the plane
}

void Plane::getMeasurements() {
    if (plane_dir_.isZero(0))
        this->getPrincipalDirections();

    // move the points to the that reference frame
    Eigen::Matrix4f cp2p(Eigen::Matrix4f::Identity()); // trnasformtaion: cp2p = camera projected ==> plane
    cp2p.block<3,3>(0,0) = plane_dir_.transpose();
    cp2p.block<3,1>(0,3) = -1.f * (cp2p.block<3,3>(0,0) * centroid_.getVector3fMap().head<3>());
    pcl::PointCloud<pcl::PointXYZRGB> cPoints;
    pcl::transformPointCloud(*cloud_, cPoints, cp2p);

    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    length_ = max_pt.y - min_pt.y; // third component, since main_dir has larger eigenvalue in third column
    width_ = max_pt.z - min_pt.z;
    // height = max_pt.x - min_pt.x; // first component, since main_dir has smaller eigenvalue in first column

    // Compute the center (of the bounding rectangle) and put it back in world reference
    Eigen::Affine3d p2cp = Eigen::Translation3d(centroid_.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(plane_dir_.cast<double>());
    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    pcl::transformPoint(mean_diag,mean_diag,p2cp.cast<float>());
    center_.x = mean_diag(0);
    center_.y = mean_diag(1);
    center_.z = mean_diag(2);


}