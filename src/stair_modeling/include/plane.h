#ifndef PLANE_H
#define PLANE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <vector>

#include "utilities.h"

class Plane
{
public:

    Plane(){
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            cloud_projected_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            hull_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            plane_dir_.setZero();
    }

    /**
     * @brief Construct a new Plane object and processes it.
     * @param cloud_in Input point cloud.
     * @param plane_coeff Plane coefficients.
     */
    Plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::ModelCoefficients::Ptr plane_coeff) {
            plane_coefficients_ = plane_coeff;
            cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>() );
            cloud_projected_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            hull_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            plane_dir_.setZero();

            *cloud_ = *cloud_in;
            processPlane();
    }

    /**
     * @brief Processes the plane by setting its features and computing its convex hull.
     */
    void processPlane();

    /**
     * @brief Calculates the slope of the plane.
     */
    void calcPlaneSlope();

    // Members:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_;
    pcl::PointXYZRGB centroid_; // Plane centroid
    pcl::PointXYZRGB center_; // Rectangle center

    pcl::ModelCoefficients::Ptr plane_coefficients_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_;
    std::vector<float> features_;

    int type_;
    float slope_;
    float width_;
    float length_;
    Eigen::Matrix3f plane_dir_;


private:

    /**
     * @brief Computes the convex hull of the projected plane.
     */
    void computePlaneHull();

    /**
     * @brief Projects the plane to 2D using the plane coefficients.
     */
    void projectPlaneTo2D();

    /**
     * @brief Get plane centroid.
     */
    void getCentroid();

    /**
     * @brief Get plane PCA.
     */
    void getPrincipalDirections();

    /**
     * @brief Get plane width and length with relate to the pca direction.
     */
    void getMeasurements();

};

#endif // PLANE_H
