#include "utilities.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void Utilities::getCloudByInliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out,
                                  const pcl::PointIndices::Ptr &inliers,
                                  bool negative, bool organized)
{   
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setNegative(negative); // true = inverted behave
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setKeepOrganized(organized);
    extract.filter(*cloud_out);
}

void Utilities::transformCloud(Eigen::Affine3d c2cp,
                               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud)
{
    pcl::transformPointCloud(*input_cloud, *output_cloud, c2cp);
}

// Voxelization downsampling method
void Utilities::voxelizingDownsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud,
                                     float leaf_xy, float leaf_z)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(input_cloud);
    filter.setLeafSize(leaf_xy, leaf_xy, leaf_z);
    filter.filter(*output_cloud);
}

// smoothing method
void Utilities::smoothing(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud)
{
    // Smoothing object (we choose what point types we want as input and output).

    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> filter;
    filter.setInputCloud(input_cloud);
    // Use all neighbors in a radius of 3cm.transformC
    filter.setSearchRadius(0.05);
    // We can tell the algorithm to also compute smoothed normals (optional).
    filter.setComputeNormals(true);
    // kd-tree object for performing searches.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    filter.setSearchMethod(kdtree);
    filter.process(*output_cloud);
}

void Utilities::cropping(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud,
                         double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
    // Define cropping box dimensions
    double xmin = min_x, xmax = max_x, ymin = min_y, ymax = max_y, zmin = min_z, zmax = max_z;
    // Crop the point cloud
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setInputCloud(input_cloud);
    Eigen::Vector4f min_point, max_point;
    min_point << xmin, ymin, zmin, 1.0;
    max_point << xmax, ymax, zmax, 1.0;
    crop.setMin(min_point);
    crop.setMax(max_point);
    //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop.filter(*output_cloud);
}

void Utilities::removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud,
                               int mean_k, double stddev_mul_thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    sor.filter(*output_cloud);
}

void Utilities::euclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.05); // 2cm
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    // For simplicity, we will only keep the points in the largest cluster
    if (!cluster_indices.empty())
    {
        for (std::vector<int>::const_iterator pit = cluster_indices.front().indices.begin(); pit != cluster_indices.front().indices.end(); ++pit)
        {
            output_cloud->points.push_back(input_cloud->points[*pit]);
        }
        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = true;
    }
}

// Function to find the average x-coordinate of points below a specified y-coordinate threshold.
float Utilities::findAvgXForPointsBelowYThreshold(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                  double yThreshold, int x_neighbors, bool is_min)
{
    // Filter points based on y-coordinate threshold
    std::vector<pcl::PointXYZRGB> filteredPoints;
    for (const auto &point : cloud->points)
    {
        if (point.y < yThreshold)
        {
            filteredPoints.push_back(point);
        }
    }

    // Sort the filtered points by x-coordinate in ascending or descending order based on is_min
    if (is_min)
    {
        std::sort(filteredPoints.begin(), filteredPoints.end(),
                  [](const pcl::PointXYZRGB &a, const pcl::PointXYZRGB &b)
                  {
                      return a.x < b.x;
                  });
    }
    else
    {
        std::sort(filteredPoints.begin(), filteredPoints.end(),
                  [](const pcl::PointXYZRGB &a, const pcl::PointXYZRGB &b)
                  {
                      return a.x > b.x;
                  });
    }

    // Take the first x_neighbors points (or all points if fewer than x_neighbors)
    int numPointsToAverage = std::min(static_cast<int>(filteredPoints.size()), x_neighbors);

    // Calculate the average x-coordinate value
    double avgX = 0.0;
    for (int i = 0; i < numPointsToAverage; ++i)
    {
        avgX += filteredPoints[i].x;
    }

    if (numPointsToAverage > 0)
    {
        avgX /= numPointsToAverage;
    }

    return static_cast<float>(avgX);
}

// Function to convert degrees to radians.
float Utilities::deg2rad(float deg_angle)
{
    return (deg_angle * 0.017453293f);
}

// Function to convert radians to degrees.
float Utilities::rad2deg(float rad_angle)
{
    return (rad_angle / 0.017453293f);
}

// Function to colorize a point cloud.
void Utilities::colorize(const pcl::PointCloud<pcl::PointXYZRGB> &pc,
                         pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
                         const std::vector<int> &color)
{

    int N = pc.points.size();

    // Clear the colored point cloud
    pc_colored.clear();

    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i)
    {
        const auto &pt = pc.points[i];

        // Assign the coordinates and color to the temporary point
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];

        // Add the colored point to the colored point cloud
        pc_colored.points.emplace_back(pt_tmp);
    }
}

// int main(int argc, char *argv[])
// {
//     return 0;
// }