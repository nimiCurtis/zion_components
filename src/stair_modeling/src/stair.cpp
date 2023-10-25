// #include "zion_components/stair.h"
#include "stair.h"

void Stair::calcStairHeightAndDist(double yThreshold, int x_neighbors)
{
    // Calculate the height of the stair by taking the absolute difference
    // between the 3rd coefficients of the plane equations of the first two planes
    step_height_ = fabs(Planes_[0].plane_coefficients_->values[3] - Planes_[1].plane_coefficients_->values[3]);

    // Check if the stair is upwards
    if(type_ == 0)
    {
        // If upwards, calculate the distance by finding the average x-coordinate
        // of the points below yThreshold in the second plane's cloud
        step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_[1].cloud_, yThreshold, x_neighbors, true);
    }
    else // The stair is downwards
    {
        // If downwards, calculate the distance by finding the average x-coordinate
        // of the points below yThreshold in the second plane's cloud
        step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_[1].cloud_, yThreshold, x_neighbors, false);
    }
}

bool Stair::checkValidHeight(float source_plane_h, float target_plane_h)
{   
    float diff = std::abs(source_plane_h - target_plane_h);
    if (k_height_min <= diff && diff <= k_height_max)
    {
        return true;
    }
    else{
        return false;
    }
}