// Copyright 2022 Nimrod Curtis
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