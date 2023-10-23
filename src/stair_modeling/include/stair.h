#ifndef STAIR_H
#define STAIR_H

// general includes
#include <vector>

// custom includes
#include "plane.h"
#include "utilities.h"

// Parameters for height of the stairs, given by the regulations:
const float k_height_min = 0.08f ;  // Min height 
const float k_height_max = 1.f ; //1.f; 0.2f Max height 
const float k_length_min = 0.1f; // Min length is 28 cm (no max length)

class Stair
{
public:

    /**
     * @brief Constructor that initializes the stair with given planes.
     * @param planes Vector of Plane objects representing the steps.
     */
    Stair(std::vector<Plane> planes){
            step_distance_ = 0.;
            step_height_ = 0.;
            step_width_ = 0.;
            step_length_ = 0.;
            step_angle_ = 0.;
            Planes_ = planes;
            for (int i=0; i< static_cast<int>(planes.size()); i++){
                Planes_.push_back(planes[i]);
            } 
        }
    
    /**
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
     * @brief Calculates the height and distance of the stair.
     * @param yThreshold Y-coordinate threshold for calculating average x-coordinate.
     * @param x_neighbors Number of neighbors to consider for averaging x-coordinate.
     */
    void calcStairHeightAndDist(double yThreshold, int x_neighbors);

    static bool checkValidHeight(float source_plane_h, float target_plane_h);

    // Members:
    std::vector<Plane> Planes_; // Step candidates given by the detection process
    int type_; // 0 = upwards, 1 = downwards
    float step_width_; // Width of the step
    float step_length_; // Length of the step
    float step_height_; // Height of the step
    float step_distance_; // Distance between steps
    float step_angle_; // Angle of the stair
    pcl::PointXYZRGB transition_point_ ; // the transition point between floor and level


private:
    // Currently no private members or methods are defined
};

#endif // STAIR_H
