#include "rotation.h"
#include <iostream>

using namespace std;

// Constructor to initialize group2DObjects
Rotation::Rotation(const vector<Group2D> &group2DObjects) : group2DObjects(group2DObjects) {}

// Function to calculate and return "no-go zones"
vector<double> Rotation::getNoGoZones()
{
    vector<double> noGoZones;

    for (const auto &group2D : group2DObjects)
    {
        if (!isLeader(group2D)) // Process obstacles only
        {
                ItemPoint obstacleCenter = group2D.getCenterPoint(); // Get the center point of the obstacle
                double obstacleAngle = obstacleCenter.getAngle();  // Get the angle of the obstacle
                noGoZones.push_back(obstacleAngle);                // Add angle to no-go zones
        }
    }

    return noGoZones;
}

// Function to calculate and return the leader's bearing
double Rotation::getLeaderBearing()
{
    for (const auto &group2D : group2DObjects)
    {
        if (isLeader(group2D))
        {                                                                     // Process only the leader
            ItemPoint leaderCenter = group2D.getCenterPoint(); 
            return leaderCenter.getAngle();                                   // Return the leader's bearing
        }
    }
    return -1; // Return -1 if no leader is found
}
