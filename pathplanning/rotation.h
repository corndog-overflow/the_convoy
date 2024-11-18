#ifndef ROTATION_H
#define ROTATION_H

#include <vector>
#include "Group2D.h" // Include the Group2D header

using namespace std;

class Rotation {
private:
    vector<Group2D> group2DObjects; // Vector of Group2D objects

public:
    // Constructor
    Rotation(const vector<Group2D>& group2DObjects);

    // Functions to calculate no-go zones and leader bearing
    vector<double> getNoGoZones();
    double getLeaderBearing();
};

#endif // ROTATION_H
