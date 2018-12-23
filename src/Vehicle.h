#include <stdlib.h>
#include <stdio.h>
#include <vector>

using namespace std;

class Vehicle {
private:
    double x_;

    double y_;

    double s_;

    double d_;

    double yaw_;

    double speed_;

    vector<double> prev_path_x;

    vector<double> prev_path_y;

    int lane;

    double ref_vel_;

public:
    Vehicle();

    void Init();

    void Update();

    /* Actions */
    void turnLeft();

    void turnRight();

    void speedUp( );

    void slowDown( );

    void doNothing( );

};
