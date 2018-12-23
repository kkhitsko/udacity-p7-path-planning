#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <stdlib.h>
#include <stdio.h>
#include <vector>

using namespace std;


static const double MIN_GAP = 30.0;

class Vehicle {

private:


    double s_;

    size_t prev_size_;

    int lane;

    double ref_vel_;

public:

    enum Action {
            DO_NOTHING = 0,
            SPEED_UP,
            SLOW_DOWN,
            GO_LEFT,
            GO_RIGHT
        };

    Vehicle( const double &s, const int& line, const double &ref_vel );

    void Update( const double &s, const size_t& prev_size );

    /* Actions */
    void turnLeft();

    void turnRight();

    void speedUp( );

    void slowDown( );

    void doNothing( );

    std::vector< Action > getAvailableActions();

    double getSpeed() const;

    int getLane() const;

    bool isVehicleClose( const int& new_lane, const vector<double>& other_car );

    bool isAnyVehicleClose(  const int& new_lane, const vector<vector<double> >& sensor_fusion );

    double getVehicleDistance( const int& new_lane, const vector<double>& other_car );

    double getMinVehicleDistance(  const int& new_lane, const vector<vector<double> >& sensor_fusion );

};

#endif
