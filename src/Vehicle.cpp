#include "Vehicle.h"
#include <math.h>
#include "common.h"


Vehicle::Vehicle( const double &s, const int& line, const double &ref_vel ):
    s_(s), lane(line), ref_vel_(ref_vel), prev_size_(0)
{

}


void Vehicle::Update( const double &s, const size_t& prev_size ) {
    s_ = s;
    prev_size_ = prev_size;
}

void Vehicle::turnLeft() {
    lane -= 1;
}

void Vehicle::turnRight() {
    lane += 1;

}

double Vehicle::getSpeed() const {
    return ref_vel_;
}

int Vehicle::getLane() const {
    return lane;
}

void Vehicle::speedUp() {
    ref_vel_ += SPEED_UP_STEP;
}

void Vehicle::slowDown() {
    ref_vel_ -= SLOW_DOWN_STEP;
}

void Vehicle::doNothing() {
    /* Don't change anything */
}

std::vector< Vehicle::Action > Vehicle::getAvailableActions() {
    std::vector< Action > actions = {Action::DO_NOTHING};
    if ( ref_vel_ > SLOW_DOWN_STEP ) {
        actions.push_back( Action::SLOW_DOWN );
    }

    if ( ref_vel_ < ( SPEED_LIMIT - SPEED_UP_STEP ) ) {
        actions.push_back( Action::SPEED_UP );
    }

    if ( lane > 0 ) {
        actions.push_back( Action::GO_LEFT );
    }

    if ( lane < 2 ) {
        actions.push_back( Action::GO_RIGHT );
    }
    //printf("Count of available actions: %ld\n", actions.size());
    return actions;
}

bool Vehicle::isVehicleClose( const int& new_lane, const vector<double>& other_car ) {
    //printf("Check line change %d safe\n", lane );
    bool is_close = false;


    double d = other_car[6];
    int car_id = int(other_car[0]);
    if ( ( d < 2 + 4*new_lane +2 ) && ( d > 2 + 4*new_lane - 2 ) ) {

        double vx = other_car[3];
        double vy = other_car[4];
        double check_speed = sqrt( vx*vx+vy*vy );
        double check_car_s = other_car[5];

        check_car_s += ((double)prev_size_*0.02*check_speed);
        auto distToCar = check_car_s - s_;

        if ( abs(distToCar) <= MIN_GAP  ) {
            is_close = true;
            //printf("Distance to car %d is %f. Lane change unsafe\n", car_id, abs(distToCar));
        }
    }
    return is_close;
}

bool Vehicle::isAnyVehicleClose(  const int& new_lane, const vector<vector<double> >& other_cars ) {
    bool is_close = false;
    for ( auto &other_car: other_cars ) {
        if ( isVehicleClose( new_lane, other_car ) ) is_close = true;
    }
    return is_close;
}


double Vehicle::getVehicleDistance( const int& new_lane, const vector<double>& other_car ) {
    //printf("Check line change %d safe\n", lane );
    double min_distance = 1000;


    double d = other_car[6];
    int car_id = int(other_car[0]);
    if ( ( d < 2 + 4*new_lane +2 ) && ( d > 2 + 4*new_lane - 2 ) ) {

        double vx = other_car[3];
        double vy = other_car[4];
        double check_speed = sqrt( vx*vx+vy*vy );
        double check_car_s = other_car[5];

        check_car_s += ((double)prev_size_*0.02*check_speed);
        auto distToCar = check_car_s - s_;

        if ( abs(distToCar) < abs(min_distance) ) {
            min_distance = distToCar;
        }
    }
    return min_distance;
}

double Vehicle::getMinVehicleDistance(  const int& new_lane, const vector<vector<double> >& other_cars ) {
    double min_distance = 1000;
    for ( auto &other_car: other_cars ) {
        auto distToCar = getVehicleDistance( new_lane, other_car );
        if ( abs(distToCar) < abs(min_distance) ) {
            min_distance = distToCar;
        }
    }
    return min_distance;
}

