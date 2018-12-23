#include "CostFunction.h"
#include <map>
#include <math.h>
#include "common.h"


 double CostFunction::getSpeedCost ( const double speed ) {
     double cost = ( SPEED_LIMIT - speed ) * ( SPEED_LIMIT - speed )/100;
     return cost;
 }

 double CostFunction::getGapCollisionCost( const double dist, const double speed ) {
     double cost = 0;
     if ( abs(dist) >= 30 ) return cost;
     if ( dist > 0 ) cost = 2*(30 - dist) + speed/2;
     if ( dist < 0 ) cost = 2*(30 + (0.5*dist)) + speed/2;
     return cost;
 }

Vehicle::Action CostFunction::getMinCostAction( Vehicle vehicle, const vector<vector<double> >& sensor_fusion ) {
    Vehicle::Action res_action = Vehicle::Action::DO_NOTHING;

    double min_cost = 100000;

    auto actions = vehicle.getAvailableActions();

    for ( auto &action: actions  ) {

        double action_cost = 0;

        double new_speed = vehicle.getSpeed();
        int new_lane = vehicle.getLane();

        switch( action ) {
            case Vehicle::Action::DO_NOTHING:
                //
                break;
            case Vehicle::Action::SPEED_UP:
                //
                new_speed +=  SPEED_UP_STEP;
                break;

            case Vehicle::Action::SLOW_DOWN:
                 //
                new_speed -= SLOW_DOWN_STEP;
                break;
            case Vehicle::Action::GO_LEFT:
                //
                new_lane -= 1;
                if ( new_speed > LOW_SPEED )
                    action_cost += CHANGE_LANE_COST;
                else
                    action_cost += LOW_SPEED_CHANGE_LANE_COST;
                break;
            case Vehicle::Action::GO_RIGHT:
                //
                new_lane += 1;
                if ( new_speed > LOW_SPEED )
                    action_cost += CHANGE_LANE_COST;
                else
                    action_cost += LOW_SPEED_CHANGE_LANE_COST;
                break;
            default:
                break;
        }

        action_cost += getSpeedCost( new_speed );

        auto dist = vehicle.getMinVehicleDistance( new_lane, sensor_fusion );

        action_cost += getGapCollisionCost( dist, new_speed );

        //printf("Min distance: %f; Action: %d(%s); cost: %f\n", dist, action, getVehicleActionName(action).c_str(),  action_cost );

        if ( action_cost < min_cost ) {
            res_action = action;
            min_cost = action_cost;

        }

    }
    printf("min cost: %f; action: [%d] -> %s \n", min_cost, res_action, getVehicleActionName(res_action).c_str());

    return res_action;
}

std::string getVehicleActionName( Vehicle::Action action ) {
    string action_name = "";
    switch ( action ) {
        case Vehicle::Action::DO_NOTHING:
            //
            action_name = "Do nothing";
            break;
        case Vehicle::Action::SPEED_UP:
            //
            action_name = "Speed up";
            break;
        case Vehicle::Action::SLOW_DOWN:
            //
            action_name = "Slow down";
            break;
        case Vehicle::Action::GO_LEFT:
            //
            action_name = "Go left lane";
            break;
        case Vehicle::Action::GO_RIGHT:
            //
            action_name = "Go right lane";
            break;
        default:
            break;
    }
    return action_name;
}
