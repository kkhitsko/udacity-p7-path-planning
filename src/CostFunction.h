#include "Vehicle.h"
#include <vector>
#include <string>


static const double GAP_COLLISION_COST = 100.0;
static const double CHANGE_LANE_COST = 3.0;
static const double LOW_SPEED_CHANGE_LANE_COST = 25.0;

static const double LOW_SPEED = 30.0;




class CostFunction {
private:

    double getSpeedCost ( const double speed );

    double getGapCollisionCost( const double dist, const double speed );



public:

    Vehicle::Action getMinCostAction( Vehicle vehicle, const vector<vector<double> >& sensor_fusion );

};

std::string getVehicleActionName( Vehicle::Action action );
