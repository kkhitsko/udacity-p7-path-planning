#include "Vehicle.h"


Vehicle::Vehicle() {

}

void Vehicle::Init() {

}

void Vehicle::turnLeft() {
    lane -= 1;
}

void Vehicle::turnRight() {
    lane += 1;

}

void Vehicle::speedUp() {
    ref_vel_ += 1.5*0.224;
}

void Vehicle::slowDown() {
    ref_vel_ -= 1.5*0.224;
    printf("Decrease speed. New speed: %f\n", ref_vel_);
}

void Vehicle::doNothing() {

}
