// ReactionWheel.cpp
#include "ReactionWheel.hpp"

ReactionWheel::ReactionWheel(double wheelInertia)
        : inertia_(wheelInertia),
          angularVelocity_(0.0)
{}

void ReactionWheel::applyTorque(double torque, double deltaTime) {
    double angularAcceleration = torque / inertia_;
    angularVelocity_ += angularAcceleration * deltaTime;
}

double ReactionWheel::getAngularVelocity() const {
    return angularVelocity_;
}
