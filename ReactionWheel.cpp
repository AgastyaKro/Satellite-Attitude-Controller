#include "ReactionWheel.hpp"

ReactionWheel::ReactionWheel(double inertia) : inertia_(inertia), 
                                            angular_velocity_(0.0), last_applied_torque_(0.0) {}

void ReactionWheel::applyTorque(double torque, double dt){ 
    // angular acc = torque / inertia
    double angular_acceleration = torque / inertia_;
    angular_velocity_ += angular_acceleration * dt;
    last_applied_torque_ = torque;
}

double ReactionWheel::getReactionTorque() const {
    return -last_applied_torque_;
}

double ReactionWheel::getAngularVelocity () const {
    return angular_velocity_;
}