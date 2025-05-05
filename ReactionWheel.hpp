#pragma once
#include <Eigen/Dense>


class ReactionWheel {
    public:
        ReactionWheel(double inertia); 

        // Apply motor torque to the wheel and update it's speed
        void applyTorque(double torque, double dt);

        // Get torque which needs to applied to satellite 
        double getReactionTorque() const;

        // Get current wheel speed
        double getAngularVelocity() const;


    private:
        double inertia_;    // Moment of inertia of the wheel (scalar)
        double angular_velocity_; // Wheel's current spin rate (rad/s)
        double last_applied_torque_; // Last applied torque for satellite reaction

};