// ReactionWheel.hpp
#ifndef REACTIONWHEEL_HPP
#define REACTIONWHEEL_HPP

class ReactionWheel {
public:
    explicit ReactionWheel(double wheelInertia = 1.0);

    // apply torque for deltaTime seconds
    void applyTorque(double torque, double deltaTime);

    // current wheel spin rate (rad/s)
    double getAngularVelocity() const;

private:
    double inertia_;
    double angularVelocity_;
};

#endif // REACTIONWHEEL_HPP
