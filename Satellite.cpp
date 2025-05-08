#include "Satellite.hpp"
// #include <iostream> // for debugging later w/ std::cout

Satellite::Satellite() : orientation(Eigen::Quaterniond::Identity()), // w,x,y,z = 1,0,0,0
                         angular_velocity(Eigen::Vector3d::Zero()), // x,y,z = 0,0,0
                         inertia(Eigen::Matrix3d::Identity()), // 3 by 3 matrix, w/ diag as 1,1,1 else 0
                                                              // it's equally hard to rotate across x,y,z axis
                         inertia_tensor_inv(Eigen::Matrix3d::Identity().inverse()),
                         wheels{ ReactionWheel(0.1), ReactionWheel(0.1), ReactionWheel(0.1)}
                         {}

/* Old torque application process not including wheels
void Satellite::applyTorque(const Eigen::Vector3d& torque, double dt) { // torque + how long it was applied for
        Eigen::Vector3d angular_acceleration = inertia.inverse() * torque; // T/I
        angular_velocity += angular_acceleration * dt;
}
*/

void Satellite::applyWheelTorque(int wheel_index, double torque, double dt){
    if (wheel_index < 0 || wheel_index >= wheels.size())
        return;
    
    wheels[wheel_index].applyTorque(torque, dt);

    // applying reaction torque to the satellite
    Eigen::Vector3d torque_vector = Eigen::Vector3d::Zero();
    torque_vector[wheel_index] = wheels[wheel_index].getReactionTorque();

    applyBodyTorque(torque_vector, dt);
}


void Satellite::update(double dt){
    Eigen::Quaterniond omega(0, angular_velocity[0], angular_velocity[1], angular_velocity[2]); // angular vel -> quarternion
    Eigen::Quaterniond smallRotation = (omega * orientation); // how orientation changes right now derivative
    smallRotation.coeffs() *= 0.5 * dt; // scales derivative to time step
    orientation.coeffs() += smallRotation.coeffs(); // applies the small rotation
    orientation.normalize(); // normalizes to remove math error build-up
}

void Satellite::setTargetOrientation(const Eigen::Quaterniond& target){
    target_orientation = target; 
}

Eigen::Quaterniond Satellite::getOrientation() const{
    return orientation;
}

Eigen::Vector3d Satellite::getAngularVelocity() const {
    return angular_velocity;
}

