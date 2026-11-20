#include <iostream>
#include <cmath>
#include <omni_control/odometry.hpp>
#include <omni_control/types.hpp>
#include <numbers>

namespace omni_control
{
    Odometry::Odometry(const Kinematics & kinematics) : kinematics_(kinematics)
    {
    }

    Twist2D Odometry::getTwist() const
    {
        return current_twist_;
    }

    Pose2D Odometry::getPose() const
    {
        return current_pose_;
    }

    void Odometry::reset(const Pose2D & pose = Pose2D{})
    {
        current_pose_ = Pose2D{};
    }

    bool Odometry::update(const WheelState & ws, double dt)
    {
        //Step 1 : check whether dt is less than 0
        if(dt <= 0)
        {
            return false;
        }

        // Step 2 : Get the twist value of the robot
        auto twist = kinematics_.computeTwist(ws);

        if(!std::isfinite(twist.vx))
        {
            return false;
        }
        if(!std::isfinite(twist.vy))
        {
            return false;
        }
        if(!std::isfinite(twist.omega))
        {
            return false;
        }
        
        current_twist_ = twist;
        // Step 3 : average the global angle with the current reported twist angle value 
        auto theta_avg = current_pose_.theta + current_twist_.omega * dt / 2; 
        
        // Step 4 : Tranform the velocity to the global frame
        auto world_vx = current_twist_.vx * std::cos(theta_avg) - current_twist_.vy * std::sin(theta_avg);
        auto world_vy = current_twist_.vx * std::sin(theta_avg) + current_twist_.vy * std::cos(theta_avg);

        // Step 5 : integrate the values
        current_pose_.x += world_vx * dt;
        current_pose_.y += world_vy * dt;
        current_pose_.theta = std::remainder(current_twist_.omega * dt + current_pose_.theta, 2 * std::numbers::pi);      // normalising the value 
        
        return true;   
    }

}