#include <iostream>
#include <cmath>
#include <omni_control/odometry.hpp>
#include <omni_control/types.hpp>

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

    bool Odometry::update(const WheelState & ws, double dt )
    {
        if(dt < 0)
        {
            return False
        }
        current_twist_ = kinematics_.computeTwist(ws);

    }

}
