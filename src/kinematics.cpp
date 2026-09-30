#include <omni_control/kinematics.hpp>

namespace omni_direction
{
    Kinematics::Kinematics(const Geometry& geometry)
        : geometry_(geometry)
    {
    }

    WheelState Kinematics::computeWheelSpeeds(const Twist2D & twist) const
    {
        WheelState wheel_speed;
        for(size_t i = 0; i < geometry_.getNumWheel(); i++)
        {
            WheelGeometry wheel = geometry_.getWheel(i);
            wheel_speed[i] = (twist.vx * wheel.rolling_direction.x() 
                            + twist.vy * wheel.rolling_direction.y()
                            + geometry_.getRobotRadius() * twist.omega) / geometry_.getWheelRadius();
        }

        return wheel_speed;
    }

}