#include <omni_control/kinematics.hpp>

namespace omni_direction
{
    Kinematics::Kinematics(const Geometry& geometry)
        : geometry_(geometry)
    {
    }

    WheelState Kinematics::computeWheelSpeeds(const Twist2D & twist) const
    {
        const WheelState& wheel_speed(geometry_.getNumWheel());
        
        for(size_t i = 0; i < geometry_.getNumWheel(); i++)
        {
            WheelGeometry wheel = geometry_.getWheel(i);

            const double nx = wheel.rolling_direction.x();
            const double ny = wheel.rolling_direction.y();
        
            const double x = wheel.position.x();
            const double y = wheel.position.y();

            const double rotational = -nx * y + ny * x;
            wheel_speed[i] = (twist.vx * nx 
                            + twist.vy * ny
                            + rotational * twist.omega) / geometry_.getWheelRadius();
        }

        return wheel_speed;
    }

}