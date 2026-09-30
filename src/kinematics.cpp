#include <kinematics.hpp>

namespace omni_direction
{
    explicit Kinematics::Kinematics(const Geometry& geometry)
        : geometry_(geometry)
    {
    }

    WheelState computeWheelSpeeds(const Twist2d & twist) const
    {
        WheelState wheel_speed;
        for(size_t i = 0; i <= geometry_.wheels.size(); i++)
        {
            WheelGeometry wheel = getwheel(i);
            wheel_speed[i] = (twist.vx * wheel.rolling_direction.x() 
                            + twist.vy * wheel.rolling_direction.y()
                            + robot_radius_* twist.omega) / wheel_radius_;
        }

        return wheel_speed;
    }

}