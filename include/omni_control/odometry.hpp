#pragma once

#include <omni_control/types.hpp>
#include <omni_control/kinematics.hpp>

namespace omni_control
{
    class Odometry
    {
        public:
            explicit Odometry(const Kinematics & kinematics);
            bool update(const WheelState & ws, double dt);
            Pose2D getPose() const;
            Twist2D getTwist() const;
            void reset(const Pose2D & pose=Pose2D{});
        
        private:
            Kinematics kinematics_;
            Pose2D current_pose_{};
            Twist2D current_twist_{};
    };
}