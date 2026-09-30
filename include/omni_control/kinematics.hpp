#pragma once

#include <omni_control/geometry.hpp>
#include <omni_control/types.hpp>

namespace omni_direction
{
    class Kinematics
    {
        public:
            explicit Kinematics(const Geometry& geometry);

            WheelState computeWheelSpeeds(const Twist2D& twist) const;


        private:
            Geometry geometry_;
    };
}