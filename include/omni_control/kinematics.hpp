#pragma once

#include <iostream>
#include <omni_control/geometry.hpp>
#include <omni_control/types.hpp>

namespace omni_direction
{
    class Kinematics
    {
        public:
            explicit Kinematics(const Geometry& geometry);

            WheelState commputeWheelSpeeds(const Twist2D& twist) const;

        private:
            Geometry geometry_;
    };
}