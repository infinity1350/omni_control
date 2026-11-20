#pragma once

#include <omni_control/geometry.hpp>
#include <omni_control/types.hpp>
#include <Eigen/Dense>

namespace omni_control
{
    class Kinematics
    {
        public:
            explicit Kinematics(const Geometry& geometry);
            WheelState computeWheelSpeeds(const Twist2D& twist) const;
            Twist2D computeTwist() const;


        private:
            Geometry geometry_;
            Eigen::MatrixXd jacobian_;
            static double cross2D(double a, double b);
    };
}