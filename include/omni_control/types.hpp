#pragma once
#include <iostream>
#include <Eigen/Core>
#include <cstddef>

namespace omni_direction
{
    struct Twist2D
    {
        double vx{0.0};
        double vy{0.0};
        double omega{0.0};
    };

    struct Pose2d
    {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    struct WheelState
    {
        Eigen::VectorXd speeds;

        explicit WheelState(std::size_t num_wheels) : speeds(num_wheels)
        {
            speeds.setZero();
        }    
        

        double& operator[](const int i)
        {
            return speeds[i];
        }

        const double&  operator[](const int i) const
        {
            return speeds[i];
        }

        std::size_t numWheels() const
        {
            return speeds.size();
        }

    };
}