#pragma once
#include <iostream>

namespace omni_kinematics
{
    struct Twist2D
    {
        double vx{0.0},
        double vy{0.0},
        double theta{0.0}
    };

    struct Pose2D
    {
        double x{0.0},
        double y{0.0},
        double theta{0.0}
    };

    struct WheelState
    {
        double w1{0.0},
        double w2{0.0},
        double w3{0.0}
    }
}