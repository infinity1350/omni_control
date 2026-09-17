#pragma once

#include <Eigen/Core>
#include <vector>
#include <Eigen/Dense>

namespace omni_direction
{
    
    struct WheelGeometry
    {
        Eigen::Vector2d position;
        Eigen::Vector2d rolling_direction;
        double angle;

    };
    class Geometry
    {
        public:    
            Geometry(double radius, int num_wheels, double robot_radius);
            
            const WheelGeometry& getWheel(int index) const;

            size_t getNumWheel() const;

        private:
            double wheel_radius_;
            int num_wheel_;
            double robot_radius_;
            void computeWheelGeometry();

            std::vector<WheelGeometry> wheels_;

    };

}