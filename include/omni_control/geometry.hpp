#pragma once
#include <iostream>
#include <Eigen::Core>
#include <vector>
#include <Eigen/Dense>

namespace omni_direction
{
    
    struct WheelGeometry
    {
        double radius;
        Eigen::Vector2d position;
        Eigen::Vector2d rolling_direction;
        double angle;
    }
    class Geometry
    {
        public:    
            Geometry(double radius, int num_wheels, int robot_radius);
            
            WheelGeometry& getWheel(int index) const;

            int getNumWheel() const;

        private:
            double radius_;
            int num_wheel_;
            double robot_radius_;

            std::vector<WheelGeometry> wheels_;
            void computeWheelGeometry();

    };

    inline WheelGeometry& Geometry::getWheel(int index) const
    {
        if(index >= num_wheel_)
        {
            std::cout << "The index reference is invalid\n" 
            return nullptr;
        }

        return wheels_.at(index);

    }
}