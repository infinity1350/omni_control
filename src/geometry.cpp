#include <geometry.hpp>
#include <cmath>

namespace omni_direction
{
    Geometry::Geometry(double radius,
        int num_wheels,
        int robot_radius)
        :radius_(radius), num_wheel_(num_wheels), robot_radius_(robot_radius) 
    {
        computeWheelGeometry()
    }

    void Geometry::computeWheelGeometry()
    {
        
        for(int i = 0; i < num_wheel_; i++)
        {
            WheelGeometry wheel;
            double theta = (2 * M_PI * i)/num_wheel_;
            wheel.position  = {robot_radius_ * std::cos(theta), robot_radius_ * std::sin(theta)};
            wheel.radius = robot_radius_;
            
        }
    }

    int Geometry::getNumWheel()
    {
        return num_wheel_;
    }

    WheelGeometry& Geometry::getWheel(int index)
    {
        if(index >= num_wheel_)
        {   
            std::cout << "The index for wheel is out of bounds\n";
            return nullptr(WheelGeometry);
        }

        return wheels_.at(index);
    
    }

}