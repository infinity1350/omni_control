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
        wheels_.clear();
        wheels_.reserver(num_wheel_);
        
        for(int i = 0; i < num_wheel_; i++)
        {
            double theta = (2 * M_PI / num_wheel_) * i;
            Eigen::Vector2d positions(
                radius_ * std::math.cos(theta), radius_ * std::math.sin(theta);
            );
            Eigen::Vector2d rolling_dir_unnormalized(-positions.y(), position.x());
            Eigen::Vector2d rolling_dir = rolling_direction_unormalized.normalized();

            WheelGeometry w;
            w.angle = theta;
            w.radius = radius_;
            w.position = positions;
            w.rolling_direction_ = rolling_dir;

            wheels_.push_back(w);

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