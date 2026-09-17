#include <geometry.hpp>
#include <cmath>

namespace omni_direction
{
    Geometry::Geometry(double radius,
        int num_wheels,
        double robot_radius)
        :wheel_radius_(radius), num_wheel_(num_wheels), robot_wheel_radius_(robot_radius) 
    {
        if (num_wheels < 3)
            throw std::invalid_argument("Need at least 3 wheels");

        if (wheel_radius <= 0.0 || robot_radius <= 0.0)
            throw std::invalid_argument("Radius must be positive");
        
            computeWheelGeometry();

    }

    void Geometry::computeWheelGeometry()
    {
        wheels_.clear();
        wheels_.reserve(num_wheel_);
        
        for(int i = 0; i < num_wheel_; i++)
        {
            double theta = (2 * M_PI / num_wheel_) * i;
            Eigen::Vector2d positions(
                robot_radius_ * std::cos(theta), robot_radius_ * std::sin(theta)
            );
            Eigen::Vector2d rolling_direction_unnormalized(-positions.y(), positions.x());
            Eigen::Vector2d rolling_dir = rolling_direction_unormalized.normalized();

            WheelGeometry w;
            w.angle = theta;
            w.radius = wheel_radius_;
            w.position = positions;
            w.rolling_direction = rolling_dir;

            wheels_.push_back(w);

        }
    }

    int Geometry::getNumWheel() const
    {
        return num_wheel_;
    }

    const WheelGeometry& Geometry::getWheel(int index) const
    {
        
        return wheels_.at(index);
        
    }

}