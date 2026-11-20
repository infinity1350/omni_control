#include <omni_control/kinematics.hpp>

namespace omni_control
{
    Kinematics::Kinematics(const Geometry& geometry)
        : geometry_(geometry)
    {
        Eigen::Index rows = static_cast<Eigen::Index>(geometry_.getNumWheel());
        jacobian_ = Eigen::MatrixXd(rows, 3);


        for(Eigen::Index i = 0; i < rows; i++)
        {   
            const auto & w = geometry_.getWheel(i);
            
            jacobian_(i, 0) = w.rolling_direction.x();
            jacobian_(i, 1) = w.rolling_direction.y();
            jacobian_(i, 2) = Kinematics::cross2D(w.position, w.rolling_direction);
        }
    }

    WheelState Kinematics::computeWheelSpeeds(const Twist2D & twist) const
    {
        WheelState wheel_speed(geometry_.getNumWheel());
        
        for(Eigen::Index i = 0; i < static_cast<Eigen::Index>(geometry_.getNumWheel()); i++)
        {
            WheelGeometry wheel = geometry_.getWheel(i);

            const double nx = wheel.rolling_direction.x();
            const double ny = wheel.rolling_direction.y();
        
            const double x = wheel.position.x();
            const double y = wheel.position.y();

            const double rotational = -nx * y + ny * x;
            wheel_speed[i] = (twist.vx * nx 
                            + twist.vy * ny
                            + rotational * twist.omega) / geometry_.getWheelRadius();
        }

        return wheel_speed;
    }

    double Kinematics::cross2D(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
    {
        if(a.size() != 2 || b.size() != 2)
        {
            throw std::invalid_argument("2D cross product requires vectors of size 2.");
        }

        return (a.x() * b.y() - a.y() * b.x());
    }

    Twist2D Kinematics::computeTwist(const WheelState & ws)
    { 
        Eigen::VectorXd twist_v = jacobian_.colPivHouseholderQr().solve(ws.speeds);
        Twist2D twist;
        return twist{v[0], v[1], v[2]};
    }

}