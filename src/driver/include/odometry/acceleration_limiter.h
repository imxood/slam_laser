#ifndef Robot_ACCELERATION_LIMITER_H_
#define Robot_ACCELERATION_LIMITER_H_

#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <ecl/time.hpp>

namespace robot
{

class AccelerationLimiter
{
  public:
    AccelerationLimiter() : is_enabled(true),
                            last_speed(0),
                            last_timestamp(ecl::TimeStamp()),
                            last_vx(0.0),
                            last_wz(0.0)
    {
    }
    void init(bool enable_acceleration_limiter, double linear_acceleration_max_ = 0.2, double angular_acceleration_max_ = 2, double linear_deceleration_max_ = -0.2 * 1.2, double angular_deceleration_max_ = -2 * 1.2)
    {
        is_enabled = enable_acceleration_limiter;
        linear_acceleration_max = linear_acceleration_max_;
        linear_deceleration_max = linear_deceleration_max_;
        angular_acceleration_max = angular_acceleration_max_;
        angular_deceleration_max = angular_deceleration_max_;
    }

    bool isEnabled() const { return is_enabled; }

    /**
   * @brief Limits the input velocity commands if gatekeeper is enabled.
   *
   * What is the limit?
   *
   * @param command : translation and angular velocity components in a 2-dim vector.
   */
    std::vector<double> limit(const std::vector<double> &command) { return limit(command[0], command[1]); }

    std::vector<double> limit(const double &vx, const double &wz)
    {
        if (is_enabled)
        {
            //get current time
            ecl::TimeStamp curr_timestamp;
            //get time difference
            ecl::TimeStamp duration = curr_timestamp - last_timestamp;
            //calculate acceleration
            double linear_acceleration = ((double)(vx - last_vx)) / duration;  // in [m/s^2]
            double angular_acceleration = ((double)(wz - last_wz)) / duration; // in [rad/s^2]

            //std::ostringstream oss;
            //oss << std::fixed << std::setprecision(4);
            //oss << "[" << std::setw(6) << (double)duration << "]";
            //oss << "[" << std::setw(6) << last_vx << ", " << std::setw(6) << last_wz << "]";
            //oss << "[" << std::setw(6) << vx << ", " << std::setw(6) << wz << "]";
            //oss << "[" << std::setw(6) << linear_acceleration << ", " << std::setw(6) << angular_acceleration << "]";

            if (linear_acceleration > linear_acceleration_max)
                command_vx = last_vx + linear_acceleration_max * duration;
            else if (linear_acceleration < linear_deceleration_max)
                command_vx = last_vx + linear_deceleration_max * duration;
            else
                command_vx = vx;
            last_vx = command_vx;

            if (angular_acceleration > angular_acceleration_max)
                command_wz = last_wz + angular_acceleration_max * duration;
            else if (angular_acceleration < angular_deceleration_max)
                command_wz = last_wz + angular_deceleration_max * duration;
            else
                command_wz = wz;
            last_wz = command_wz;

            last_timestamp = curr_timestamp;

            //oss << "[" << std::setw(6) << command_vx << ", " << std::setw(6) << command_wz << "]";
            //std::cout << oss.str() << std::endl;

            std::vector<double> ret_val;
            ret_val.push_back(command_vx);
            ret_val.push_back(command_wz);
            return ret_val;
        }
    }

    std::vector<double> limit_back(const std::vector<double> &command) { return limit_back(command[0], command[1]); }
    bool flag = true;
    std::vector<double> limit_back(const double &vx, const double &wz)
    {
        
        if (is_enabled)
        {
            //get current time
            ecl::TimeStamp curr_timestamp;
            //get time difference
            ecl::TimeStamp duration = curr_timestamp - last_timestamp;
            //calculate acceleration
            double linear_acceleration = ((double)(vx - last_vx)) / duration;  // in [m/s^2]
            double angular_acceleration = ((double)(wz - last_wz)) / duration; // in [rad/s^2]

            //std::ostringstream oss;
            //oss << std::fixed << std::setprecision(4);
            //oss << "[" << std::setw(6) << (double)duration << "]";
            //oss << "[" << std::setw(6) << last_vx << ", " << std::setw(6) << last_wz << "]";
            //oss << "[" << std::setw(6) << vx << ", " << std::setw(6) << wz << "]";
            //oss << "[" << std::setw(6) << linear_acceleration << ", " << std::setw(6) << angular_acceleration << "]";
            if(last_vx > (vx / 4.0) || last_wz > (wz / 4.0))
            {
                last_vx = 0;
                last_wz = 0;
                
                flag = true;
            }
                
            if (flag)
            {
                if (linear_acceleration > linear_acceleration_max)
                    command_vx = last_vx + linear_acceleration_max * duration*0.2;
                else if (linear_acceleration < linear_deceleration_max)
                    command_vx = last_vx + linear_deceleration_max * duration*0.2;
                else
                    command_vx = vx;
                last_vx = command_vx;

                if (angular_acceleration > angular_acceleration_max)
                    command_wz = last_wz + angular_acceleration_max * duration*0.2;
                else if (angular_acceleration < angular_deceleration_max)
                    command_wz = last_wz + angular_deceleration_max * duration*0.2;
                else
                    command_wz = wz;
                last_wz = command_wz;

                if (command_vx >= vx / 4.0)
                {
                    flag = false;
                }
            }

            last_timestamp = curr_timestamp;

            //oss << "[" << std::setw(6) << command_vx << ", " << std::setw(6) << command_wz << "]";
            //std::cout << oss.str() << std::endl;

            if (0)
            {
                if (linear_acceleration > linear_acceleration_max)
                    command_vx = last_vx - linear_acceleration_max * duration*0.5;
                else if (linear_acceleration < linear_deceleration_max)
                    command_vx = last_vx - linear_deceleration_max * duration*0.5;
                else
                    command_vx = vx;

                if (command_vx <= vx / 8.0)
                    flag = false;
            }

            std::vector<double> ret_val;
            ret_val.push_back(command_vx);
            ret_val.push_back(command_wz);
            return ret_val;
        }
    }

  private:
    bool is_enabled;
    short last_speed;
    short last_radius;
    //  unsigned short last_timestamp;
    ecl::TimeStamp last_timestamp;

    double last_vx, last_wz;                                   // In [m/s] and [rad/s]
    double command_vx, command_wz;                             // In [m/s] and [rad/s]
    double linear_acceleration_max, linear_deceleration_max;   // In [m/s^2]
    double angular_acceleration_max, angular_deceleration_max; // In [rad/s^2]
};

} // namespace kobuki

#endif /* Robot_ACCELERATION_LIMITER__HPP_ */
