#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>
#include <eigen3/Eigen/Core>

namespace robot {

namespace dim {
    const double robot_diameter = 0.23;
    const double robot_height = 0.28;
    const double wheel_radius = 0.049;
    const double wheel_distance = 0.21;
}

namespace prop {
    const double encoder_publish_frequency = 10.0;
    const double ticks_per_rev = 360.0;
    const double max_velocity = 5.0;
    const double max_rot_velocity = M_PI/4.0;
}

namespace ir {
    const int id_front_left = 0;
    const int id_front_right = 1;
    const int id_rear_left = 2;
    const int id_rear_right = 3;
    
    //in m
    const double offset_front_left = dim::robot_diameter/2.0 - 0.023;
    const double offset_front_right = dim::robot_diameter/2.0 - 0.024;
    const double offset_rear_left = dim::robot_diameter/2.0 - 0.034;
    const double offset_rear_right = dim::robot_diameter/2.0 - 0.032;

    static double distance(int id, int voltage) {
        switch(id) {
            case id_front_left:
            {
                return 0;
            }
            case id_front_right:
            {
                return 0;
            }
            case id_rear_left:
            {
                return 0;
            }
            case id_rear_right:
            {
                return 0;
            }
            default:
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
}

/**
  * Calculates the number of ticks per wheel to achieve the desired rotation.
  * @param rot Desired rotation in degrees
  * @return Vector2d, where the first coefficient denotes the delta ticks for
  *         the left wheel. The values are rounded down.
  */
static Vector2i Delta_ticks_for_rotation(double rot) {

    //boil rotation down to +-180 deg
    for(;rot >  180.0; rot -= 360.0);
    for(;rot < -180.0; rot += 360.0);

    double turn_circumference = M_PI*dim::robot_diameter;
    double s = turn_circumference * (rot/360.0);

    double s_per_rev = 2.0*M_PI*dim::wheel_radius;
    int delta_ticks_right = (int) ( prop::ticks_per_rev * (s / s_per_rev));
    int delta_ticks_left  = -delta_ticks_right;

    Vector2i delta_ticks(delta_ticks_left, delta_ticks_right);
    return delta_ticks;
}

}

#endif // ROBOT_H
