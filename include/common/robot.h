#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>
#include <cmath>
#include <Eigen/Core>

namespace robot {

namespace dim {
    const double robot_diameter = 0.25;
    const double robot_height = 0.28;
    const double wheel_radius = 0.04975;
    //const double wheel_distance = 0.215;
    const double wheel_distance = 0.213; //corrected
    const double body_width = 0.163;
}

namespace prop {
    const double encoder_publish_frequency = 50.0;
    const double ticks_per_rev = 360.0;
    const double max_velocity = 5.0;
    const double max_rot_velocity = M_PI/4.0;
}

namespace ir {
    const int id_front_left = 1;
    const int id_front_right = 2;
    const int id_rear_left = 3;
    const int id_rear_right = 4;
    const int id_front_long_left = 7;
    const int id_front_long_right = 8;
    
    //in m
    const double offset_front_left = dim::body_width/2.0 - 0.021;
    const double offset_front_right = dim::body_width/2.0 - 0.020;
    const double offset_rear_left = dim::body_width/2.0 - 0.0305;
    const double offset_rear_right = dim::body_width/2.0 - 0.030;
    
    const double offset_front_left_forward = 0.084;
    const double offset_front_right_forward = 0.084;
    const double offset_rear_left_forward = 0.082;
    const double offset_rear_right_forward = 0.083;

    static double distance(int id, int voltage) {
        double dist= 0;
	double v = voltage;
        switch(id) {
            case id_front_left:
            {
                const double FL_a1 =  8.636*pow(10,16)  ;
                const double FL_b1 =  -3591  ;
                const double FL_c1 =  613.4 ;
                const double FL_a2 =  245.8 ;
                const double FL_b2 =  -2220;
                const double FL_c2 =  1402 ;
                
                dist =  0.01*(FL_a1*exp(-((v-FL_b1)/FL_c1)*((v-FL_b1)/FL_c1)) + FL_a2*exp(-((v-FL_b2)/FL_c2)*((v-FL_b2)/FL_c2)));
               
                if (std::isinf(dist) || dist < 0.04) return 0.04;
                return dist;
            }
            case id_front_right:
            {
                const double FR_a1 =  7.334*pow(10,15)  ;
                const double FR_b1 =  -4256  ;
                const double FR_c1 =  751.5 ;
                const double FR_a2 =  11.02 ;
                const double FR_b2 =  78.94;
                const double FR_c2 =  510.6 ;
                
                dist =  0.01*(FR_a1*exp(-((v-FR_b1)/FR_c1)*((v-FR_b1)/FR_c1)) + FR_a2*exp(-((v-FR_b2)/FR_c2)*((v-FR_b2)/FR_c2)));
               
                if (std::isinf(dist) || dist < 0.04) return 0.04;
                return dist;
            }
            case id_rear_left:
            {    
                const double BL_a=20080;
                const double BL_b=-1.44;
                const double BL_c=4.234;
                dist=0.01*(BL_a*pow(v,BL_b)+BL_c);

                if (std::isinf(dist) || dist < 0.04) return 0.04;
                return dist;
            }
            case id_rear_right:
            {      
                const double BR_a1 =  7.831*pow(10,15)  ;
                const double BR_b1 =  -5190  ;
                const double BR_c1 =  912.1 ;
                const double BR_a2 =  9.748 ;
                const double BR_b2 =  148.4;
                const double BR_c2 =  461.8 ;
                
                dist =  0.01*(BR_a1*exp(-((v-BR_b1)/BR_c1)*((v-BR_b1)/BR_c1)) + BR_a2*exp(-((v-BR_b2)/BR_c2)*((v-BR_b2)/BR_c2)));
               
                if (std::isinf(dist) || dist < 0.04) return 0.04;
                return dist;
            }
            case id_front_long_left:
            {   
                const double FL_L_a=5095;
                const double FL_L_b= -0.9736;
                const double FL_L_c=-0.7674;
                dist=0.01*(FL_L_a*pow(v,FL_L_b)+FL_L_c);

                if (std::isinf(dist) || dist < 0.04) return 0.1;
                return dist;
            }
            case id_front_long_right:
            {
              const double FR_L_a=9093;
                const double FR_L_b= -1.068;
                const double FR_L_c=-0.5847;
                dist=0.01*(FR_L_a*pow(v,FR_L_b)+FR_L_c);

                if (std::isinf(dist) || dist < 0.04) return 0.1;
                return dist;
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
static Eigen::Vector2i Delta_ticks_for_rotation(double rot) {

    //boil rotation down to +-180 deg
    for(;rot >  180.0; rot -= 360.0);
    for(;rot < -180.0; rot += 360.0);

    double turn_circumference = M_PI*dim::wheel_distance;
    double s = turn_circumference * (rot/360.0);

    double s_per_rev = 2.0*M_PI*dim::wheel_radius;
    int delta_ticks_right = (int) ( prop::ticks_per_rev * (s / s_per_rev));
    int delta_ticks_left  = -delta_ticks_right;

    Eigen::Vector2i delta_ticks(delta_ticks_left, delta_ticks_right);
    return delta_ticks;
}

}

#endif // ROBOT_H
