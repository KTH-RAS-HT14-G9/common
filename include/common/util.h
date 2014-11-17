#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H

#include <ros/ros.h>

namespace common {

    template<typename T>
    inline T Clamp(T value, T min, T max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    template<typename T>
    class Hysteresis {
    public:
        Hysteresis(T lower_threshold,
                   T upper_threshold,
                   T upper_output,
                   T lower_output,
                   T initial_state,
                   T initial_output)

            :_lower_threshold(lower_threshold)
            ,_upper_threshold(upper_threshold)
            ,_upper_output(upper_output)
            ,_lower_output(lower_output)
            ,_last_state(initial_state)
            ,_last_output(initial_output)
        {
        }

        T apply(T state)
        {
            T output;

            if (state > _last_state)
            {
                if (state > _upper_threshold)
                    output = _upper_output;
                else
                    output = _last_output;
            }
            else
            {
                if (state < _lower_threshold)
                    output = _lower_output;
                else
                    output = _last_output;
            }

            _last_output = output;
            _last_state = state;

            return output;
        }

    protected:
        T _last_state;
        T _last_output;

        T _lower_threshold;
        T _upper_threshold;

        T _lower_output;
        T _upper_output;
    };

}

#endif // COMMON_UTIL_H
