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
        Hysteresis() {

			_lower_threshold = 0;
			_upper_threshold = 0;
			_lower_output = 0;
			_upper_output = 0;
			_last_state = 0;
			_last_output = 0;

		}

        Hysteresis(T lower_threshold,
                   T upper_threshold,
                   T lower_output,
                   T upper_output,
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

        virtual void set(T lower_threshold,
                 T upper_threshold,
                 T lower_output,
                 T upper_output)
        {
            _lower_threshold = lower_threshold;
            _upper_threshold = upper_threshold;
            _lower_output = lower_output;
            _upper_output = upper_output;

        }

        virtual T apply(T state)
        {
            T output;

            if (state > _last_state)
            {
                if (state >= _upper_threshold)
                    output = _upper_output;
                else
                    output = _last_output;
            }
            else
            {
                if (state <= _lower_threshold)
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

    /**
      * Implements gradient from upper threshold to lower threshold
      * when the upper threshold is breached by a decreasing signal
      */
    template<typename T>
    class GradientHysteresis : public Hysteresis<T> {
    public:
        GradientHysteresis()
            :Hysteresis<T>()
            ,_interpolate(false)
        {}

        virtual void set(T lower_threshold, T upper_threshold, T lower_output, T upper_output)
        {
            if (upper_output < lower_output)
                _sign = T(-1);
            else
                _sign = T(1);

            Hysteresis<T>::set(_sign*lower_threshold,_sign*upper_threshold,_sign*lower_output,_sign*upper_output);
        }

        virtual T apply(T state)
        {
            state = _sign * state;

            T output;

            if (state > Hysteresis<T>::_last_state)
            {
                if (state >= Hysteresis<T>::_upper_threshold) {
                    output = Hysteresis<T>::_upper_output;
                    _interpolate = true;
                }
                else {

                    if (state >= Hysteresis<T>::_lower_threshold && _interpolate)
                        output = func(state);
                    else
                        output = Hysteresis<T>::_last_output;
                }
            }
            else
            {
                if (state <= Hysteresis<T>::_lower_threshold) {
                    output = Hysteresis<T>::_lower_output;
                    _interpolate = false;
                }
                else {

                    if (state <= Hysteresis<T>::_upper_threshold && _interpolate)
                        output = func(state);
                    else
                        output = Hysteresis<T>::_last_output;
                }
            }

            Hysteresis<T>::_last_output = output;
            Hysteresis<T>::_last_state = state;

            return _sign*output;
        }

    protected:
        bool _interpolate;
        T _sign;
        T func(T state) {
            double x = (double)(state - Hysteresis<T>::_lower_threshold) / (double)(Hysteresis<T>::_upper_threshold - Hysteresis<T>::_lower_threshold);
            double o = (double)Hysteresis<T>::_lower_output + (double)(Hysteresis<T>::_upper_output - Hysteresis<T>::_lower_output)*x;

            return common::Clamp<T>(o, Hysteresis<T>::_lower_output, Hysteresis<T>::_upper_output);
        }
    };


    void test_gradient_hysteresis()
    {
        ROS_INFO("Positive input..............................................");
        GradientHysteresis<int> hyst_pos;
        hyst_pos.set(0,10,0,50);

        for(int i = 0; i < 15; ++i)
            ROS_INFO("i=%d \t o=%d", i, hyst_pos.apply(i));

        for(int i = 15; i > 2; --i )
            ROS_INFO("i=%d \t o=%d", i, hyst_pos.apply(i));

        for(int i = 2; i < 15; ++i)
            ROS_INFO("i=%d \t o=%d", i, hyst_pos.apply(i));

        for(int i = 15; i > -5; --i)
            ROS_INFO("i=%d \t o=%d", i, hyst_pos.apply(i));

        for(int i = -5; i < 10; ++i)
            ROS_INFO("i=%d \t o=%d", i, hyst_pos.apply(i));

        ROS_INFO("\n\nNegative input..............................................");
        GradientHysteresis<int> hyst;
        hyst.set(0,-10,0,-50);

        for(int i = 0; i > -15; --i)
            ROS_INFO("i=%d \t o=%d", i, hyst.apply(i));

        for(int i = -15; i < -2; ++i )
            ROS_INFO("i=%d \t o=%d", i, hyst.apply(i));

        for(int i = -2; i > -15; --i)
            ROS_INFO("i=%d \t o=%d", i, hyst.apply(i));

        for(int i = -15; i < 5; ++i)
            ROS_INFO("i=%d \t o=%d", i, hyst.apply(i));

        for(int i = 5; i > -10; --i)
            ROS_INFO("i=%d \t o=%d", i, hyst.apply(i));
    }

}

#endif // COMMON_UTIL_H
