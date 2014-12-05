#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


namespace common {

    class LowPassFilter {
    public:
        LowPassFilter(double inertia) 
        :_inertia(inertia)
        ,_initialized(false)
        {}

        double init_to(double value) {
            _last_y = value;
        }

        double filter(double measurement) {
            if (_initialized == false) {
                _last_y = measurement;
                _initialized = true;
            }

            double y = (1.0-_inertia)*measurement + _inertia*_last_y;
            _last_y = y;

            return y;
        }

        void set_inertia(double inertia) {
            _inertia = inertia;
        }

    protected:
        double _inertia;
        double _last_y;
        bool _initialized;
    };

}

#endif // COMMON_OBB_H
