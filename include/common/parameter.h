#ifndef PARAMETER_H
#define PARAMETER_H

#include <ros/ros.h>

/**
  * Handles any kind of ros parameter and ensures that it is always up to date.
  * Already set parameters are not overwritten when the class is instantiated.
  * Example:
  *
  * Parameter<double> param("/xyz/topic", 0.5);
  * double value = param();
  * value = foo();
  * param.set(value);
  */
template<typename T>
class Parameter {
public:
    Parameter(const std::string& key, T fallback_value);

    T operator() ();
    void set(T& value);

protected:
    std::string _key;
    T _value;
};

template<typename T>
Parameter<T>::Parameter(const std::string& key, T fallback_value)
    :_key(key)
{
    if(!ros::param::has(key)) {
        ros::param::set(_key,fallback_value);
    }
    ros::param::getCached(_key,_value);
}

template<typename T>
T Parameter<T>::operator() (){
    ros::param::getCached(_key, _value);
    return _value;
}

template<typename T>
void Parameter<T>::set(T& value) {
    _value = value;
    ros::param::set(_key, _value);
}

#endif // ROBOT_H
