#include "ForkliftModel.h"
#include <cppad/cppad.hpp>
#include <cmath>

template<typename T>
ForkliftModel<T>::ForkliftModel()
: position(0, 0), size(1.5, 1)
{
}

template<typename T>
void ForkliftModel<T>::update(double dt, T (*SIN)(T), T (*COS)(T))
{
    if (steer == 0) {
        position[0] += dt * speed * COS(heading);
        position[1] += dt * speed * SIN(heading);
    } else {
        T r = size[0] / SIN(steer);
        T da = dt * speed / r;
        heading  += da;
        T h = r * COS(steer);
        position[0] +=    h * SIN(heading + da) - h * SIN(heading);
        position[1] += - (h * COS(heading + da) - h * COS(heading));
    }
}

template<typename T>
void ForkliftModel<T>::updateApproximate(double dt, T (*SIN)(T), T (*COS)(T))
{
    double magic = 0.8;
    position[0] += dt * speed * COS(heading);
    position[1] += dt * speed * SIN(heading);
    heading += dt * steer * speed / (size[0] * magic);
}

template class ForkliftModel<double>;
template class ForkliftModel<CppAD::AD<double>>;
