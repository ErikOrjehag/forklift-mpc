#include "ForkliftModel.h"
#include <cppad/cppad.hpp>

template<typename T>
ForkliftModel<T>::ForkliftModel() : position(0, 0), size(1.5, 1)
{
    position.dot(size);
}

template<typename T>
ForkliftModel<T>::~ForkliftModel()
{

}

template<typename T>
void ForkliftModel<T>::update(double dt)
{
    if (steer == 0) {
        this->da = 0;
        this->dx = dt * speed * cos(heading);
        this->dy = dt * speed * sin(heading);
        position[0] += dx;
        position[1] += dy;
    } else {
        T r = size[0] / sin(steer);
        this->da = dt * speed / r;
        heading  += da;
        T h = r * cos(steer);
        this->dx =    h * sin(heading + da) - h * sin(heading);
        this->dy = - (h * cos(heading + da) - h * cos(heading));
        position[0] += dx;
        position[1] += dy;
    }
}

template class ForkliftModel<double>;
template class ForkliftModel<CppAD::AD<double>>;
