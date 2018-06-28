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
        position[0] += dt * speed * cos(heading);
        position[1] += dt * speed * sin(heading);
    } else {
        T r = size[0] / sin(steer);
        T da = dt * speed / r;
        heading  += da;
        T h = r * cos(steer);
        position[0] += h * sin(heading + da) - h * sin(heading);
        position[1] -= h * cos(heading + da) - h * cos(heading);
    }
}

template class ForkliftModel<double>;
template class ForkliftModel<CppAD::AD<double>>;
