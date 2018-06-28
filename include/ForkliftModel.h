#ifndef FORKLIFTSTATE_H
#define FORKLIFTSTATE_H

#include <Eigen/Dense>

template<typename T>
class ForkliftModel
{
    public:
        ForkliftModel();
        virtual ~ForkliftModel();

        Eigen::Matrix<T, 2, 1> size;
        Eigen::Matrix<T, 2, 1> position;
        T heading = 0;
        T steer = 0;
        T speed = 0;

        const double MAX_STEER = M_PI / 2;
        const double MAX_STEER_DELTA = 50 * M_PI / 180;
        const double MAX_SPEED = 1.5;
        const double MAX_SPEED_DELTA = 0.8;

        void update(double dt);

    protected:

    private:
};

#endif // FORKLIFTSTATE_H
