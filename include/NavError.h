#ifndef NAVERROR_H
#define NAVERROR_H

#include <Eigen/Dense>
#include <vector>
#include "Frame.h"

/*

    DEPRICATED

*/

class NavError
{
    public:
        NavError();
        NavError(Frame frame, float linear, float angle);
        virtual ~NavError();

        static NavError calcNavError(const std::vector<Eigen::Vector2d>&, Frame frame, bool goForward = true);
        void draw(sf::RenderWindow& window, TransformStack& ts);

        float linear = 0;
        float angle = 0;

    protected:

    private:
        Frame frame;
};

#endif // NAVERROR_H
