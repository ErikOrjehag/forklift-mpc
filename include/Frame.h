#ifndef FRAME_H
#define FRAME_H

#include <SFML/Graphics.hpp>
#include "Eigen/Dense"
#include "TransformStack.h"

class Frame
{
    public:
        Frame();
        Frame(Eigen::Vector2d pos, float angle);
        virtual ~Frame();

        float angle = 0;
        Eigen::Vector2d pos;

        void draw(sf::RenderWindow& window, TransformStack& ts);

    protected:

    private:
};

#endif // FRAME_H
