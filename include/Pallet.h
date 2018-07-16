#ifndef PALLET_H
#define PALLET_H

#include <SFML/Graphics.hpp>
#include "TransformStack.h"
#include <Eigen/Dense>

class Pallet
{
    public:
        Pallet();
        virtual ~Pallet();

        void draw(sf::RenderWindow& window, TransformStack& ts);

        Eigen::Vector2d position;
        double heading = 20 * M_PI / 180;

    protected:

    private:
        sf::Texture texture;
        sf::Sprite sprite;
};

#endif // PALLET_H
