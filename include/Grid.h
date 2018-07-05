#ifndef GRID_H
#define GRID_H

#include <SFML/Graphics.hpp>
#include "TransformStack.h"
#include <Eigen/Dense>

class Grid
{
    public:
        Grid();

        void toggleActive();
        bool isActive();
        void snapPoint(Eigen::Vector2d& point);
        void draw(sf::RenderWindow& window, TransformStack& ts);

    protected:

    private:
        int resolution = 2; // Steps/m
        int halfSize = 10; // m
        double step = 1.0 / resolution;
        bool active = false;

        std::vector<sf::Vertex> vertices;

        sf::Color colorFull;
        sf::Color colorStep;
};

#endif // GRID_H
