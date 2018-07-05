#ifndef MULTIBEIZER_H
#define MULTIBEIZER_H

#include "BeizerCurve.h"
#include <vector>
#include "TransformStack.h"
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

class MultiBeizer
{
    public:
        MultiBeizer();

        void add(BeizerCurve& curve);
        void inputPoint(Eigen::Vector2d point);
        void commitPoint();
        void commitBeizer();
        void abortBeizer();
        void deletePoint();
        std::vector<Eigen::Vector2d> path();
        void draw(sf::RenderWindow& window, TransformStack& ts);

    protected:

    private:
        std::vector<BeizerCurve> curves;
        bool isEditing = false;
};

#endif // MULTIBEIZER_H
