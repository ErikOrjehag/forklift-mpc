#ifndef BEIZERCURVE_H
#define BEIZERCURVE_H

#include <vector>
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include "TransformStack.h"

class BeizerCurve
{
    public:
        BeizerCurve();
        virtual ~BeizerCurve();

        std::vector<Eigen::Vector2d> points;

        void clear();
        void add(Eigen::Vector2d point);
        Eigen::Vector2d getPoint(float t);
        std::vector<Eigen::Vector2d> getCurve();
        void draw(sf::RenderWindow& window, TransformStack& ts, sf::Color color = sf::Color::Blue);

    protected:

    private:
        int fac(int num);
        int n_take_k(int n, int k);
};

#endif // BEIZERCURVE_H
