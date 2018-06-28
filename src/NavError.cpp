#include "NavError.h"

#include <iostream>

NavError::NavError()
{

}

NavError::NavError(Frame frame, float linear, float angle)
    : frame(frame), linear(linear), angle(angle)
{
    //ctor
}

NavError::~NavError()
{
    //dtor
}

NavError NavError::calcNavError(const std::vector<Eigen::Vector2d>& curve, Frame f, bool goForward)
{
    if (curve.size() < 2) {
        return NavError();
    }

    Eigen::Vector2d posDesired = curve[0];
    Eigen::Vector2d dirDesired = curve[1] - curve[0];

    for (int i = 0; i < curve.size(); i++) {

        Eigen::Vector2d segment;

        if (i != curve.size() - 1) {
            segment = curve[i+1] - curve[i];
        } else {
            segment = curve[i] - curve[i-1];
        }

        if ((f.pos - curve[i]).squaredNorm() < (f.pos - posDesired).squaredNorm()) {
            posDesired = curve[i];
            dirDesired = segment;
        }

        if (i != curve.size() - 1) {

            float projection = (f.pos - curve[i]).dot(segment) / segment.squaredNorm();

            if (projection > 0 && projection < 1) {

                Eigen::Vector2d p = curve[i] + projection * segment;

                if ((f.pos - p).squaredNorm() < (f.pos - posDesired).squaredNorm()) {
                    posDesired = p;
                    dirDesired = segment;
                }
            }
        }
    }

    if (!goForward) dirDesired *= -1;

    Eigen::Vector2d errVec = (f.pos - posDesired);
    float determinant = errVec[0] * dirDesired[1] - errVec[1] * dirDesired[0];

    float angleDesired = std::atan2(dirDesired[1], dirDesired[0]);
    float linearError = (f.pos - posDesired).norm() * (determinant > 0 ? -1 : 1);
    float angleError = std::fmod(f.angle - angleDesired + M_PI, M_PI*2) - M_PI;

    return NavError(Frame(posDesired, angleDesired), linearError, angleError);
}

void NavError::draw(sf::RenderWindow& window, TransformStack& ts)
{
    sf::Vertex linearErrorLine[] =
    {
        sf::Vertex(sf::Vector2f(0, 0), sf::Color::Magenta),
        sf::Vertex(sf::Vector2f(0, linear), sf::Color::Magenta)
    };

    sf::Vertex angleErrorLine[] =
    {
        sf::Vertex(sf::Vector2f(0, 0), sf::Color::Magenta),
        sf::Vertex(sf::Vector2f(0.2, 0), sf::Color::Magenta)
    };

    frame.draw(window, ts);
    ts.push();
    ts.translate(frame.pos[0], frame.pos[1]);
    ts.rotate(frame.angle * 180 / M_PI);

    window.draw(linearErrorLine, 2, sf::Lines, ts);

    ts.rotate(angle * 180 / M_PI);
    window.draw(angleErrorLine, 2, sf::Lines, ts);
    //std::cout << angle << std::endl;

    ts.pop();
}
