#include "Frame.h"

Frame::Frame()
{
    pos << 0, 0;
}

Frame::Frame(Eigen::Vector2d pos, float angle)
    : pos(pos), angle(angle)
{

}

Frame::~Frame()
{
    //dtor
}

void Frame::draw(sf::RenderWindow& window, TransformStack& ts)
{
    sf::Vertex xAxis[] =
    {
        sf::Vertex(sf::Vector2f(0, 0), sf::Color::Red),
        sf::Vertex(sf::Vector2f(1, 0), sf::Color::Red),
        sf::Vertex(sf::Vector2f(0.9, 0.1), sf::Color::Red),
        sf::Vertex(sf::Vector2f(1, 0), sf::Color::Red),
        sf::Vertex(sf::Vector2f(0.9, -0.1), sf::Color::Red),
        sf::Vertex(sf::Vector2f(1, 0), sf::Color::Red)
    };

    sf::Vertex yAxis[] =
    {
        sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
        sf::Vertex(sf::Vector2f(0, 1), sf::Color::Green),
        sf::Vertex(sf::Vector2f(0.1, 0.9), sf::Color::Green),
        sf::Vertex(sf::Vector2f(0, 1), sf::Color::Green),
        sf::Vertex(sf::Vector2f(-0.1, 0.9), sf::Color::Green),
        sf::Vertex(sf::Vector2f(0, 1), sf::Color::Green)
    };

    ts.push();
    ts.translate(pos[0], pos[1]);
    ts.rotate(angle * 180 / M_PI);
    window.draw(xAxis, 6, sf::Lines, ts);
    window.draw(yAxis, 6, sf::Lines, ts);
    ts.pop();
}
