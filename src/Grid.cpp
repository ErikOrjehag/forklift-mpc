#include "Grid.h"

Grid::Grid() : colorFull(200, 200, 200), colorStep(240, 240, 240)
{
    vertices.push_back(sf::Vertex(sf::Vector2f(halfSize, 0), colorFull));
    vertices.push_back(sf::Vertex(sf::Vector2f(-halfSize, 0), colorFull));
    vertices.push_back(sf::Vertex(sf::Vector2f(0, halfSize), colorFull));
    vertices.push_back(sf::Vertex(sf::Vector2f(0, -halfSize), colorFull));

    for (int i = 1; i < halfSize * resolution; i++) {
        sf::Color color = i % resolution == 0 ? colorFull : colorStep;
        vertices.push_back(sf::Vertex(sf::Vector2f(halfSize, i*step), color));
        vertices.push_back(sf::Vertex(sf::Vector2f(-halfSize, i*step), color));

        vertices.push_back(sf::Vertex(sf::Vector2f(halfSize, -i*step), color));
        vertices.push_back(sf::Vertex(sf::Vector2f(-halfSize, -i*step), color));


        vertices.push_back(sf::Vertex(sf::Vector2f(i*step, halfSize), color));
        vertices.push_back(sf::Vertex(sf::Vector2f(i*step, -halfSize), color));

        vertices.push_back(sf::Vertex(sf::Vector2f(-i*step, halfSize), color));
        vertices.push_back(sf::Vertex(sf::Vector2f(-i*step, -halfSize), color));
    }
}

void Grid::toggleActive()
{
    active = !active;
}

bool Grid::isActive()
{
    return active;
}

void Grid::snapPoint(Eigen::Vector2d& point)
{
    point[0] = std::round(point[0] / step) * step;
    point[1] = std::round(point[1] / step) * step;
}

void Grid::draw(sf::RenderWindow& window, TransformStack& ts)
{
    window.draw(&vertices[0], vertices.size(), sf::Lines, ts);
}
