#include "BeizerCurve.h"
#include <iostream>

BeizerCurve::BeizerCurve()
{
    //ctor
}

BeizerCurve::~BeizerCurve()
{
    //dtor
}

void BeizerCurve::clear()
{
    points.clear();
}

void BeizerCurve::add(Eigen::Vector2d point)
{
    points.push_back(point);
}

int BeizerCurve::fac(int n)
{
    int f = 1;
    while (n-- > 0) f *= (n+1);
    return f;
}

int BeizerCurve::n_take_k(int n, int k) {
    assert(n >= k);
    return (float)fac(n) / (fac(k) * fac(n-k));
}

Eigen::Vector2d BeizerCurve::getPoint(float t)
{
    int n = points.size();
    Eigen::Vector2d p(0, 0);
    for (int i = 0; i < n; i++) {
        p += n_take_k(n-1, i) * std::pow(1.f-t, n-1-i) * std::pow(t, i) * points[i];
    }
    return p;
}

std::vector<Eigen::Vector2d> BeizerCurve::getCurve()
{
    std::vector<Eigen::Vector2d> vertices;
    int resolution = 30;

    for (int r = 0; r <= resolution; r++) {
        Eigen::Vector2d p = getPoint((1.0/resolution) * r);
        vertices.push_back(Eigen::Vector2d(p[0], p[1]));
    }

    return vertices;
}

void BeizerCurve::draw(sf::RenderWindow& window, TransformStack& ts)
{

    std::vector<Eigen::Vector2d> curve = getCurve();
    std::vector<sf::Vertex> vertices;
    for (Eigen::Vector2d& p : curve) vertices.push_back(sf::Vertex(sf::Vector2f(p[0], p[1]), sf::Color::Blue));
    window.draw(&vertices[0], vertices.size(), sf::LineStrip, ts);

    sf::CircleShape  circle;
    circle.setRadius(0.05);
    circle.setOutlineColor(sf::Color::Blue);
    circle.setOutlineThickness(0.01);

    for (Eigen::Vector2d& p : points) {
        circle.setPosition(sf::Vector2f(p[0]-0.025, p[1]-0.025));
        window.draw(circle, ts);
    }
}
