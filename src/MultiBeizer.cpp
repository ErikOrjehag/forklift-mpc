#include "MultiBeizer.h"

MultiBeizer::MultiBeizer()
{
    //ctor
}

void MultiBeizer::add(BeizerCurve& curve)
{
    curves.push_back(curve);
}

void MultiBeizer::inputPoint(Eigen::Vector2d point)
{
    if (isEditing) {
        BeizerCurve& last = curves[curves.size()-1];
        Eigen::Vector2d& p = last.points[last.points.size()-1];
        p[0] = point[0];
        p[1] = point[1];
    } else {
        isEditing = true;
        BeizerCurve beizer;
        BeizerCurve& last = curves[curves.size()-1];
        beizer.add(last.points[last.points.size()-1]);
        beizer.add(point);
        curves.push_back(beizer);
    }
}

void MultiBeizer::commitPoint()
{
    if (isEditing) {
        BeizerCurve& last = curves[curves.size()-1];
        Eigen::Vector2d p = last.points[last.points.size()-1];
        last.add(p);
    }
}

void MultiBeizer::commitBeizer()
{
    isEditing = false;
}

void MultiBeizer::abortBeizer()
{
    if (isEditing) {
        isEditing = false;
        curves.pop_back();
    }
}

void MultiBeizer::deletePoint()
{
    BeizerCurve& last = curves[curves.size()-1];
    last.points.pop_back();
    if (last.points.size() <= 1) {
        curves.pop_back();
    }
}

void MultiBeizer::clear()
{
    curves.clear();
}

std::vector<Eigen::Vector2d> MultiBeizer::path()
{
    std::vector<Eigen::Vector2d> path;
    std::vector<Eigen::Vector2d> curve;
    for (BeizerCurve& beizer : curves) {
        curve = beizer.getCurve();
        path.insert(path.end(), curve.begin(), curve.end());
    }
    return path;
}

void MultiBeizer::draw(sf::RenderWindow& window, TransformStack& ts)
{
    for (int i = 0; i < curves.size(); i++) {
        if (isEditing && (i == curves.size()-1)) {
            curves[i].draw(window, ts, sf::Color::Red);
        } else {
            curves[i].draw(window, ts);
        }
    }
}

