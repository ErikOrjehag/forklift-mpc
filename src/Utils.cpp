#include "Utils.h"
#include <algorithm>
#include <iostream>


std::vector<Eigen::Vector2d> transformPointsIntoFrame(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& position, double heading)
{
    std::vector<Eigen::Vector2d> result;
    for (int i = 0; i < points.size(); i++) {
        double dx = points[i][0] - position[0];
        double dy = points[i][1] - position[1];
        result.push_back(Eigen::Vector2d(
            dx * cos(-heading) - dy * sin(-heading),
            dy * cos(-heading) + dx * sin(-heading)
        ));
    }
    return result;
}

void drawPath(sf::RenderWindow& window, TransformStack& ts, std::vector<Eigen::Vector2d> path, sf::Color color)
{
    std::vector<sf::Vertex> vertices;
    for (Eigen::Vector2d& p : path) vertices.push_back(sf::Vertex(sf::Vector2f(p[0], p[1]), color));
    window.draw(&vertices[0], vertices.size(), sf::LineStrip, ts);
}

Eigen::VectorXd polyfit(const std::vector<Eigen::Vector2d>& points, int order)
{
  // Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  assert(order >= 1 && order <= points.size() - 1);

  Eigen::VectorXd xvals(points.size());
  Eigen::VectorXd yvals(points.size());
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < points.size(); ++i) {
    A(i, 0) = 1.0;
    xvals[i] = points[i][0];
    yvals[i] = points[i][1];
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


std::vector<Eigen::Vector2d> rollingWindowPath(const std::vector<Eigen::Vector2d>& path, double windowBackward, double windowForward)
{
    // Assumes the path is in the frame of the vehicle (origo).
    std::vector<Eigen::Vector2d> result;

    if (path.size() < 2) {
        return result;
    }

    double closestSqrd = path[0].squaredNorm();
    int closestIndex = 0;

    for (int i = 0; i < path.size(); i++) {
        double distSqrd = path[i].squaredNorm();
        if (distSqrd < closestSqrd) {
            closestSqrd = distSqrd;
            closestIndex = i;
        }
    }

    int bi;
    for (bi = closestIndex-1; (bi >= 0) && (windowBackward > 0); bi--) {
        result.push_back(path[bi]);
        int neighbour = std::min<int>(bi+1, path.size()-1);
        windowBackward -= (path[bi] - path[neighbour]).norm();
    }

    std::reverse(result.begin(), result.end());

    windowForward += windowBackward;

    for (int i = closestIndex; (i < path.size()) && (windowForward > 0); i++) {
        result.push_back(path[i]);
        int neighbour = std::max<int>(i-1, 0);
        windowForward -= (path[i] - path[neighbour]).norm();
    }

    windowBackward += windowForward;

    for (/**/; (bi >= 0) && (windowBackward > 0); bi--) {
        result.insert(result.begin(), path[bi]);
        int neighbour = std::min<int>(bi+1, path.size()-1);
        windowBackward -= (path[bi] - path[neighbour]).norm();
    }

    return result;
}


Eigen::Vector2d angleToDir(double angle)
{
    return Eigen::Vector2d(cos(angle), sin(angle));
}

MultiBeizer pathFromTo(Eigen::Vector2d a, Eigen::Vector2d aDir, Eigen::Vector2d b, Eigen::Vector2d bDir)
{
    MultiBeizer result;
    BeizerCurve curve;
    BeizerCurve straight;
    curve.add(a - aDir * 0.5);
    curve.add(a + aDir * 1.5);
    curve.add(b - bDir * 2.5);
    curve.add(b - bDir * 1.5);
    straight.add(b - bDir * 1.5);
    straight.add(b);
    result.add(curve);
    result.add(straight);
    return result;
}
