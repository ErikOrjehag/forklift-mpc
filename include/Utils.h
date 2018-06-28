#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <Eigen/Dense>
#include <vector>
#include <SFML/Graphics.hpp>
#include "TransformStack.h"

std::vector<Eigen::Vector2d> transformPointsIntoFrame(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& position, double heading);

void drawPath(sf::RenderWindow& window, TransformStack& ts, std::vector<Eigen::Vector2d> path, sf::Color color);

Eigen::VectorXd polyfit(const std::vector<Eigen::Vector2d>& points, int order);

std::vector<Eigen::Vector2d> rollingWindowPath(const std::vector<Eigen::Vector2d>& path, double windowBackward, double windowForward);

#endif // UTILS_H_INCLUDED
