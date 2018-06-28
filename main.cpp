
#include <SFML/Graphics.hpp>
#include "Forklift.h"
#include "Frame.h"
#include <iostream>
#include "BeizerCurve.h"
#include "NavError.h"
#include "MPC.h"
#include "Utils.h"

const int WINDOW_WIDTH = 960;
const int WINDOW_HEIGHT = 600;

int main()
{
    Forklift forklift;

    MPC mpc;

    Frame globalFrame;
    Frame targetFrame;

    BeizerCurve beizer1;
    beizer1.add(Eigen::Vector2d(1, 3));
    beizer1.add(Eigen::Vector2d(3, 3));
    beizer1.add(Eigen::Vector2d(3, 1));

    BeizerCurve beizer2;
    beizer2.add(Eigen::Vector2d(3, 1));
    beizer2.add(Eigen::Vector2d(3, 0));
    beizer2.add(Eigen::Vector2d(2, 0));
    beizer2.add(Eigen::Vector2d(2, -1));

    BeizerCurve beizer3;
    beizer3.add(Eigen::Vector2d(2, -1));
    beizer3.add(Eigen::Vector2d(2, -3));

    BeizerCurve beizer4;
    beizer4.add(Eigen::Vector2d(2, -3));
    beizer4.add(Eigen::Vector2d(2, -5));
    beizer4.add(Eigen::Vector2d(0, -5));

    std::vector<Eigen::Vector2d> path;
    std::vector<Eigen::Vector2d> curve;
    std::vector<Eigen::Vector2d> segment;

    curve = beizer1.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer2.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer3.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer4.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "MPC", sf::Style::Default, settings);

    TransformStack ts;
    ts.rotate(-90);
    ts.scale(1, -1);
    ts.translate(WINDOW_HEIGHT/-2, WINDOW_WIDTH/-2);
    ts.scale(70, 70);

    sf::Clock clock;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f point = static_cast<sf::Transform>(ts).getInverse().transformPoint(event.mouseButton.x, event.mouseButton.y);
                    targetFrame.pos = Eigen::Vector2d(point.x, point.y);
                    //beizer1.add(targetFrame.pos);
                }
            }
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) forklift.driveCmd = FORWARD;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) forklift.driveCmd = BACKWARD;
        else forklift.driveCmd = STOP;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) forklift.steerCmd = RIGHT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) forklift.steerCmd = LEFT;
        else forklift.steerCmd = NONE;

        float dt = clock.restart().asSeconds();
        forklift.update(dt);

        /*Frame f;
        f.pos = forklift.model.position;
        f.angle = forklift.model.heading;
        NavError navError = NavError::calcNavError(curve, f);*/

        segment = transformPointsIntoFrame(path, forklift.model.position, forklift.model.heading);
        segment = rollingWindowPath(segment, 0.2, 2.0);

        const int ORDER = 3;
        Eigen::VectorXd K = polyfit(segment, ORDER);

        std::vector<Eigen::Vector2d> fitted;
        for (int i = -50; i < 50; ++i) {
            const double dx = 0.1 * i;
            const double dy = K[3] * dx * dx * dx + K[2] * dx * dx + K[1] * dx + K[0];
            fitted.push_back(Eigen::Vector2d(dx, dy));
        }

        mpc.solve(forklift.model, K);

        window.clear(sf::Color::White);

        globalFrame.draw(window, ts);
        targetFrame.draw(window, ts);

        beizer1.draw(window, ts);
        beizer2.draw(window, ts);
        beizer3.draw(window, ts);
        beizer4.draw(window, ts);

        //navError.draw(window, ts);

        forklift.draw(window, ts);

        //mpc.draw(window, ts);

        drawPath(window, ts, fitted, sf::Color::Cyan);
        drawPath(window, ts, segment, sf::Color::Magenta);

        window.display();
    }

    return 0;
}
