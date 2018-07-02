
#include <SFML/Graphics.hpp>
#include "Forklift.h"
#include "Frame.h"
#include <iostream>
#include "BeizerCurve.h"
#include "NavError.h"
#include "MPC.h"
#include "Utils.h"
#include <stdlib.h>

const int WINDOW_WIDTH = 960;
const int WINDOW_HEIGHT = 600;

int main()
{
    bool automatic = false;

    Forklift forklift;
    forklift.model.position[0] = 1.1;
    forklift.model.position[1] = 2.9;
    forklift.steer = 0.01;

    MPC mpc(forklift.model);

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

    BeizerCurve beizer5;
    beizer5.add(Eigen::Vector2d(0, -5));
    beizer5.add(Eigen::Vector2d(-2, -5));

    BeizerCurve beizer6;
    beizer6.add(Eigen::Vector2d(-2, -5));
    beizer6.add(Eigen::Vector2d(-3, -5));
    beizer6.add(Eigen::Vector2d(-3, -4));

    BeizerCurve beizer7;
    beizer7.add(Eigen::Vector2d(-3, -4));
    beizer7.add(Eigen::Vector2d(-3, -2));

    BeizerCurve beizer8;
    beizer8.add(Eigen::Vector2d(-3, -2));
    beizer8.add(Eigen::Vector2d(-1, -2));
    beizer8.add(Eigen::Vector2d(-1, 0));

    BeizerCurve beizer9;
    beizer9.add(Eigen::Vector2d(-1, 0));
    beizer9.add(Eigen::Vector2d(-1, 3));
    beizer9.add(Eigen::Vector2d(1, 3));

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
    curve = beizer5.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer6.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer7.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer8.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer9.getCurve();
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
            if (event.type == sf::Event::Closed) {
                window.close();

            } else if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f point = static_cast<sf::Transform>(ts).getInverse().transformPoint(event.mouseButton.x, event.mouseButton.y);
                    targetFrame.pos = Eigen::Vector2d(point.x, point.y);
                    //beizer1.add(targetFrame.pos);
                }

            } else if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::A) {
                    automatic = !automatic;
                } else if(event.key.code == sf::Keyboard::D) {
                    double max_lin = 0.2; // meters
                    double max_heading = 30; // degrees
                    Eigen::Vector2d d_pos(max_lin*2.0*((double)rand()/RAND_MAX-0.5), max_lin*2.0*((double)rand()/RAND_MAX-0.5));
                    float d_heading = max_heading*M_PI/180*2.0*((double)rand()/RAND_MAX-0.5);
                    forklift.model.position += d_pos;
                    forklift.model.heading += d_heading;
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

        /*Frame f;
        f.pos = forklift.model.position;
        f.angle = forklift.model.heading;
        NavError navError = NavError::calcNavError(curve, f);*/

        segment = transformPointsIntoFrame(path, forklift.model.position, forklift.model.heading);
        segment = rollingWindowPath(segment, 1.0, 1.0);

        const int ORDER = 3;
        Eigen::VectorXd K = polyfit(segment, ORDER);
        std::vector<Eigen::Vector2d> fitted;
        for (int i = -50; i < 50; ++i) {
            const double dx = 0.1 * i;
            const double dy = K[3] * dx * dx * dx + K[2] * dx * dx + K[1] * dx + K[0];
            fitted.push_back(Eigen::Vector2d(dx, dy));
        }

        mpc.solve(K, forklift.model.MAX_SPEED * 0.5);

        if (automatic) {
            forklift.speed += mpc.speed_ac * dt;
            forklift.steer += mpc.steer_ac * dt;
        } else {
            forklift.control(dt);
        }

        forklift.update(dt);

        window.clear(sf::Color::White);

        globalFrame.draw(window, ts);
        targetFrame.draw(window, ts);

        beizer1.draw(window, ts);
        beizer2.draw(window, ts);
        beizer3.draw(window, ts);
        beizer4.draw(window, ts);
        beizer5.draw(window, ts);
        beizer6.draw(window, ts);
        beizer7.draw(window, ts);
        beizer8.draw(window, ts);
        beizer9.draw(window, ts);

        //navError.draw(window, ts);

        forklift.draw(window, ts);

        ts.push();
        ts.translate(forklift.model.position[0], forklift.model.position[1]);
        ts.rotate(forklift.model.heading * 180 / M_PI);

        drawPath(window, ts, fitted, sf::Color::Cyan);
        drawPath(window, ts, segment, sf::Color::Black);
        mpc.draw(window, ts);

        ts.pop();

        window.display();
    }

    return 0;
}
