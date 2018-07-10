
#include <SFML/Graphics.hpp>
#include "Forklift.h"
#include "Frame.h"
#include <iostream>
#include "BeizerCurve.h"
#include "NavError.h"
#include "MPC.h"
#include "Utils.h"
#include <stdlib.h>
#include "MultiBeizer.h"
#include "Grid.h"

const int WINDOW_WIDTH = 960;
const int WINDOW_HEIGHT = 600;
const int PX_PER_METER = 70;

int main()
{
    bool automatic = false;
    int autoDirection = 1; // forward: 1, reverse: -1

    Forklift forklift;
    forklift.steer = 45 * M_PI / 180;
    forklift.model.position[0] = 1.5;
    forklift.model.position[1] = 2.9;
    forklift.model.heading = 20 * M_PI / 180;

    Grid grid;

    MPC mpc(forklift.model);

    Frame globalFrame;

    MultiBeizer multiBeizer;

    BeizerCurve beizer1;
    beizer1.add(Eigen::Vector2d(1, 3));
    beizer1.add(Eigen::Vector2d(3, 3));
    beizer1.add(Eigen::Vector2d(3, 1));
    multiBeizer.add(beizer1);

    BeizerCurve beizer2;
    beizer2.add(Eigen::Vector2d(3, 1));
    beizer2.add(Eigen::Vector2d(3, 0));
    beizer2.add(Eigen::Vector2d(2, 0));
    beizer2.add(Eigen::Vector2d(2, -1));
    multiBeizer.add(beizer2);

    BeizerCurve beizer3;
    beizer3.add(Eigen::Vector2d(2, -1));
    beizer3.add(Eigen::Vector2d(2, -3));
    multiBeizer.add(beizer3);

    BeizerCurve beizer4;
    beizer4.add(Eigen::Vector2d(2, -3));
    beizer4.add(Eigen::Vector2d(2, -5));
    beizer4.add(Eigen::Vector2d(0, -5));
    multiBeizer.add(beizer4);

    BeizerCurve beizer5;
    beizer5.add(Eigen::Vector2d(0, -5));
    beizer5.add(Eigen::Vector2d(-2, -5));
    multiBeizer.add(beizer5);

    BeizerCurve beizer6;
    beizer6.add(Eigen::Vector2d(-2, -5));
    beizer6.add(Eigen::Vector2d(-3, -5));
    beizer6.add(Eigen::Vector2d(-3, -4));
    multiBeizer.add(beizer6);

    BeizerCurve beizer7;
    beizer7.add(Eigen::Vector2d(-3, -4));
    beizer7.add(Eigen::Vector2d(-3, -2));
    multiBeizer.add(beizer7);

    BeizerCurve beizer8;
    beizer8.add(Eigen::Vector2d(-3, -2));
    beizer8.add(Eigen::Vector2d(-3, -1));
    beizer8.add(Eigen::Vector2d(-2, -1));
    multiBeizer.add(beizer8);

    BeizerCurve beizer9;
    beizer9.add(Eigen::Vector2d(-2, -1));
    beizer9.add(Eigen::Vector2d(-1, -1));
    beizer9.add(Eigen::Vector2d(-1, 0));
    multiBeizer.add(beizer9);

    BeizerCurve beizer10;
    beizer10.add(Eigen::Vector2d(-1, 0));
    beizer10.add(Eigen::Vector2d(-1, 1));
    beizer10.add(Eigen::Vector2d(-2, 1));
    beizer10.add(Eigen::Vector2d(-2, 2));
    multiBeizer.add(beizer10);

    BeizerCurve beizer11;
    beizer11.add(Eigen::Vector2d(-2, 2));
    beizer11.add(Eigen::Vector2d(-2, 3));
    beizer11.add(Eigen::Vector2d(-1, 3));
    beizer11.add(Eigen::Vector2d(0, 3));
    multiBeizer.add(beizer11);

    BeizerCurve beizer12;
    beizer12.add(Eigen::Vector2d(0, 3));
    beizer12.add(Eigen::Vector2d(1, 3));
    multiBeizer.add(beizer12);

    std::vector<Eigen::Vector2d> path = multiBeizer.path();
    std::vector<Eigen::Vector2d> segment;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "MPC", sf::Style::Default, settings);

    TransformStack ts;
    ts.rotate(-90);
    ts.scale(1, -1);
    ts.translate(WINDOW_HEIGHT/-2, WINDOW_WIDTH/-2);
    ts.scale(PX_PER_METER, PX_PER_METER);

    ts.translate(0, 1);

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
                    sf::Vector2f p = static_cast<sf::Transform>(ts).getInverse().transformPoint(event.mouseButton.x, event.mouseButton.y);
                    Eigen::Vector2d point(p.x, p.y);
                    if (grid.isActive()) {
                        grid.snapPoint(point);
                    }
                    multiBeizer.inputPoint(point);
                }

            } else if (event.type == sf::Event::KeyReleased) {

                // Toggle to/from automatic mode.
                if (event.key.code == sf::Keyboard::A) {
                    automatic = !automatic;

                // Toggle forward/reverse automatic direction.
                } else if (event.key.code == sf::Keyboard::S) {
                    autoDirection *= -1;

                // Make forklift longer.
                } else if (event.key.code == sf::Keyboard::O) {
                    forklift.model.size[0] += 0.1;

                // Make forklift shorter.
                } else if (event.key.code == sf::Keyboard::L) {
                    forklift.model.size[0] -= 0.1;

                // Disturb truck location to simulate sensor noise.
                } else if(event.key.code == sf::Keyboard::D) {
                    double max_lin = 0.2; // meters
                    double max_heading = 30; // degrees
                    Eigen::Vector2d d_pos(max_lin*2.0*((double)rand()/RAND_MAX-0.5), max_lin*2.0*((double)rand()/RAND_MAX-0.5));
                    float d_heading = max_heading*M_PI/180*2.0*((double)rand()/RAND_MAX-0.5);
                    forklift.model.position += d_pos;
                    forklift.model.heading += d_heading;

                } else if (event.key.code == sf::Keyboard::Q) {
                    multiBeizer.abortBeizer();

                } else if (event.key.code == sf::Keyboard::W) {
                    multiBeizer.deletePoint();

                } else if (event.key.code == sf::Keyboard::E) {
                    multiBeizer.commitPoint();

                } else if (event.key.code == sf::Keyboard::R) {
                    multiBeizer.commitBeizer();
                    path = multiBeizer.path();

                } else if (event.key.code == sf::Keyboard::G) {
                    grid.toggleActive();
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

        if (automatic) {
            mpc.solve(K, forklift.model.MAX_SPEED * autoDirection);
            forklift.speed += mpc.speed_ac * dt;
            forklift.steer += mpc.steer_ac * dt;
        } else {
            forklift.control(dt);
        }

        forklift.update(dt);

        // Draw to screen

        window.clear(sf::Color::White);
        if (grid.isActive()) {
            grid.draw(window, ts);
        }
        globalFrame.draw(window, ts);
        multiBeizer.draw(window, ts);
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
