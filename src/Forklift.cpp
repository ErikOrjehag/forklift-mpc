#include "Forklift.h"
#include <iostream>

Forklift::Forklift() {}

Forklift::~Forklift() {}

void Forklift::draw(sf::RenderWindow& window, TransformStack& ts)
{
    sf::CircleShape  origo;
    origo.setRadius(0.05);
    origo.setPosition(sf::Vector2f(-0.05, -0.05));
    origo.setOutlineColor(sf::Color::Black);
    origo.setOutlineThickness(0.01);

    sf::RectangleShape driveWheel;
    driveWheel.setSize(sf::Vector2f(0.3, 0.1));
    driveWheel.setPosition(sf::Vector2f(-0.15, -0.05));
    driveWheel.setOutlineColor(sf::Color::Black);
    driveWheel.setOutlineThickness(0.02);

    sf::RectangleShape smallWheel;
    smallWheel.setSize(sf::Vector2f(0.2, 0.05));
    smallWheel.setPosition(sf::Vector2f(-0.1, -0.025));
    smallWheel.setOutlineColor(sf::Color::Black);
    smallWheel.setOutlineThickness(0.02);

    sf::RectangleShape debugLine;
    debugLine.setSize(sf::Vector2f(0.01, 100));
    debugLine.setPosition(sf::Vector2f(-0.005, -50));
    debugLine.setFillColor(sf::Color(200, 200, 200));

    sf::RectangleShape bodyBack;
    bodyBack.setSize(sf::Vector2f(0.05, model.size[1]));
    bodyBack.setPosition(sf::Vector2f(-0.025, model.size[1]/-2));
    bodyBack.setFillColor(sf::Color::Black);

    sf::RectangleShape bodySide;
    bodySide.setSize(sf::Vector2f(model.size[0] + 0.15, 0.05));
    bodySide.setPosition(sf::Vector2f(-model.size[0] - 0.15, -0.025));
    bodySide.setFillColor(sf::Color::Black);

    sf::RectangleShape fork;
    fork.setSize(sf::Vector2f(1.2, 0.08));
    fork.setPosition(sf::Vector2f(-1.2 + 0.5, -0.04));
    fork.setFillColor(sf::Color::Black);

    ts.push();
    ts.translate(model.position[0], model.position[1]);
    ts.rotate(model.heading * 180 / M_PI);

    window.draw(debugLine, ts);
    window.draw(origo, ts);

    ts.push();
    ts.translate(model.size[0] + 0.3, 0);
    window.draw(bodyBack, ts);
    ts.translate(0, model.size[1] / -2);
    window.draw(bodySide, ts);
    ts.translate(0, model.size[1]);
    window.draw(bodySide, ts);
    ts.pop();

    ts.push();
    ts.translate(0, 0.08 + 0.08);
    window.draw(fork, ts);
    ts.translate(0, -0.16 - 0.16);
    window.draw(fork, ts);
    ts.pop();

    ts.push();
    ts.translate(0, model.size[1] / -2);
    window.draw(smallWheel, ts);
    ts.translate(0, model.size[1]);
    window.draw(smallWheel, ts);
    ts.pop();

    ts.push();
    ts.translate(model.size[0], 0);
    ts.rotate(model.steer * 180 / M_PI);
    window.draw(debugLine, ts);
    window.draw(driveWheel, ts);
    ts.pop();

    frame.draw(window, ts);

    ts.pop();
}

void Forklift::control(double dt)
{
    switch (driveCmd) {
    case FORWARD:
        speed += model.MAX_SPEED_DELTA * dt;
        break;
    case BACKWARD:
        speed -= model.MAX_SPEED_DELTA * dt;
        break;
    default:
        speed *= 0.97;
    }

    if (steerCmd == RIGHT) steer -= model.MAX_STEER_DELTA * dt;
    else if (steerCmd == LEFT) steer += model.MAX_STEER_DELTA * dt;

    speed = std::max(-model.MAX_SPEED, std::min(model.MAX_SPEED, speed));
    steer = std::max(-model.MAX_STEER, std::min(model.MAX_STEER, steer));
}

void Forklift::update(double dt)
{
    model.speed = speed;
    model.steer = steer;
    model.update(dt);
}
