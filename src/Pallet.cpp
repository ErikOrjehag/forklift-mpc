#include "Pallet.h"
#include <iostream>
#include "Frame.h"

Pallet::Pallet() : position(1, 1)
{
    if (!texture.loadFromFile("pallet.png")) {
        std::cout << "could not load texture!" << std::endl;
    }
    sprite.setTexture(texture);
    sprite.setScale(0.0025, 0.0025);
    sprite.setPosition(-0.68, -0.49);
}

Pallet::~Pallet()
{
    //dtor
}

void Pallet::draw(sf::RenderWindow& window, TransformStack& ts)
{
    ts.push();
    ts.translate(position[0], position[1]);
    ts.rotate(heading * 180 / M_PI);
    window.draw(sprite, ts);
    Frame frame;
    frame.draw(window, ts);
    ts.pop();
}
