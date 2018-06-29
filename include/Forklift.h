#ifndef FORKLIFT_H
#define FORKLIFT_H

#include <math.h>
#include <SFML/Graphics.hpp>
#include "Eigen/Dense"
#include "TransformStack.h"
#include "Frame.h"
#include "ForkliftModel.h"

enum DriveCommand
{
    STOP, FORWARD, BACKWARD
};

enum SteerCommand
{
    NONE, RIGHT, LEFT
};

class Forklift
{
    public:
        Forklift();
        virtual ~Forklift();

        ForkliftModel<double> model;
        Frame frame;

        double steer = 0;
        double speed = 0;

        DriveCommand driveCmd = STOP;
        SteerCommand steerCmd = NONE;

        void draw(sf::RenderWindow& window, TransformStack& ts);
        void control(double dt);
        void update(double dt);

    protected:

    private:
};

#endif // FORKLIFT_H
