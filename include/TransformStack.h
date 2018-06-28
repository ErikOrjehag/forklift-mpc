#ifndef TRANSFORMSTACK_H
#define TRANSFORMSTACK_H

#include <SFML/Graphics.hpp>
#include <stack>

class TransformStack {
    private:
        sf::Transform transformCur;
        sf::Transform transformTot;
        std::stack<sf::Transform> stack;
    public:
        void translate(float x, float y);
        void rotate(float angle);
        void scale(float sx, float sy);
        void scale(float s);
        void push();
        void pop();
        operator sf::Transform();
        operator sf::RenderStates();
};

#endif // TRANSFORMSTACK_H
