#include "TransformStack.h"

void TransformStack::translate(float x, float y) {
    transformCur.translate(x,y);
}

void TransformStack::rotate(float angle) {
    transformCur.rotate(angle);
}
void TransformStack::scale(float sx, float sy) {
    transformCur.scale(sx, sy);
}
void TransformStack::scale(float s)
{
    scale(s,s);
}

void TransformStack::push() {
    transformTot = transformTot.combine(transformCur);
    stack.push(transformCur);
    transformCur = sf::Transform();
}

void TransformStack::pop() {
    sf::Transform transform = stack.top();
    stack.pop();
    sf::Transform invTransform = transform.getInverse();
    transformTot = transformTot.combine(invTransform);
    transformCur = transform;
}

TransformStack::operator sf::Transform() {
    return transformTot * transformCur;
}

TransformStack::operator sf::RenderStates() {
    return static_cast<sf::RenderStates>( operator sf::Transform() );
}
