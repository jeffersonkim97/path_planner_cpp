#include "obstacles.h"

Obstacles::Obstacles(){

}

void Obstacles::addObstacle(Vector2f firstPoint, Vector2f secondPoint){
    Vector2f tmp;
    if (firstPoint.x() > secondPoint.x() && firstPoint.y() > secondPoint.y()) {
        tmp = firstPoint;
        firstPoint = secondPoint;
        secondPoint = tmp;
    } else if (firstPoint.x() < secondPoint.x() && firstPoint.y() > secondPoint.y()) {
        int height = firstPoint.y() - secondPoint.y();
        firstPoint.y() -= height;
        secondPoint.y() += height;
    } else if (firstPoint.x() > secondPoint.x() && firstPoint.y() < secondPoint.y()) {
        int length = firstPoint.y() - secondPoint.y();
        firstPoint.x() -= length;
        secondPoint.x() += length;
    }
}