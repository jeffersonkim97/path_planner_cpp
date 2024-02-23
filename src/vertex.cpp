#include "vertex.h"
#include <iostream>

namespace vertex3D{
VERTEX::VERTEX(Vector2f pos, float t){
    Vertex *vertex;
    // vertex = new Vertex;
    vertex->position = pos;
    vertex->time_stamp = t;
    
}

Vector2f VERTEX::getPos(){
    // return vertex->position
}

float VERTEX::getTime(){
    // return vertex
}


}