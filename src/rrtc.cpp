#include "rrtc.h"
#include <iostream>

namespace rrtc{
RRTC::RRTC(){
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;

    // RRT Loop Control
    step_size = 3;
    max_iter = 3000;

    rootStart = new Node;
    rootStart->parent = NULL;
    rootStart->position = startPos;
    
    rootGoal = new Node;
    rootGoal->parent = NULL;
    rootGoal->position = endPos;
    lastNode = rootStart;
    nodesStart.push_back(rootStart);
    nodesGoal.push_back(rootGoal);
    
    int counter = 0;
}

Node* RRTC::randomSample(){
    Node* sample;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT){
        sample = new Node;
        sample->position = point;
        return sample;
    }
    return NULL;
}

int RRTC::distance(Vector2f &p, Vector2f &q){
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

void RRTC::proximity(Vector2f point, float radius, vector<Node *>& out_nodes, int counter){
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

Node* RRTC::find_neighbor(Vector2f point, int counter){
    float minDist = 1e9;
    Node *closest = NULL;
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist){
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vector2f RRTC::extend(Node *q, Node *qnear){
    Vector2f to = q->position;
    Vector2f from = qnear->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

double RRTC::Cost(Node *q){
    return q->cost;
}

double RRTC::PathCost(Node *qFrom, Node *qTo){
    return distance(qTo->position, qFrom->position);
}

void RRTC::add(Node *qnear, Node *qnew, int counter){
    qnew->parent = qnear;
    qnear->children.push_back(qnew);
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    nodes.push_back(qnew);
    lastNode = qnew;
}

bool RRTC::reached(){
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD){
        return true;
    }
    return false;
}
}