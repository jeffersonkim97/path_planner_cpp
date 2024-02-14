#include "rrt.h"

RRT::RRT(){
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;

    // Root added as new node, no parent
    root = new Node;
    root->parent = Null;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);

    // RRT Loop Control
    step_size = 3;
    max_iter = 3000;
}

void RRT::initialize(){
    root = new Node;
    root->parent = Null;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
}

Node* RRT::randomSample(){
    Node* ret;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT){
        ret = new Node;
        ret->position = point;
        return ret;
    }
    return NULL;
}

int RRT::distance(Vector2f &p, Vector2f &q){
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

Node* RRT::find_neighbor(Vector2f point){
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++){
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist){
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vector2f RRT::extend(Node *q, Node *qnear){
    Vector2f to = q->position;
    Vector2f from = qnear->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

void RRT::add(Node *qnear, Node *qnew){
    qnew->parent = qnear;
    qnear->children.push_back(qnew);
    nodes.push_back(qnew);
    lastNode = qnew;
}

bool RRT::reached(){
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD){
        return true;
    }
    return false;
}