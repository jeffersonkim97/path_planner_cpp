#include "rrt_connect.h"
#include <iostream>

RRTC::RRTC(){
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;

    // Root added as new node, no parent
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);

    // RRT Loop Control
    step_size = 3;
    max_iter = 3000;
}

void RRTC::initialize(){
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
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

Node* RRTC::find_neighbor(Vector2f point){
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

Vector2f RRTC::extend(Node *q, Node *qnear){
    Vector2f to = q->position;
    Vector2f from = qnear->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

void RRTC::add(Node *qnear, Node *qnew){
    qnew->parent = qnear;
    qnear->children.push_back(qnew);
    nodes.push_back(qnew);
    lastNode = qnew;
}

bool RRTC::reached(){
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD){
        return true;
    }
    return false;
}

int main()
{
    RRTC rrt;
    rrt.initialize();

    int max_iter = rrt.max_iter;

    for (int i = 0; i < max_iter; i++)
    {
        Node *q = rrt.randomSample();

        if (q){
            Node *qnear = rrt.find_neighbor(q->position);
            if (rrt.distance(q->position, qnear->position)>rrt.step_size){
                Vector2f qNew = rrt.extend(q, qnear);
                Node *qnew = new Node;
                qnew->position = qNew;

                rrt.add(qnear, qnew);

                Node *qParent = qnear->parent;
                qnear->parent = qnew;
                qnew->children.push_back(qnear);
            };
        };

        if (rrt.reached()){
            cout << "End Position: " << rrt.endPos << endl;
            cout << "Last RRT Node: " << rrt.lastNode->position << endl;
            cout << "Destination Reached \n";
            break;
        };
    }
    // Node *q;
    // if (rrt.reached()) {
    //     q = rrt.lastNode;  
    // }    else{
    //     q = rrt.find_neighbor(rrt.endPos);
    // }

    // while (q!= NULL){
    //     rrt.path.push_back(q);
    //     q = q->parent;
    // };
};