#include "rrtp.h"
#include <iostream>

namespace rrtp{
RRTP::RRTP(){
    // Initialize Start Tree
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    startTime = 0;
    startPoint.position = startPos;
    startPoint.time = startTime;

    // Initialize Goal Tree
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;

    n = 10;
    minGoalTime = 50;
    maxGoalTime = 100;
    vector<double> parallel_sample_vector = RRTP::parallel_sample(minGoalTime, maxGoalTime, n);

    // RRT Loop Control
    step_size = 3;
    max_iter = 3000;

    rootStart = new Node;
    rootStart->parent = NULL;
    rootStart->vertex = startPoint;
    nodesStart.push_back(rootStart);
    
    vector<Node *> rootGoal;
    for (int i = 0; i < n; i++){
        rootGoali = new Node;
        rootGoali->parent = NULL;
        goalTime = parallel_sample_vector[i];
        goalPoint.time = goalTime;
        rootGoali->vertex = goalPoint;
        nodesGoal.push_back(rootGoali);
    }
}

vector<double> RRTP::parallel_sample(double tmin, double tmax, int n){
    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<double> distr(tmin, tmax);

    vector<double> tvec(n);
    for (int i = 0; i < n; i++){
        tvec[i] = distr(generator);
    }
    return tvec;
}

Node* RRTP::randomSample(){
    Node* sample;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_LENGTH);
    float time = drand48() * WORLD_HEIGHT;
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT && time >= 0 && time <= WORLD_HEIGHT){
        sample = new Node;
        Vertex v;
        v.position = point;
        v.time = time;

        sample->vertex = v;
        return sample;
    }
    return NULL;
}

int RRTP::distance(Vertex &p, Vertex &q){
    Vector2f posp = p.position;
    Vector2f posq = q.position;
    Vector2f v = posp - posq;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

void RRTP::proximity(Vertex point, float radius, vector<Node *>& out_nodes, int counter){
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        double dist = distance(point, nodes[i]->vertex);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

Node* RRTP::find_neighbor(Vertex point, int counter){
    float minDist = 1e9;
    Node *closest = NULL;
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        float dist = distance(point, nodes[i]->vertex);
        if (dist < minDist){
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vertex RRTP::extend(Node *q, Node *qnear){
    Vector2f to = q->vertex.position;
    Vector2f from = qnear->vertex.position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f extended = from + step_size * intermediate;
    float moveTime = intermediate.norm();
    
    Vertex ret;
    ret.position = extended;
    ret.time = moveTime;

    return ret;
}

double RRTP::Cost(Node *q){
    return q->cost;
}

double RRTP::PathCost(Node *qFrom, Node *qTo){
    return distance(qTo->vertex, qFrom->vertex);
}

void RRTP::add(Node *qnear, Node *qnew, int counter){
    qnew->parent = qnear;
    qnear->children.push_back(qnew);

    if (counter % 2 == 0){
        lastStartNode = qnew;
        nodesStart.push_back(qnew);
    } else {
        lastGoalNode = qnew;
        nodesGoal.push_back(qnew);
    }
}

bool RRTP::reached(int counter){
    // Fix: iteratively go through each vertex and find connection within end threshold
    
    if (counter % 2 == 0){
        for (size_t i = 0; i < nodesGoal.size(); i++){
            Node *goalVertex = nodesGoal[i];
            if (distance(lastStartNode->vertex, goalVertex->vertex) < END_DIST_THRESHOLD){
                return true;
            }
        }
        return false;
    } else {
        for (size_t i = 0; i < nodesStart.size(); i++){
            Node *startVertex = nodesStart[i];
            if (distance(lastGoalNode->vertex, startVertex->vertex) < END_DIST_THRESHOLD){
                return true;
            }
        }
        return false;
    }
}


}