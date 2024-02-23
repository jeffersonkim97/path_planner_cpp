#include "rrtc3D.h"
#include <iostream>

namespace rrtc3D{
RRTC3D::RRTC3D(){
    // Initialize Start Tree
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    startTime = 0;
    startPoint.position = startPos;
    startPoint.time = startTime;

    // Initialize Goal Tree
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;
    goalTime = END_POS_Z;
    goalPoint.position = endPos;
    goalPoint.time = goalTime;

    n = 10;
    minGoalTime = 50;
    maxGoalTime = 100;
    // RRT Loop Control
    step_size = 3;
    max_iter = 3000;

    rootStart = new Node;
    rootStart->parent = NULL;
    rootStart->vertex = startPoint;
    nodesStart.push_back(rootStart);
    
    rootGoal = new Node;
    rootGoal->parent = NULL;
    goalPoint.time = goalTime;
    rootGoal->vertex = goalPoint;
    nodesGoal.push_back(rootGoal);
}

Node* RRTC3D::randomSample(int counter){
    Node* sample;
    // float time = drand48() * WORLD_HEIGHT;
    int time = rand()%((int(WORLD_HEIGHT) - 0) + 1) + 0;

    Vector2f point;

    if (counter%2 == 0){
        point.x() = drand48() * VEHICLE_SPEED*time;
        point.y() = drand48() * VEHICLE_SPEED*time;
    } else {
        point.x() = drand48() * VEHICLE_SPEED*(END_POS_Z-time);
        point.y() = drand48() * VEHICLE_SPEED*(END_POS_Z-time);
    }
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT && time >= 0 && time <= WORLD_HEIGHT){
        
        // std::cout << "time: " << time << std::endl;
        // std::cout << "Point: " << point.x() << ", " << point.y() << std::endl;

        sample = new Node;
        Vertex v;
        v.position = point;
        v.time = time;

        sample->vertex = v;
        return sample;
    } else {
        return RRTC3D::randomSample(counter);
    }
    // return NULL;
}

int RRTC3D::distance(Vertex &p, Vertex &q){
    Vector2f posp = p.position;
    Vector2f posq = q.position;
    Vector2f v = posp - posq;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
    // float dt = abs(p.time - q.time);
    // return sqrt(powf(v.x(), 2) + powf(v.y(), 2) + powf(dt,2));
}

bool RRTC3D::reachable(Vertex p, Vertex q, int counter){
    // Vector2f posp = p.position;
    // Vector2f posq = q.position;
    float timep = p.time;
    float timeq = q.time;

    // std::cout << "p, q time: " << timep << ", " << timeq << std::endl;
    // std::cout << "q bigger?: " << (timeq-timep>0) << std::endl;

    if (counter%2 == 0) {
        if ((timeq - timep) > 0){
            return true;
        }
        return false;
    } else {
        if (timeq - timep < 0){
            return true;
        }
        return false;
    }
}

void RRTC3D::proximity(Vertex point, float radius, vector<Node *>& out_nodes, int counter){
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        double dist = distance(point, nodes[i]->vertex);
        if (dist < radius && reachable(nodes[i]->vertex, point, counter)) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

Node* RRTC3D::find_neighbor(Vertex point, int counter){
    float minDist = 1e9;
    Node *closest = NULL;
    vector<Node *> nodes;
    if (counter%2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        float dist = distance(point, nodes[i]->vertex);
        // std::cout << "reachable?: " << reachable(nodes[i]->vertex, point, counter) << std::endl;
        if ((dist < minDist) && reachable(point, nodes[i]->vertex, counter)){
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vertex RRTC3D::extend(Node *q, Node *qnear, int counter){
    Vector2f to = q->vertex.position;
    Vector2f from = qnear->vertex.position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f extended = from + step_size * intermediate;

    Vertex ret;
    if (counter%2 == 0){
        ret.position = extended;
        ret.time = (qnear->vertex.time - q->vertex.time)/intermediate.norm();
    } else {
        ret.position = extended;
        ret.time = (q->vertex.time - qnear->vertex.time)/intermediate.norm();
    }
    

    return ret;
}

double RRTC3D::Cost(Node *q){
    return q->cost;
}

double RRTC3D::PathCost(Node *qFrom, Node *qTo){
    return distance(qTo->vertex, qFrom->vertex);
}

void RRTC3D::add(Node *qnear, Node *qnew, int counter){
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

bool RRTC3D::reached(int counter){
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