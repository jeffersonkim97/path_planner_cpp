#ifndef RRT_TEST_H
#define RRT_TEST_H

#include <iostream>
#include <vector>
#include <string>
#include <vector>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include "constants3D.h"
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace Eigen;
namespace rrtc3D{

struct Vertex{
   Vector2f position;
   float time;
   bool eliminated = true;
};

struct Node {
    vector<Node *> children;
    Node *parent;
    // Vector2f position;
    Vertex vertex;
    float orientation;
    double cost;
    int counter;
};

class RRTC3D{
    public:
        RRTC3D();
        void initialize();
        Node* randomSample(int counter);
        Node* find_neighbor(Vertex point, int counter);
        int distance(Vertex &p, Vertex &q);
        void proximity(Vertex point, float radius, vector<Node *>& out_nodes, int counter);
        bool reachable(Vertex p, Vertex q, int counter);
        Vertex extend(Node *q, Node *qnear, int counter);
        double Cost(Node *q);
        double PathCost(Node *qFrom, Node *qTo);
        void add(Node *qnear, Node *qnew, int counter);
        bool reached(int counter);

        vector<double> parallel_sample_vector;
        vector<Node *> nodesStart, nodesGoal;
        vector<Node *> path;
        Node *rootStart, *rootGoal, *lastStartNode, *lastGoalNode;
        Vector2f startPos, endPos;
        double startTime, goalTime;
        Vertex startPoint, goalPoint;

        int max_iter;
        int step_size;
        int n;
        double minGoalTime, maxGoalTime;
};
}
# endif