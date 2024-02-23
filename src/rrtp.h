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
namespace rrtp{

struct Vertex{
   Vector2f position;
   float time;
   bool eliminated = true;
//    Vertex(Vector2f position, float time){
//       float posx = position.x();
//       float posy = position.y();
//       float timez = time;
//    }
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

class RRTP{
    public:
        RRTP();
        void initialize();
        vector<double> parallel_sample(double tmin, double tmax, int n);
        Node* randomSample();
        Node* find_neighbor(Vertex point, int counter);
        int distance(Vertex &p, Vertex &q);
        void proximity(Vertex point, float radius, vector<Node *>& out_nodes, int counter);
        Vertex extend(Node *q, Node *qnear);
        double Cost(Node *q);
        double PathCost(Node *qFrom, Node *qTo);
        void add(Node *qnear, Node *qnew, int counter);
        bool reached(int counter);

        vector<double> parallel_sample_vector;
        vector<Node *> nodesStart, nodesGoal, *rootGoal;
        vector<Node *> path;
        Node *rootStart, *lastStartNode, *lastGoalNode, *rootGoali;
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