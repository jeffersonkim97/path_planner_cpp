#ifndef RRT_TEST_H
#define RRT_TEST_H

#include <iostream>
#include <vector>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "constants.h"
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace Eigen;

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
};

class RRTC{
    public:
        RRTC();
        void initialize();
        Node* randomSample();
        Node* find_neighbor(Vector2f point);
        int distance(Vector2f &p, Vector2f &q);
        Vector2f extend(Node *q, Node *qnear);
        void add(Node *qnear, Node *qnew);
        bool reached();

        vector<Node *> nodes;
        vector<Node *> path;
        Node *root, *lastNode;
        Vector2f startPos, endPos;

        int max_iter;
        int step_size;
};

# endif