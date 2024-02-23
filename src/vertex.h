#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace vertex3D{

struct Vertex {
    Vector2f position;
    float time_stamp;
};

class VERTEX{
    public:
        VERTEX(Vector2f pos, float t);
        Vector2f getPos();
        float getTime();
};

}