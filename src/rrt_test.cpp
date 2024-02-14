#include "rrt_test.h"

int main()
{
    RRT rrt;

    int max_iter = 3000;

    for (int i = 0; i < max_iter; i++)
    {
        Vector2f random_sample = rrt.randomSample();
    }
    cout << endl;
}