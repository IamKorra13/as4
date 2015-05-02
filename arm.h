#ifndef ARM_H
#define ARM_H
#include <cmath>
#include <iostream>
#include <vector>
#include "vector.cpp"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;


class Joint
{
    public:
        Joint();
        float length;
        float angle;
};


class Arm
{
    public:
        Arm();
        vector<Joint*> list_joints;
        Vector base;
        Vector target;
        float step_size;
};

#endif