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
        Vector rotation;
        Vector p;

        Matrix4f rodrigues();
        Matrix4f transformation();
};


class Arm
{
    public:
        Arm();
        vector<Joint*> list_joints;
        Vector base;
        Vector target;
        vector<Vector> rotations;
        float step_size;
        
        Vector F(vector<Vector>); /* get endpoint of the arm */
};

#endif