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
        Vector3f rotation;
        Vector3f p;

        Matrix4f rodrigues();
        Matrix4f transformation();
        void print();
};


class Arm
{
    public:
        Arm();
        vector<Joint*> list_joints;
        Vector3f base;
        Vector3f target;
        float step_size;
        
        Vector4f F(vector<Vector3f>); /* get endpoint of the arm */
        MatrixXf jacobian(vector<Vector3f>);
        void print();
};

#endif