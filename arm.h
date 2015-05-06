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
        
        Matrix4f rodrigues(Vector3f);
        Vector3f F(VectorXf); /* get endpoint of the arm */
        MatrixXf jacobian(VectorXf);
        void print();
};



#endif