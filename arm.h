#ifndef ARM_H
#define ARM_H
#include <cmath>
#include <iostream>
#include <vector>
#include "vector.cpp"
#include "Eigen/Dense"
#include "Eigen/SVD"

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
        Vector3f goal;
        float step_size;
        
        //VectorXf bigTheta;
        
        Matrix4f rodrigues(Vector3f);
        Vector3f F(VectorXf); /* get endpoint of the arm */
        MatrixXf jacobian(VectorXf);
        MatrixXf psuedo_inv_jacobian(VectorXf);
        Vector3f C(VectorXf);
        VectorXf update(VectorXf);
        void print();
};



#endif