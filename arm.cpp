#include "arm.h"

Joint::Joint() {
    length = 2.0f;
    rotation = Vector();
    p = Vector();
    
}

Arm::Arm() {
    list_joints = vector<Joint*>();
    base = Vector();
    target = Vector(4.0f, 0.0f, 0.0f);
}

Matrix4f Joint::rodrigues() {
        Vector3f R(rotation.x, rotation.y, rotation.z);
        R.normalize();

        Matrix3f crossProd(3,3);
        crossProd(0,0) = 0.0f; crossProd(0, 1) =  -(R(2)); crossProd(0,2) = R(1);
        crossProd(1,0) = R(2); crossProd(1, 1) = 0.0; crossProd(1,2) = -(R(0));
        crossProd(2,0) = -(R(1)); crossProd(2, 1) = R(0); crossProd(2,2) = 0.0;

        Matrix3f crossProd_squ(3,3);
        crossProd_squ = crossProd * crossProd;

        float theta = rotation.magnitude();

        MatrixXf result = (R * R.transpose()) + sin(theta) * crossProd - cos(theta)*crossProd_squ;
        result.resize(4,4);
        result(0, 3) = 0.0f; result(1, 3) = 0.0f; result(2,3) = 0.0f; result(3,3) = 1.0f;
        result(3, 0) = 0.0f; result(3, 1) = 0.0f; result(3,2) = 0.0f;
        cout << result;
        return result;
}

Vector Arm::F(vector<Vector> theta) {
    Matrix4f R1; R1 = list_joints[0]->rodrigues(); cout << R1;
    Matrix4f R2; R2 = list_joints[1]->rodrigues(); cout << R2;
    Matrix4f R3; R3 = list_joints[2]->rodrigues(); cout << R3;
    Matrix4f R4; R4 = list_joints[3]->rodrigues(); cout << R4;
}