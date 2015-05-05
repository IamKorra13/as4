#include "arm.h"

Joint::Joint() {
    length = 2.0f;
    rotation << 0.0f, 0.0f, 0.0f;
    p << 0.0f, 0.0f, 0.0f;
    
}

Arm::Arm() {
    list_joints = vector<Joint*>();
    base << 0, 0, 0;
    target << 4, 0, 0;
}

Matrix4f Joint::transformation() {
	Matrix4f result;
	result << Matrix4f::Identity();
	result(0,3) = length;
	//cout << result;
	return result;
}

Matrix4f Joint::rodrigues() {
        Vector3f R; R << rotation(0), rotation(1), rotation(2);

        // if 0 vector
        if(R.norm() <= 0.0f) {
        	Matrix4f result;
        	result << MatrixXf::Zero(4, 4);
        	return result;
        }

        R.normalize();

        Matrix3f crossProd(3,3);
        crossProd(0,0) = 0.0f; crossProd(0, 1) =  -(R(0)); crossProd(0,2) = R(1);
        crossProd(1,0) = R(2); crossProd(1, 1) = 0.0; crossProd(1,2) = -(R(0));
        crossProd(2,0) = -(R(1)); crossProd(2, 1) = R(0); crossProd(2,2) = 0.0;

        Matrix3f crossProd_squ(3,3);
        crossProd_squ = crossProd * crossProd;

        float theta = rotation.norm();

        MatrixXf result = (R * R.transpose()) + sin(theta) * crossProd - cos(theta)*crossProd_squ;
        result.conservativeResize(4,4);
        result(0, 3) = 0.0f; result(1, 3) = 0.0f; result(2,3) = 0.0f; result(3,3) = 1.0f;
        result(3, 0) = 0.0f; result(3, 1) = 0.0f; result(3,2) = 0.0f;

      	//cout << "R vector is: " << R(0) << R(1) << R(2) << endl;
        //cout << "Rodrigues:" << endl << result;
        return result;
}

void Joint::print() {
	cout << "Length: " << length << endl;
	cout << "Rotation: " << rotation << endl;
}

Vector4f Arm::F(vector<Vector3f> theta) {
    Matrix4f R1; R1 = list_joints[0]->rodrigues();
    Matrix4f R2; R2 = list_joints[1]->rodrigues();
    Matrix4f R3; R3 = list_joints[2]->rodrigues();
    Matrix4f R4; R4 = list_joints[3]->rodrigues();

    Matrix4f T1; T1 = list_joints[0]->transformation();
    Matrix4f T2; T2 = list_joints[0]->transformation();
    Matrix4f T3; T3 = list_joints[0]->transformation();
    Matrix4f T4; T4 = list_joints[0]->transformation();

    Vector4f identity(0.0, 0.0, 0.0, 1.0);
    Vector4f result;

    result = R1 * T1 * R2 * T2 * R3 * T3 * R4 * T4 * identity;

    cout << "R1:" << endl << R1 << endl;
    cout << "T1:" << endl << T1 << endl;
    cout << "R2:" << endl << R2 << endl;
    cout << "T2:" << endl << T2 << endl;
    cout << "R3:" << endl << R1 << endl;
    cout << "T3:" << endl << T1 << endl;
    cout << "R4:" << endl << R1 << endl;
    cout << "T4:" << endl << T1 << endl;

    cout << "R1 * T1: " << R1 * T1 << endl;
    cout << "result: " << endl << result;

    return result;
}