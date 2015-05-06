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

void Joint::print() {
	cout << "Length: " << length << endl;
	cout << "Rotation: " << rotation << endl;
}


Matrix4f Arm::rodrigues(Vector3f rot) {
        Vector3f R; R << rot(0), rot(1), rot(2);
/*
        // if 0 vector
        if(R.norm() <= 0.0f) {
        	Matrix4f result;
        	result << MatrixXf::Zero(4, 4);
        	return result;
        }
*/
  		if(R.norm() > 0.0f) {
  			R.normalize();
  		}
        

        Matrix3f identity;
        identity << Matrix3f::Identity();

        Matrix3f crossProd(3,3);
        crossProd(0,0) = 0.0f; crossProd(0, 1) =  -(R(0)); crossProd(0,2) = R(1);
        crossProd(1,0) = R(2); crossProd(1, 1) = 0.0; crossProd(1,2) = -(R(0));
        crossProd(2,0) = -(R(1)); crossProd(2, 1) = R(0); crossProd(2,2) = 0.0;

        Matrix3f crossProd_squ(3,3);
        crossProd_squ = crossProd * crossProd;

        float theta = rot.norm();

        MatrixXf result = identity + sin(theta) * crossProd + (1-cos(theta))*crossProd_squ;
        cout << "result matrix: " << endl << result;
        result.conservativeResize(4,4);
        result(0, 3) = 0.0f; result(1, 3) = 0.0f; result(2,3) = 0.0f; result(3,3) = 1.0f;
        result(3, 0) = 0.0f; result(3, 1) = 0.0f; result(3,2) = 0.0f;

      	//cout << "R vector is: " << R(0) << R(1) << R(2) << endl;
        //cout << "Rodrigues:" << endl << result;
        return result;
}


Vector4f Arm::F(vector<Vector3f> theta) {
    Matrix4f R1; R1 = rodrigues(theta[0]);
    Matrix4f R2; R2 = rodrigues(theta[1]);
    Matrix4f R3; R3 = rodrigues(theta[2]);
    Matrix4f R4; R4 = rodrigues(theta[3]);

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

MatrixXf Arm::jacobian(vector<Vector3f> theta) {
	float epsilon = 0.0005;
	MatrixXf result(3, 12);
	Vector3f endpoint; endpoint << F(theta);

	//finite differences

	//
	vector<Vector3f> add = vector<Vector3f>();
	for(int i = 0; i < 3; i++) {
		add[i] = theta[i];
	}
	vector<Vector3f> subtract = vector<Vector3f>();
	for(int i = 0; i < 3; i++) {
		subtract[i] = theta[i];
	}
	add[1](0) += epsilon;
	subtract[1](0) -= epsilon;
	cout << "Theta:" << endl << theta[0] << theta[1] << theta[2] << theta[3];
	cout << "add:" << endl << add[0] << add[1] << add[2] << add[3];
	cout << "subtract:" << endl << subtract[0] << subtract[1] << subtract[2] << subtract[3];
	//result(0, 0) = (F(add) - F(subtract))/ (2*epsilon);
}