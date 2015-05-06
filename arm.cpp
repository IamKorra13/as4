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
	cout << "arm length: " << length;
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
        //cout << "result matrix: " << endl << result;
        result.conservativeResize(4,4);
        result(0, 3) = 0.0f; result(1, 3) = 0.0f; result(2,3) = 0.0f; result(3,3) = 1.0f;
        result(3, 0) = 0.0f; result(3, 1) = 0.0f; result(3,2) = 0.0f;

        return result;
}


Vector3f Arm::F(VectorXf theta) {
	Vector3f rot1(theta(0), theta(1), theta(2));
	Vector3f rot2(theta(3), theta(4), theta(5));
	Vector3f rot3(theta(6), theta(7), theta(8));
	Vector3f rot4(theta(9), theta(10), theta(11));
    Matrix4f R1; R1 = rodrigues(rot1);
    Matrix4f R2; R2 = rodrigues(rot2);
    Matrix4f R3; R3 = rodrigues(rot3);
    Matrix4f R4; R4 = rodrigues(rot4);

    Matrix4f T1; T1 = list_joints[0]->transformation();
    Matrix4f T2; T2 = list_joints[1]->transformation();
    Matrix4f T3; T3 = list_joints[2]->transformation();
    Matrix4f T4; T4 = list_joints[3]->transformation();

    Vector4f identity(0.0, 0.0, 0.0, 1.0);
    Vector4f result;

    result = R1 * T1 * R2 * T2 * R3 * T3 * R4 * T4 * identity;
/*
    cout << "R1:" << endl << R1 << endl;
    cout << "T1:" << endl << T1 << endl;
    cout << "R2:" << endl << R2 << endl;
    cout << "T2:" << endl << T2 << endl;
    cout << "R3:" << endl << R1 << endl;
    cout << "T3:" << endl << T1 << endl;
    cout << "R4:" << endl << R1 << endl;
    cout << "T4:" << endl << T1 << endl;
    */
    Vector3f ret(result(0), result(1), result(2));
    cout << "Theta:" << endl << theta;
    cout << "ret: " << endl << ret << endl;
    return ret;
}

MatrixXf Arm::jacobian(VectorXf theta) {

	float epsilon = 0.005;
	MatrixXf result(3, 12);
	Vector3f endpoint; endpoint << F(theta);


	VectorXf add(12);
	VectorXf subtract(12);
	

	// first element
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(0) += epsilon;
	subtract(0) -= epsilon;

	//cout << "Theta:" << endl << theta[0] << endl << theta[1] << endl << theta[2] << endl << theta[3];
	//cout << "add:" << endl << add[0] << endl << add[1] << endl << add[2] << endl << add[3];
	//cout << "subtract:" << endl << subtract[0] << subtract[1] << subtract[2] << subtract[3];
	result(0, 0) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);
	cout << "Jacobian: " << endl << result;
}