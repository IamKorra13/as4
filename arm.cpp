#include "arm.h"

Joint::Joint() {
    length = 2.0f;
    rotation << 0.0f, 0.0f, 90.0f;
    p << 0.0f, 0.0f, 0.0f;
    
}

Arm::Arm() {
    list_joints = vector<Joint*>();
    base << 0.0f, 0.0f, 0.0f;
    goal << 4, 0, 0;
}

Matrix4f Joint::transformation() {
	Matrix4f result;
	result << Matrix4f::Identity();
	result(0,3) = length;
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

    Vector3f ret(result(0), result(1), result(2));
    // cout << "Theta:" << endl << theta;
    // cout << "ret: " << endl << ret << endl;=
    return ret;
}

MatrixXf Arm::psuedo_inv_jacobian(VectorXf theta) {
	MatrixXf J = jacobian(theta);

	float epsilon = 0.0001f;
	MatrixXf result (12, 3);
	Matrix3f sigma(3, 3);

	JacobiSVD<MatrixXf> svd(J, ComputeThinU | ComputeThinV);
	Vector3f singularValues = svd.singularValues();

	float sigma1, sigma2, sigma3;
	if(singularValues(0) < epsilon) {
		sigma1 = 0.0f;
	} else {
		sigma1 = 1/singularValues(0);
	}

	if(singularValues(1) < epsilon) {
		sigma2 = 0.0f;
	} else {
		sigma2 = 1/singularValues(1);
	}

	if(singularValues(2) < epsilon) {
		sigma3 = 0.0f;
	} else {
		sigma3 = 1/singularValues(2);
	}

	sigma << sigma1, 0, 0,
			0, sigma2, 0,
			0, 0, sigma3;

	result = svd.matrixV() * sigma * svd.matrixU().transpose();

	// cout << "psuedo inverse: " << endl << result;
	return result;
}

MatrixXf Arm::jacobian(VectorXf theta) {

	float epsilon = 0.005;
	MatrixXf result(3, 12);
	Vector3f endpoint; endpoint << F(theta);


	VectorXf add(12);
	VectorXf subtract(12);
	

	// Fx/rx1
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(0) += epsilon;
	subtract(0) -= epsilon;
	result(0, 0) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx1
	result(1, 0) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx1
	result(2, 0) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

	/////////////////////////

	// Fx/ry1
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(1) += epsilon;
	subtract(1) -= epsilon;
	result(0, 1) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/ry1
	result(1, 1) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/ry1
	result(2, 1) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

	/////////////////////////

	// Fx/rz1
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(2) += epsilon;
	subtract(2) -= epsilon;
	result(0, 2) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rz1
	result(1, 2) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rz1
	result(2, 2) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

	/////////////////////////

	// Fx/rx2
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(3) += epsilon;
	subtract(3) -= epsilon;
	result(0, 3) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx2
	result(1, 3) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx2
	result(2, 3) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

	/////////////////////////

	// Fx/ry2
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(4) += epsilon;
	subtract(4) -= epsilon;
	result(0, 4) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx2
	result(1, 4) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx2
	result(2, 4) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

		/////////////////////////

	// Fx/rz2
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(5) += epsilon;
	subtract(5) -= epsilon;
	result(0, 5) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rz2
	result(1, 5) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rz2
	result(2, 5) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

			/////////////////////////

	// Fx/rx3
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(6) += epsilon;
	subtract(6) -= epsilon;
	result(0, 6) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx3
	result(1, 6) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx3
	result(2, 6) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

				/////////////////////////

	// Fx/ry3
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(7) += epsilon;
	subtract(7) -= epsilon;
	result(0, 7) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx3
	result(1, 7) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx3
	result(2, 7) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

				/////////////////////////

	// Fx/rz3
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(8) += epsilon;
	subtract(8) -= epsilon;
	result(0, 8) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx3
	result(1, 8) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx3
	result(2, 8) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

				/////////////////////////

	// Fx/rx4
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(9) += epsilon;
	subtract(9) -= epsilon;
	result(0, 9) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx3
	result(1, 9) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx3
	result(2, 9) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);


				/////////////////////////

	// Fx/ry4
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(10) += epsilon;
	subtract(10) -= epsilon;
	result(0, 10) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx3
	result(1, 10) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx3
	result(2, 10) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

					/////////////////////////

	// Fx/ry4
	add << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	subtract << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5), theta(6), theta(7), theta(8), theta(9), theta(10), theta(11);
	add(11) += epsilon;
	subtract(11) -= epsilon;
	result(0, 11) = (F(add)(0) - F(subtract)(0))/ (2*epsilon);

	// Fy/rx3
	result(1, 11) = (F(add)(1) - F(subtract)(1))/ (2*epsilon);

	// Fz/rx3
	result(2, 11) = (F(add)(2) - F(subtract)(2))/ (2*epsilon);

	return result;
}

Vector3f Arm::C(VectorXf theta) {
	Vector3f result = F(theta) - goal;
	return result;
}

VectorXf Arm::update(VectorXf theta) {
	VectorXf newTheta(12);
	newTheta = theta - psuedo_inv_jacobian(theta) * C(theta) * step_size;
	return newTheta;
}