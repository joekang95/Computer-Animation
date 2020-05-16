#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

	// Converts degrees to radians.
	template<typename real>
	inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

	template<typename real>
	Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order) {
		Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
		Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
		Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

		switch (order)
		{
		case RotateOrder::XYZ:
			return RZ * RY * RX;
		case RotateOrder::YZX:
			return RX * RZ * RY;
		case RotateOrder::ZXY:
			return RY * RX * RZ;
		case RotateOrder::XZY:
			return RY * RZ * RX;
		case RotateOrder::YXZ:
			return RZ * RX * RY;
		case RotateOrder::ZYX:
			return RX * RY * RZ;
		}
		assert(0);
	}

	// Performs forward kinematics, using the provided "fk" class.
	// This is the function whose Jacobian matrix will be computed using adolc.
	// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
	//   IKJointIDs is an array of integers of length "numIKJoints"
	// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
	// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
	template<typename real>
	void forwardKinematicsFunction(
		int numIKJoints, const int * IKJointIDs, const FK & fk,
		const std::vector<real> & eulerAngles, std::vector<real> & handlePositions) {
		// Students should implement this.
		// The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
		// The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
		// Then, implement the same algorithm into this function. To do so,
		// you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
		// Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
		// It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
		// so that code is only written once. We considered this; but it is actually not easily doable.
		// If you find a good approach, feel free to document it in the README file, for extra credit.
		int numJoint = fk.getNumJoints();
		real eulerAngle[3], jointOrientationEulerAngle[3];
		Mat3<real> R_L, R_O;
		vector<Mat3<real>> localTransforms(numJoint), globalTransforms(numJoint);
		vector<Vec3<real>> localTranslates(numJoint), globalTranslates(numJoint);
		for (int i = 0; i < numJoint; i++) {

			// Obtain R_L = joint rotation
			eulerAngle[0] = eulerAngles[3 * i];
			eulerAngle[1] = eulerAngles[3 * i + 1];
			eulerAngle[2] = eulerAngles[3 * i + 2];
			RotateOrder rotateOrder = fk.getJointRotateOrder(i);
			R_L = Euler2Rotation(eulerAngle, rotateOrder);		  // R_L usually XYZ, so need to know rotate order

			// Obtain R_O = joint orient
			Vec3d jointOrient = fk.getJointOrient(i);
			jointOrientationEulerAngle[0] = jointOrient[0];
			jointOrientationEulerAngle[1] = jointOrient[1];
			jointOrientationEulerAngle[2] = jointOrient[2];
			R_O = Euler2Rotation(jointOrientationEulerAngle, XYZ); // R_O always uses XYZ

			localTransforms[i] = R_O * R_L; // R = R_O * R_L

			// Obtain Local Translation
			Vec3d jointRestTranslate = fk.getJointRestTranslation(i);
			localTranslates[i][0] = jointRestTranslate[0];
			localTranslates[i][1] = jointRestTranslate[1];
			localTranslates[i][2] = jointRestTranslate[2];

			// Obtain Global Translation
			int current = fk.getJointUpdateOrder(i);
			Vec3d jointRestTranslateG = fk.getJointRestTranslation(current);
			globalTranslates[current][0] = jointRestTranslateG[0];
			globalTranslates[current][1] = jointRestTranslateG[1];
			globalTranslates[current][2] = jointRestTranslateG[2];
		}

		for (int i = 0; i < numJoint; i++) {
			int current = fk.getJointUpdateOrder(i);	// Obtain Current Joint in Order
			int jointParent = fk.getJointParent(current); // Obtain Parent of Current Joint
			if (jointParent == -1) { 
				// If Root, Global Transform = Local Transform
				globalTransforms[current] = localTransforms[current];
			}
			else { 
				// Child_Global = Parent_Global * Child_Local
				// Parameters: const Mat3<real> & R1, const Vec3<real> & t1, const Mat3<real> & R2, const Vec3<real> & t2, Mat3<real> & Rout, Vec3<real> & tout
				multiplyAffineTransform4ds(
					globalTransforms[jointParent], globalTranslates[jointParent],
					localTransforms[current], localTranslates[current],
					globalTransforms[current], globalTranslates[current]
				);
			}
		}

		// Update Handle Position
		for (int i = 0; i < numIKJoints; i++) {
			int id = IKJointIDs[i];
			handlePositions[3 * i] = globalTranslates[id][0];
			handlePositions[3 * i + 1] = globalTranslates[id][1];
			handlePositions[3 * i + 2] = globalTranslates[id][2];
		}
	}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID) {
	this->numIKJoints = numIKJoints;
	this->IKJointIDs = IKJointIDs;
	this->fk = inputFK;
	this->adolc_tagID = adolc_tagID;

	FKInputDim = fk->getNumJoints() * 3;
	FKOutputDim = numIKJoints * 3;

	train_adolc();
}

void IK::train_adolc() {
	// Students should implement this.
	// Here, you should setup adol_c:
	//   Define adol_c inputs and outputs. 
	//   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
	//   This will later make it possible for you to compute the gradient of this function in IK::doIK
	//   (in other words, compute the "Jacobian matrix" J).
	// See ADOLCExample.cpp .
	int n = FKInputDim, m = FKOutputDim;
	trace_on(adolc_tagID);

	vector<adouble> eulerAngles(n);		// Define the input of the function f
	for (int i = 0; i < n; i++) {
		eulerAngles[i] <<= 0.0;			// The <<= syntax tells ADOL-C that these are the input variables.
	}

	vector<adouble> handlePosition(m);	// Define the output of the function f

	// Compute function f (forwardKinematicsFunction):
	forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eulerAngles, handlePosition);


	vector<double> output(m);
	for (int i = 0; i < m; i++) {
		handlePosition[i] >>= output[i];// Use >>= to tell ADOL-C that y[i] are the output variables
	}

	trace_off();						// ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles, int lbs, int dls) {
	// You may find the following helpful:
	int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

	// Students should implement this.
	// Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
	// Specifically, use ::function, and ::jacobian .
	// See ADOLCExample.cpp .
	//
	// Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
	// Note that at entry, "jointEulerAngles" contains the input Euler angles. 
	// Upon exit, jointEulerAngles should contain the new Euler angles.

	int n = FKInputDim, m = FKOutputDim;
	double * input = jointEulerAngles->data();
	vector<double> output(m);
	::function(adolc_tagID, m, n, input, &output[0]);		// Run Function f (forwardKinematicsFunction)

	vector<double> jacobianMatrix(m * n);					// Declare matrix
	vector<double *> jacobianMatrixEachRow(m);				// Declare pointers to each row 

	for (int i = 0; i < m; i++) {
		jacobianMatrixEachRow[i] = &jacobianMatrix[i * n];	// Set Poniter of Each Row
	}
	::jacobian(adolc_tagID, m, n, input, &jacobianMatrixEachRow[0]); // Run Jacobian Matrix (Gradient) Calculation

	// Ax = B => (J^T*J + £\I)£G£c = J^T * £Gp
	double alpha = 0.02;						

	Eigen::MatrixXd J(m, n), J_T(n, m), I(n, n);			// Declare J, J Transpose, and Identity Matrix

	// I = Identity Matrix
	I = Eigen::MatrixXd::Identity(n, n);					// Set Identity Matrix

	// J = Jacobian Matrix
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			J(i, j) = jacobianMatrix[i * n + j];			// Store Jacboian Values
		}
	}

	// J^T = Jacobian Transpose Matrix
	J_T = J.transpose();									// Set Jacobian Transpose Matrix

	// £Gb = m ¡Ñ 1 vector representing the change of handle global positions
	vector<double> dp(m);
	for (int i = 0; i < m; i++) {
		dp[i] = targetHandlePositions->data()[i] - output[i]; // Original Position - Calculated New Postion
	}
	Eigen::VectorXd delta_p = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dp.data(), dp.size()); // Set delta p

	if (dls) {
		// A = (J^T*J + £\I)
		Eigen::MatrixXd A(n, n);
		A = J_T * J + alpha * I;				// Compute A

		// B = J^T * £Gp
		Eigen::VectorXd B(m);
		B = J_T * delta_p;						// Compute B

		// x = £G£c
		Eigen::VectorXd x = A.ldlt().solve(B);	// Solve x (£G£c) using ldlt
		vector<double> euler(n);				// Euler Angles £G£c
		for (int i = 0; i < n; i++) {
			euler[i] = x[i];					// Set £G£c
		}
		for (int i = 0; i < numJoints; i++) {
			jointEulerAngles[i][0] += euler[3 * i];
			jointEulerAngles[i][1] += euler[3 * i + 1];
			jointEulerAngles[i][2] += euler[3 * i + 2];
		}
	}
	else {

		// £G£c = J_dagger * £Gp
		// J_dagger = J^T (J J^T)^-1
		Eigen::MatrixXd J_Dagger(n, m), J_Inverse(m, m);
		J_Inverse = J * J_T;
		J_Inverse = J_Inverse.inverse();
		J_Dagger = J_T * J_Inverse;

		Eigen::VectorXd dtheta(n);
		dtheta = J_Dagger * delta_p;
		vector<double> euler(n);					// Euler Angles £G£c
		for (int i = 0; i < n; i++) {
			euler[i] = dtheta[i];					// Set £G£c
		}
		for (int i = 0; i < numJoints; i++) {
			jointEulerAngles[i][0] += euler[3 * i];
			jointEulerAngles[i][1] += euler[3 * i + 1];
			jointEulerAngles[i][2] += euler[3 * i + 2];
		}
	}

}

