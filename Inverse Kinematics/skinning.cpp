#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include "quaternion.h"
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li


Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
	const std::string & meshSkinningWeightsFilename) {
	this->numMeshVertices = numMeshVertices;
	this->restMeshVertexPositions = restMeshVertexPositions;

	cout << "Loading skinning weights..." << endl;
	ifstream fin(meshSkinningWeightsFilename.c_str());
	assert(fin);
	int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
	fin >> numWeightMatrixRows >> numWeightMatrixCols;
	assert(fin.fail() == false);
	assert(numWeightMatrixRows == numMeshVertices);
	int numJoints = numWeightMatrixCols;

	vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
	vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
	fin >> ws;
	while (fin.eof() == false) {
		int rowID = 0, colID = 0;
		double w = 0.0;
		fin >> rowID >> colID >> w;
		weightMatrixColumnIndices[rowID].push_back(colID);
		weightMatrixEntries[rowID].push_back(w);
		assert(fin.fail() == false);
		fin >> ws;
	}
	fin.close();

	// Build skinning joints and weights.
	numJointsInfluencingEachVertex = 0;
	for (int i = 0; i < numMeshVertices; i++)
		numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
	assert(numJointsInfluencingEachVertex >= 2);

	// Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
	meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
	meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
	for (int vtxID = 0; vtxID < numMeshVertices; vtxID++) {
		vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
		for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++) {
			int frameID = weightMatrixColumnIndices[vtxID][j];
			double weight = weightMatrixEntries[vtxID][j];
			sortBuffer[j] = make_pair(weight, frameID);
		}
		sortBuffer.resize(weightMatrixEntries[vtxID].size());
		assert(sortBuffer.size() > 0);
		sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
		for (size_t i = 0; i < sortBuffer.size(); i++) {
			meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
			meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
		}

		// Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
		// the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
	}
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions, int numJoints, int lbs) const {
	// Students should implement this

	if (lbs) {
		/*************************************************************************************************************
		Linear Blend Skinning
		Rigid transform: p -> Rp + t
		- R = 3 x 3 rotation matrix, t = 3 x 1 vector
		or
		p_i = Sigma_j(w_j * M_j * p_i_bar)
		- j = all joints that affect the vertex ----> numJointsInfluencingEachVertex
		- w_j = skinning weight of joint j to the vertex ----> meshSkinningWeights
		- M_j = joint's skinning transform matrix 4 x 4 ----> jointSkinTransforms
		- p_i_bar = homogeneous coordinates of the rest/undeformed position of vertex ----> restMeshVertexPositions
		**************************************************************************************************************/

		Vec4d newVertexPositions, restVertexPositions;
		int index = 0;
		for (int i = 0; i < numMeshVertices; i++) {
			newVertexPositions = Vec4d(0.0, 0.0, 0.0, 0.0);
			restVertexPositions = Vec4d(restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2], 1.0);

			for (int j = 0; j < numJointsInfluencingEachVertex; j++) {
				index = i * numJointsInfluencingEachVertex + j;
				newVertexPositions += meshSkinningWeights[index] * jointSkinTransforms[meshSkinningJoints[index]] * restVertexPositions;
			}
			newMeshVertexPositions[3 * i + 0] = newVertexPositions[0];
			newMeshVertexPositions[3 * i + 1] = newVertexPositions[1];
			newMeshVertexPositions[3 * i + 2] = newVertexPositions[2];
		}
	}
	else {
		/*************************************************************************************************************
		Dual Quaternion Skinning
		Covert R_i -> q_i
		Form: (q_i, 1/2 * t_i * q_i) =: q_i_hat
		q_hat(p) = Sigma_j(w_j(p) * q_j_hat) = (q0, q1) 

		q = q0            \ q = q0
		1/2 * t * q = q1  / t = 2q1 / q

		q -> R
		p -> t + Rp
		**************************************************************************************************************/

		Vec3d translation;
		Mat3d rotationMat;
		vector<Quaternion<double>> q0(numJoints), q1(numJoints);
	
		for (int i = 0; i < numJoints; i++) {

			// Convert RigidTransform R -> Quaternion q
			double R[9];
			rotationMat = jointSkinTransforms[i].getRotation();
			rotationMat.convertToArray(R);

			// Get translation t
			translation = jointSkinTransforms[i].getTranslation();

			// (q_i, 1/2 * t_i * q_i) =: q_i_hat
			Quaternion<double> t(0.0, translation[0], translation[1], translation[2]);
			q0[i] = q0[i].Matrix2Quaternion(R);
			q1[i] = (0.5 * t) * q0[i];
		}

		Vec3d newT, newVertexPositions;
		int index = 0;
		Quaternion<double> newQ, newQ0, newQ1;

		for (int i = 0; i < numMeshVertices; i++) {

			// q_hat(p) = Sigma_j(w_j(p) * q_j_hat)
			newQ0 = Quaternion<double>(0.0, 0.0, 0.0, 0.0);
			newQ1 = Quaternion<double>(0.0, 0.0, 0.0, 0.0);
			for (int j = 0; j < numJointsInfluencingEachVertex; j++) {
				index = i * numJointsInfluencingEachVertex + j;
				newQ0 = newQ0 + (meshSkinningWeights[index] * q0[meshSkinningJoints[index]]);
				newQ1 = newQ1 + (meshSkinningWeights[index] * q1[meshSkinningJoints[index]]);
			}

			// q = q0
			newQ = newQ0;

			// t = 2q1 / q
			newQ1 =  (newQ1 * 2.0) / newQ;
			newT = Vec3d(newQ1.Getx(), newQ1.Gety(), newQ1.Getz());

			//q -> R
			double R[9];
			newQ.Normalize();
			newQ.Quaternion2Matrix(R);
			Mat3d newR(R);

			//t + Rx
			newVertexPositions = newT + newR * Vec3d(restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2]);

			newMeshVertexPositions[3 * i + 0] = newVertexPositions[0];
			newMeshVertexPositions[3 * i + 1] = newVertexPositions[1];
			newMeshVertexPositions[3 * i + 2] = newVertexPositions[2];
		}
	}

	// The following below is just a dummy implementation.
	/*for (int i = 0; i < numMeshVertices; i++) {
		newMeshVertexPositions[3 * i + 0] = restMeshVertexPositions[3 * i + 0];
		newMeshVertexPositions[3 * i + 1] = restMeshVertexPositions[3 * i + 1];
		newMeshVertexPositions[3 * i + 2] = restMeshVertexPositions[3 * i + 2];
	}*/
}

