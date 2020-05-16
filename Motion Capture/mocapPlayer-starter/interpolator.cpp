#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <time.h>

Interpolator::Interpolator() {
	//Set default interpolation type
	m_InterpolationType = LINEAR;

	//set default angle representation to use for interpolation
	m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator() {
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N, int T) {
	//Allocate new motion
	*pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton());

	srand(time(NULL));
	clock_t start, end;
	char *interType;
	//Perform the interpolation
	if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER)) {
		interType = "LINEAR EULER";
		start = clock();
		LinearInterpolationEuler(pInputMotion, *pOutputMotion, N, T);
		end = clock();
	}
	else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION)) {
		interType = "LINEAR QUATERNION";
		start = clock();
		LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N, T);
		end = clock();
	}
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER)) {
		interType = "BEZIER EULER";
		start = clock();
		BezierInterpolationEuler(pInputMotion, *pOutputMotion, N, T);
		end = clock();
	}
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION)) {
		interType = "BEZIER QUATERNION";
		start = clock();
		BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N, T);
		end = clock();
	}
	else {
		printf("Error: unknown interpolation / angle representation type.\n");
		exit(1);
	}
	
	printf("Interpolation Type: %s\n", interType);
	printf("Computation time: %lf\n", (double)(end - start) / (double)CLOCKS_PER_SEC);
	//system("pause");
}

/******************************************
 Linear Euler support non-uniform keyframes
******************************************/
void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N, int T) {
	int inputLength = pInputMotion->GetNumFrames(); // Frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0; // Initialize start keyframe
	int offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0;	// For non-uniform keyframes

	while (startKeyframe + (N + offset) + 1 < inputLength) {
		int endKeyframe = startKeyframe + (N + offset) + 1;

		// Obtain start and end posture
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// Copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// Interpolate in between
		for (int frame = 1; frame <= (N + offset); frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / ((N + offset) + 1);

			// Interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// Interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

			// Set posture of frame n with interpolated posture
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
		offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0; // Produce next offset to produce non-uniform keyframes
	}
	// Set remaining frames
	for (int frame = startKeyframe + 1; frame < inputLength; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

void Interpolator::Rotation2Euler(double R[9], double angles[3]) {
	double cy = sqrt(R[0] * R[0] + R[3] * R[3]); //sqrt(cos^2(£c)cos^2(£p) + cos^2(£c)sin^2(£p)) = cos(£c)

	if (cy > 16 * DBL_EPSILON) {		// If cos(£c) > 0
		angles[0] = atan2(R[7], R[8]);	// R[7] / R[8] = sin(£r)cos(£c) / cos(£r)cos(£c) = tan(£r)
		angles[1] = atan2(-R[6], cy);	// R[6] = -sin(£c)
		angles[2] = atan2(R[3], R[0]);	// R[3] / R[0] = cos(£c)sin(£p) / cos(£c)cos(£p) = tan(£p)
	}
	else {								// If cos(£c) = 0 => sin(£c) = 1 or -1
		angles[0] = atan2(-R[5], R[4]); // R[5] / R[4] = (cos(£r)sin(£p) - sin(£r)cos(£p)) / (sin(£r)sin(£p) + cos(£r)cos(£p)) = -sin(£r - £p) / cos(£r - £p) = -tan(£r - £p) = tan(£r)
										// or = (-cos(£r)sin(£p) - sin(£r)cos(£p)) / (-sin(£r)sin(£p) + cos(£r)cos(£p)) = -sin(£r + £p) / cos(£r + £p) = -tan(£r + £p) = tan(£r)
		angles[1] = atan2(-R[6], cy);	// R[6] = -sin(£c)
		angles[2] = 0;					// £p = 0
	}

	for (int i = 0; i < 3; i++) {
		angles[i] *= 180 / M_PI;		// convert angles to degree
	}
}

void Interpolator::MatrixMultiply(double A[3][3], double B[3][3], double C[3][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
}

void Interpolator::Euler2Rotation(double angles[3], double R[9]) {
	// students should implement this
	double psi = (double)(angles[0] / 180)* M_PI;
	double theta = (double)(angles[1] / 180)* M_PI;
	double phi = (double)(angles[2] / 180)* M_PI;

	double Rx[3][3] = {
		{1.0, 0.0, 0.0},
		{0.0, cos(psi), -sin(psi)},
		{0.0, sin(psi), cos(psi)}
	};

	double Ry[3][3] = {
		{cos(theta), 0.0, sin(theta)},
		{0.0, 1.0, 0.0},
		{-sin(theta), 0.0, cos(theta)}
	};

	double Rz[3][3] = {
		{cos(phi), -sin(phi), 0.0},
		{sin(phi), cos(phi), 0.0},
		{0.0, 0.0, 1.0}
	};

	//R = Rz(£p)Ry(£c)Rx(£r)
	double Rzy[3][3] = { }, Rotation[3][3] = { };
	MatrixMultiply(Rz, Ry, Rzy);
	MatrixMultiply(Rzy, Rx, Rotation);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[j + (3 * i)] = Rotation[i][j];
		}
	}
}

/******************************************
 Bezier Euler support non-uniform keyframes
******************************************/
void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N, int T) {
	// students should implement this
	int totalFrames = pInputMotion->GetNumFrames(); // Frames are indexed 0, ..., inputLength-1
	
	int startKeyframe = 0;	// Initialize start keyframe
	int offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0;			// For non-uniform keyframes
	int next_offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0;	// Initialize offset for next keyframe
	int prev_offset = 0;	// Initialize offset for previous keyframe
	vector p0, p1, p2, p3;	// Control points

	while (startKeyframe + (N + offset) + 1 < totalFrames) {
		int prevKeyframe = startKeyframe - (N + prev_offset) - 1;
		int endKeyframe = startKeyframe + (N + offset) + 1;
		int nextKeyframe = endKeyframe + (N + next_offset) + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		vector an, bn;

		// interpolate in between
		for (int frame = 1; frame <= (N + offset); frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / ((N + offset) + 1);
			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			/****************************************************
			If total of N frames
			-----------------------------------------------------
			a1 = (p1, (p3, p2, 2.0), 1.0 / 3.0)
			   = p1 + ((p3, p2, 2.0) - p1) / 3.0
			   = p1 + (p3 + (p2 - p3) * 2) - p1) / 3.0
			   = p1 + (p2 * 2.0 - p3 - p1) / 3.0
			an = (pn , an_bar , 1.0 / 3.0)
			   = pn + (pn+1 - (pn * 2 - pn-1) * 0.5 - pn) / 3.0;
			-----------------------------------------------------
			(pn-1, pn, 2.0) = pn * 2 - pn-1
			an_bar = ((pn-1, pn, 2.0), pn+1, 0.5)
			   = ((pn * 2 - pn-1), pn+1, 0.5)
			   = (pn * 2 - pn-1) + (pn+1 - (pn * 2 - pn-1) * 0.5)
			   = pn+1 - (pn * 2 - pn-1) * 0.5
			-----------------------------------------------------
			bn = (pn , an_bar , -1.0 / 3.0)
			   = pn - (pn+1 - (pn * 2 - pn-1) * 0.5 - pn) / 3.0;
			bN = (pN, (pN-2, pN-1, 2.0), 1.0 / 3.0)
			   = pN + ((pN-2, pN-1, 2.0) - pN) / 3.0
			   = pN + (pN-2 + (pN-1 - pN-2) * 2.0 - pN) / 3.0
			   = pN + (pN-1 * 2.0 - pN-2 - pN) / 3.0
			****************************************************/

			if (startKeyframe == 0) { // If first keyframe
				Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
				p3 = nextPosture->root_pos;
				an = p1 + (p2 * 2.0 - p3 - p1) / 3.0;
				bn = p2 + ((p2 * 2.0 - p1 + p3) * 0.5 - p2) / 3.0;
			}
			else {
				Posture * prevPosture = pInputMotion->GetPosture(prevKeyframe);
				p0 = prevPosture->root_pos;

				if (nextKeyframe <= totalFrames) { // If keyframes in between
					Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
					p3 = nextPosture->root_pos;
					an = p1 + ((p1 * 2.0 - p0 + p2) * 0.5 - p1) / 3.0;
					bn = p2 - ((p2 * 2.0 - p1 + p3) * 0.5 - p2) / 3.0;
				}
				else { // If last keyframe
					an = p1 + ((p1 * 2.0 - p0 + p2) * 0.5 - p1) / 3.0;
					bn = p2 + (p1 * 2.0 - p0 - p2) / 3.0;
				}
			}
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, an, bn, p2);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {

				p1 = startPosture->bone_rotation[bone];
				p2 = endPosture->bone_rotation[bone];

				if (startKeyframe == 0) { // If first keyframe
					Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
					p3 = nextPosture->bone_rotation[bone];
					an = p1 + (p2 * 2.0 - p3 - p1) / 3.0;
					bn = p2 + ((p2 * 2.0 - p1 + p3) * 0.5 - p2) / 3.0;
				}
				else {
					Posture * prevPosture = pInputMotion->GetPosture(prevKeyframe);
					p0 = prevPosture->bone_rotation[bone];

					if (nextKeyframe <= totalFrames) { // If keyframes in between
						Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
						p3 = nextPosture->bone_rotation[bone];
						an = p1 + ((p1 * 2.0 - p0 + p2) * 0.5 - p1) / 3.0;
						bn = p2 - ((p2 * 2.0 - p1 + p3) * 0.5 - p2) / 3.0;
					}
					else { // If last keyframe
						an = p1 + ((p1 * 2.0 - p0 + p2) * 0.5 - p1) / 3.0;
						bn = p2 + (p1 * 2.0 - p0 - p2) / 3.0;
					}
				}
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, an, bn, p2);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		prev_offset = offset; // Track offset for previous frame 
		offset = next_offset; // Track offset for next frame
		startKeyframe = endKeyframe;
		next_offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0; // Produce next offset to produce non-uniform keyframes
	}
	for (int frame = startKeyframe + 1; frame < totalFrames; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

/******************************************
 Linear SLERP support non-uniform keyframes
******************************************/
void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N, int T) {
	// students should implement this
	int totalFrames = pInputMotion->GetNumFrames(); // Frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0; // Initialize start keyframe
	int offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0;	// For non-uniform keyframes
	Quaternion<double> slerping; // Temp for computing SLERP

	while (startKeyframe + (N + offset) + 1 < totalFrames) {
		int endKeyframe = startKeyframe + (N + offset) + 1;

		// Obtain start and end posture
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// Copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// Interpolate in between
		for (int frame = 1; frame <= (N + offset); frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / ((N + offset) + 1);

			// Interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// Interpolate bone rotations
			Quaternion<double> startBone, endBone;
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				Euler2Quaternion(startPosture->bone_rotation[bone].p, startBone);	// Obtain Start Quaternion
				Euler2Quaternion(endPosture->bone_rotation[bone].p, endBone);		// Obtain End Quaternion
				slerping = Slerp(t, startBone, endBone);
				Quaternion2Euler(slerping, interpolatedPosture.bone_rotation[bone].p); // Obtain Euler Angles
			}
			// Set posture of frame n with interpolated posture
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
		offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0; // Produce next offset to produce non-uniform keyframes
	}
	// Set posture of remaining frames
	for (int frame = startKeyframe + 1; frame < totalFrames; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

/************
 Bezier SLERP
************/
void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N, int T) {
	// students should implement this
	int totalFrames = pInputMotion->GetNumFrames(); // Frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0; // Initialize start keyframe
	int offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0; // For non-uniform keyframes
	int next_offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0; // Initialize offset for next keyframe
	int prev_offset = 0; // Initialize offset for previous keyframe

	vector p0, p1, p2, p3; // Control points
	Quaternion<double> q0, q1, q2, q3; // Control points for quaternions
	Quaternion<double> doubling, slerping;  // Temp for computing Double and SLERP
	int error = (T != 0 ) ? (rand() % (T * 2 + 1) - T) : 0;

	while (startKeyframe + (N + offset) + 1 < totalFrames) {
		int prevKeyframe = startKeyframe - (N + prev_offset) - 1;
		int endKeyframe = startKeyframe + (N + offset) + 1;
		int nextKeyframe = endKeyframe + (N + next_offset) + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		vector an, bn;
		Quaternion<double> qan, qbn;

		// interpolate in between
		for (int frame = 1; frame <= (N + offset); frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / ((N + offset) + 1);
			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			/****************************************************
			If total of N frames
			-----------------------------------------------------
			a1 = (p1, (p3, p2, 2.0), 1.0 / 3.0)
			   = p1 + ((p3, p2, 2.0) - p1) / 3.0
			   = p1 + (p3 + (p2 - p3) * 2) - p1) / 3.0
			   = p1 + (p2 * 2.0 - p3 - p1) / 3.0
			an = (pn , an_bar , 1.0 / 3.0)
			   = pn + (pn+1 - (pn * 2 - pn-1) * 0.5 - pn) / 3.0;
			-----------------------------------------------------
			(pn-1, pn, 2.0) = pn * 2 - pn-1
			an_bar = ((pn-1, pn, 2.0), pn+1, 0.5)
			   = ((pn * 2 - pn-1), pn+1, 0.5)
			   = (pn * 2 - pn-1) + (pn+1 - (pn * 2 - pn-1) * 0.5)
			   = pn+1 - (pn * 2 - pn-1) * 0.5
			-----------------------------------------------------
			bn = (pn , an_bar , -1.0 / 3.0)
			   = pn - (pn+1 - (pn * 2 - pn-1) * 0.5 - pn) / 3.0;
			bN = (pN, (pN-2, pN-1, 2.0), 1.0 / 3.0)
			   = pN + ((pN-2, pN-1, 2.0) - pN) / 3.0
			   = pN + (pN-2 + (pN-1 - pN-2) * 2.0 - pN) / 3.0
			   = pN + (pN-1 * 2.0 - pN-2 - pN) / 3.0
			****************************************************/

			if (startKeyframe == 0) { // If first keyframe
				Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
				p3 = nextPosture->root_pos;
				an = p1 + (p2 * 2.0 - p3 - p1) / 3.0;
				bn = p2 + ((p2 * 2.0 - p1 + p3) * 0.5 - p2) / 3.0;
			}
			else {
				Posture * prevPosture = pInputMotion->GetPosture(prevKeyframe);
				p0 = prevPosture->root_pos;

				if (nextKeyframe <= totalFrames) { // If keyframes in between
					Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
					p3 = nextPosture->root_pos;
					an = p1 + ((p1 * 2.0 - p0 + p2) * 0.5 - p1) / 3.0;
					bn = p2 - ((p2 * 2.0 - p1 + p3) * 0.5 - p2) / 3.0;
				}
				else { // If last keyframe
					an = p1 + ((p1 * 2.0 - p0 + p2) * 0.5 - p1) / 3.0;
					bn = p2 + (p1 * 2.0 - p0 - p2) / 3.0;
				}
			}
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, an, bn, p2);

			/****************************************************
			If total of N frames
			-----------------------------------------------------
			a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1.0 / 3.0)
			an = Slerp(qn , an_bar , 1.0 / 3)
			-----------------------------------------------------
			an_bar = Slerp(Slerp(qn-1, qn, 2.0), qn+1, 0.5)
			-----------------------------------------------------
			bn =  Slerp(qn , an_bar , -1.0 / 3)
			bN = Slerp(qN, Slerp(qN-2, qN-1, 2.0), 1.0 / 3.0)
			****************************************************/

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);

				if (startKeyframe == 0) { // If first keyframe
					Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
					Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
					doubling = Double(q3, q2);
					qan = Slerp((1.0 / 3.0), q1, doubling);

					doubling = Double(q1, q2);
					slerping = Slerp(0.5, doubling, q3);
					qbn = Slerp((-1.0 / 3.0), q2, slerping);
				}
				else {
					Posture * prevPosture = pInputMotion->GetPosture(prevKeyframe);
					Euler2Quaternion(prevPosture->bone_rotation[bone].p, q0);

					if (nextKeyframe <= totalFrames) { // If keyframes in between 
						Posture * nextPosture = pInputMotion->GetPosture(nextKeyframe);
						Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
						doubling = Double(q0, q1);
						slerping = Slerp(0.5, doubling, q2);
						qan = Slerp((1.0 / 3.0), q1, slerping);

						doubling = Double(q1, q2);
						slerping = Slerp(0.5, doubling, q3);
						qbn = Slerp((-1.0 / 3.0), q2, slerping);
					}
					else { // If last keyframe
						doubling = Double(q0, q1);
						slerping = Slerp(0.5, doubling, q2);
						qan = Slerp((1.0 / 3.0), q1, slerping);

						doubling = Double(q0, q1);
						qbn = Slerp((1.0 / 3.0), q2, doubling);
					}
				}
				Quaternion<double> deCasteljau = DeCasteljauQuaternion(t, q1, qan, qbn, q2);
				Quaternion2Euler(deCasteljau, interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		prev_offset = offset; // Track offset for previous frame 
		offset = next_offset; // Track offset for next frame
		startKeyframe = endKeyframe;
		next_offset = (T != 0) ? (rand() % (T * 2 + 1) - T) : 0; // Produce next offset to produce non-uniform keyframes
	}
	for (int frame = startKeyframe + 1; frame < totalFrames; frame++) {
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) {
	// students should implement this
	double Rotation[9];
	Euler2Rotation(angles, Rotation);					// Convert Euler Angles to Rotation Matrix
	q = Quaternion<double>::Matrix2Quaternion(Rotation);// Convert Rotation Matrix to Quaternion
	q.Normalize();										// Normalize Quaternion
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) {
	// students should implement this
	double Rotation[9];
	q.Quaternion2Matrix(Rotation);		// Convert Quaternion to Rotation Matrix
	Rotation2Euler(Rotation, angles);	// Convert Rotation Matrix to Euler Angles
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_) {
	// students should implement this
	Quaternion<double> result;

	// cos(£c) = q1 . q2 = s1s2 + x1x2 + y1y2 + z1z2
	double cosine = (qStart.Gets() * qEnd_.Gets()) + 
		(qStart.Getx() * qEnd_.Getx()) + 
		(qStart.Gety() * qEnd_.Gety()) + 
		(qStart.Getz() * qEnd_.Getz());

	// Equation: (sin((1 - u)£c) / sin(£c) * q1) + (sin(u£c) / sin(£c) * q2)
	// Any rotation is given by two quaternions, so there are two SLERP choices; pick the shortest
	if (cosine < 0) {
		cosine = -cosine;
		qEnd_ = -1.0 * qEnd_;
	}

	double theta = acos(cosine); // Obtain £c
	double sine = sin(theta);    // Obtain sin(£c)

	if (sine == 0.0) {
		// If start point is end point. no need to interpolate
		result = qStart;
	}
	else {
		// Else interpolate with equation = (sin((1 - u)£c) * q1 + sin(u£c) * q2) / sin(£c)
		result = (sin((1 - t) * theta) * qStart + sin(t * theta) * qEnd_) / sin(theta);
		result.Normalize();
	}
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q) {
	// students should implement this
	Quaternion<double> result;
	result = Slerp(2.0, p, q); // Slerp with 2.0 = Doubling the length
	return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3) {
	// students should implement this
	// PPT p.30
	vector p;
	vector q0 = p0 * (1 - t) + p1 * t;  // Q0 = P0(1 - t) + P1(t)
	vector q1 = p1 * (1 - t) + p2 * t;  // Q1 = P1(1 - t) + P2(t)
	vector q2 = p2 * (1 - t) + p3 * t;  // Q2 = P2(1 - t) + P3(t)
	vector r0 = q0 * (1 - t) + q1 * t;  // R0 = Q0(1 - t) + Q1(t)
	vector r1 = q1 * (1 - t) + q2 * t;  // R1 = Q1(1 - t) + Q2(t)
	p = r0 * (1 - t) + r1 * t;			// P(t) = R0(1 - t) + R1(t)
	return p;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3) {
	// students should implement this
	// PPT p.31
	Quaternion<double> p;
	Quaternion<double> q0 = Slerp(t, p0, p1); // Q0 = Slerp(P0,P1,t)
	Quaternion<double> q1 = Slerp(t, p1, p2); // Q1 = Slerp(P1,P2,t)
	Quaternion<double> q2 = Slerp(t, p2, p3); // Q2 = Slerp(P2,P3,t)
	Quaternion<double> r0 = Slerp(t, q0, q1); // R0 = Slerp(Q0,Q1,t)
	Quaternion<double> r1 = Slerp(t, q1, q2); // R1 = Slerp(Q1,Q2,t)
	p = Slerp(t, r0, r1);					  // P(t)= Slerp(R0,R1,t)
	return p;
}

