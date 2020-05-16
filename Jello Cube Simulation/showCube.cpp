/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"
#include <algorithm> 
#include <vector>
#include <utility>

using namespace std;

int pointMap(int side, int i, int j) {
	int r;

	switch (side) {
	case 1: //[i][j][0] bottom face
		r = 64 * i + 8 * j;
		break;
	case 6: //[i][j][7] top face
		r = 64 * i + 8 * j + 7;
		break;
	case 2: //[i][0][j] front face
		r = 64 * i + j;
		break;
	case 5: //[i][7][j] back face
		r = 64 * i + 56 + j;
		break;
	case 3: //[0][i][j] left face
		r = 8 * i + j;
		break;
	case 4: //[7][i][j] right face
		r = 448 + 8 * i + j;
		break;
	}

	return r;
}

void showCube(struct world * jello) {
	int i, j, k, ip, jp, kp;
	point r1, r2, r3; // aux variables

	/* normals buffer and counter for Gourad shading*/
	struct point normal[8][8];
	int counter[8][8];

	int face;
	double faceFactor, length;

	if (fabs(jello->p[0][0][0].x) > 10) {
		printf("Your cube somehow escaped way out of the box.\n");
		exit(0);
	}

#define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))


#define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\


	if (viewingMode == 0) { // render wireframe
		glLineWidth(1);
		glPointSize(5);
		glDisable(GL_LIGHTING);
		for (i = 0; i <= 7; i++) {
			for (j = 0; j <= 7; j++) {
				for (k = 0; k <= 7; k++) {
					if (i*j*k*(7 - i)*(7 - j)*(7 - k) != 0) // not surface point
						continue;

					glBegin(GL_POINTS); // draw point
					glColor4f(0, 0, 0, 0);
					glVertex3f(jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z);
					glEnd();

					//
					//if ((i!=7) || (j!=7) || (k!=7))
					//  continue;

					glBegin(GL_LINES);
					// structural
					if (structural == 1) {
						glColor4f(0, 0, 1, 1);
						PROCESS_NEIGHBOUR(1, 0, 0);
						PROCESS_NEIGHBOUR(0, 1, 0);
						PROCESS_NEIGHBOUR(0, 0, 1);
						PROCESS_NEIGHBOUR(-1, 0, 0);
						PROCESS_NEIGHBOUR(0, -1, 0);
						PROCESS_NEIGHBOUR(0, 0, -1);
					}

					// shear
					if (shear == 1) {
						glColor4f(0, 1, 0, 1);
						PROCESS_NEIGHBOUR(1, 1, 0);
						PROCESS_NEIGHBOUR(-1, 1, 0);
						PROCESS_NEIGHBOUR(-1, -1, 0);
						PROCESS_NEIGHBOUR(1, -1, 0);
						PROCESS_NEIGHBOUR(0, 1, 1);
						PROCESS_NEIGHBOUR(0, -1, 1);
						PROCESS_NEIGHBOUR(0, -1, -1);
						PROCESS_NEIGHBOUR(0, 1, -1);
						PROCESS_NEIGHBOUR(1, 0, 1);
						PROCESS_NEIGHBOUR(-1, 0, 1);
						PROCESS_NEIGHBOUR(-1, 0, -1);
						PROCESS_NEIGHBOUR(1, 0, -1);

						PROCESS_NEIGHBOUR(1, 1, 1)
						PROCESS_NEIGHBOUR(-1, 1, 1)
						PROCESS_NEIGHBOUR(-1, -1, 1)
						PROCESS_NEIGHBOUR(1, -1, 1)
						PROCESS_NEIGHBOUR(1, 1, -1)
						PROCESS_NEIGHBOUR(-1, 1, -1)
						PROCESS_NEIGHBOUR(-1, -1, -1)
						PROCESS_NEIGHBOUR(1, -1, -1)
					}

					// bend
					if (bend == 1) {
						glColor4f(1, 0, 0, 1);
						PROCESS_NEIGHBOUR(2, 0, 0);
						PROCESS_NEIGHBOUR(0, 2, 0);
						PROCESS_NEIGHBOUR(0, 0, 2);
						PROCESS_NEIGHBOUR(-2, 0, 0);
						PROCESS_NEIGHBOUR(0, -2, 0);
						PROCESS_NEIGHBOUR(0, 0, -2);
					}
					glEnd();
				}
			}
		}
		glEnable(GL_LIGHTING);
	}
	else
	{
		glPolygonMode(GL_FRONT, GL_FILL);

		for (face = 1; face <= 6; face++) {
			// face == face of a cube
			// 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top

			if ((face == 1) || (face == 3) || (face == 5))
				faceFactor = -1; // flip orientation
			else
				faceFactor = 1;


			for (i = 0; i <= 7; i++) { // reset buffers
				for (j = 0; j <= 7; j++) {
					normal[i][j].x = 0; normal[i][j].y = 0; normal[i][j].z = 0;
					counter[i][j] = 0;
				}
			}

			/* process triangles, accumulate normals for Gourad shading */

			for (i = 0; i <= 6; i++) {
				for (j = 0; j <= 6; j++) { // process block (i,j)

					pDIFFERENCE(NODE(face, i + 1, j), NODE(face, i, j), r1); // first triangle
					pDIFFERENCE(NODE(face, i, j + 1), NODE(face, i, j), r2);
					CROSSPRODUCTp(r1, r2, r3); pMULTIPLY(r3, faceFactor, r3);
					pNORMALIZE(r3);
					pSUM(normal[i + 1][j], r3, normal[i + 1][j]);
					counter[i + 1][j]++;
					pSUM(normal[i][j + 1], r3, normal[i][j + 1]);
					counter[i][j + 1]++;
					pSUM(normal[i][j], r3, normal[i][j]);
					counter[i][j]++;

					pDIFFERENCE(NODE(face, i, j + 1), NODE(face, i + 1, j + 1), r1); // second triangle
					pDIFFERENCE(NODE(face, i + 1, j), NODE(face, i + 1, j + 1), r2);
					CROSSPRODUCTp(r1, r2, r3); pMULTIPLY(r3, faceFactor, r3);
					pNORMALIZE(r3);
					pSUM(normal[i + 1][j], r3, normal[i + 1][j]);
					counter[i + 1][j]++;
					pSUM(normal[i][j + 1], r3, normal[i][j + 1]);
					counter[i][j + 1]++;
					pSUM(normal[i + 1][j + 1], r3, normal[i + 1][j + 1]);
					counter[i + 1][j + 1]++;
				}
			}


			/* the actual rendering */
			for (j = 1; j <= 7; j++) {

				if (faceFactor > 0)
					glFrontFace(GL_CCW); // the usual definition of front face
				else
					glFrontFace(GL_CW); // flip definition of orientation

				glBegin(GL_TRIANGLE_STRIP);
				for (i = 0; i <= 7; i++) {
					glNormal3f(normal[i][j].x / counter[i][j], normal[i][j].y / counter[i][j],
						normal[i][j].z / counter[i][j]);
					glVertex3f(NODE(face, i, j).x, NODE(face, i, j).y, NODE(face, i, j).z);
					glNormal3f(normal[i][j - 1].x / counter[i][j - 1], normal[i][j - 1].y / counter[i][j - 1],
						normal[i][j - 1].z / counter[i][j - 1]);
					glVertex3f(NODE(face, i, j - 1).x, NODE(face, i, j - 1).y, NODE(face, i, j - 1).z);
				}
				glEnd();
			}


		}
	} // end for loop over faces
	glFrontFace(GL_CCW);
}

void showBoundingBox() {
	int i, j;

	glColor4f(0.6, 0.6, 0.6, 0);

	glBegin(GL_LINES);

	// front face
	for (i = -2; i <= 2; i++) {
		glVertex3f(i, -2, -2);
		glVertex3f(i, -2, 2);
	}
	for (j = -2; j <= 2; j++) {
		glVertex3f(-2, -2, j);
		glVertex3f(2, -2, j);
	}

	// back face
	for (i = -2; i <= 2; i++) {
		glVertex3f(i, 2, -2);
		glVertex3f(i, 2, 2);
	}
	for (j = -2; j <= 2; j++) {
		glVertex3f(-2, 2, j);
		glVertex3f(2, 2, j);
	}

	// left face
	for (i = -2; i <= 2; i++) {
		glVertex3f(-2, i, -2);
		glVertex3f(-2, i, 2);
	}
	for (j = -2; j <= 2; j++) {
		glVertex3f(-2, -2, j);
		glVertex3f(-2, 2, j);
	}

	// right face
	for (i = -2; i <= 2; i++) {
		glVertex3f(2, i, -2);
		glVertex3f(2, i, 2);
	}
	for (j = -2; j <= 2; j++) {
		glVertex3f(2, -2, j);
		glVertex3f(2, 2, j);
	}

	glEnd();

	return;
}


point findCentroid(vector<point> points) {
	point center = { 0.0, 0.0, 0.0 };
	for (point p : points) {
		pSUM(center, p, center);
	}
	pDIVIDE(center, points.size(), center);
	return center;
}

vector<point> sortVerticies(vector<point> points, point normal) {
	// get centroid
	point center;
	center = findCentroid(points);

	point v;
	double length;
	pDIFFERENCE(points[0], center, v);
	pNORMALIZE(v);
	point u;
	CROSSPRODUCTp(v, normal, u);
	pNORMALIZE(u);

	vector< pair<double, int> > angles;
	for (int i = 0; i < points.size(); i++) {
		double pu, pv;
		DOTPRODUCT(points[i], u, pu);
		DOTPRODUCT(points[i], v, pv);
		pair<double, int> degree = make_pair(atan2(pu, pv), i);
		angles.push_back(degree);
	}
	sort(angles.begin(), angles.end());
	vector<point> sorted;
	for (pair<double, int> p : angles) {
		sorted.push_back(points[p.second]);
	}
	return sorted;
}


void showPlane(struct world * jello) {
	if (jello->incPlanePresent && plane) {
		// Inclined plane has equation a * x + b * y + c * z + d = 0;
		double a = jello->a, b = jello->b, c = jello->c, d = jello->d;
		vector<point> intersection; // 12 Edges
		// check 12 possible intersections
		// Back z = -2
		point current = { -2.0, -(a * -2.0 + c * -2.0 + d) / b , -2.0 };	// x = -2
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.y >= -2 && current.y <= 2) { intersection.push_back(current); }
		}

		//current = { 2.0, -(a * 2.0 + c * -2.0 + d) / b , -2.0 };			// x = 2
		current.x = 2.0;
		current.y = -(a * 2.0 + c * -2.0 + d) / b;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.y >= -2 && current.y <= 2) { intersection.push_back(current); }
		}

		//current = { -(b * -2.0 + c * -2.0 + d) / a, -2.0 , -2.0 };		// y = -2
		current.x = -(b * -2.0 + c * -2.0 + d) / a;
		current.y = -2.0;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.x >= -2 && current.x <= 2) { intersection.push_back(current); }
		}

		//current = { -(b * 2.0 + c * -2.0 + d) / a, 2.0 , -2.0 };			// y = 2
		current.x = -(b * 2.0 + c * -2.0 + d) / a;
		current.y = 2.0;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.x >= -2 && current.x <= 2) { intersection.push_back(current); }
		}

		// Front z = 2
		//current = { -2.0, -(a * -2.0 + c * 2.0 + d) / b , 2.0 };			// x = -2
		current.z = 2.0;
		current.x = -2.0;
		current.y = -(a * -2.0 + c * 2.0 + d) / b;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.y >= -2 && current.y <= 2) { intersection.push_back(current); }
		}

		//current = { 2.0, -(a * 2.0 + c * 2.0 + d) / b , 2.0 };			// x = 2
		current.x = 2.0;
		current.y = -(a * 2.0 + c * 2.0 + d) / b;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.y >= -2 && current.y <= 2) { intersection.push_back(current); }
		}

		//current = { -(b * -2.0 + c * 2.0 + d) / a, -2.0 , 2.0 };		// y = -2
		current.x = -(b * -2.0 + c * 2.0 + d) / a;
		current.y = -2.0;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.x >= -2 && current.x <= 2) { intersection.push_back(current); }
		}

		//current = { -(b * 2.0 + c * 2.0 + d) / a, 2.0 , 2.0 };			// y = 2
		current.x = -(b * 2.0 + c * 2.0 + d) / a;
		current.y = 2.0;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.x >= -2 && current.x <= 2) { intersection.push_back(current); }
		}

		// Middle y = -2
		//current = { -2.0, -2.0 , -(a * -2.0 + b * -2.0 + d) / c };		// x = -2
		current.y = -2.0;
		current.x = -2.0;
		current.z = -(a * -2.0 + b * -2.0 + d) / c;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.z >= -2 && current.z <= 2) { intersection.push_back(current); }
		}

		//current = { 2.0, -2.0 , -(a * 2.0 + b * -2.0 + d) / c };		// x = 2
		current.x = 2.0;
		current.z = -(a * 2.0 + b * -2.0 + d) / c;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.z >= -2 && current.z <= 2) { intersection.push_back(current); }
		}

		// Middle y = 2
		//current = { -2.0, 2.0 , -(a * -2.0 + b * 2.0 + d) / c };			// x = -2
		current.y = 2.0;
		current.x = -2.0;
		current.z = -(a * -2.0 + b * 2.0 + d) / c;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.z >= -2 && current.z <= 2) { intersection.push_back(current); }
		}

		//current = { 2.0, 2.0 , -(a * 2.0 + b * 2.0 + d) / c };			// x = 2
		current.x = 2.0;
		current.z = -(a * 2.0 + b * 2.0 + d) / c;
		if (find(intersection.begin(), intersection.end(), current) == intersection.end()) {
			if (current.z >= -2 && current.z <= 2) { intersection.push_back(current); }
		}


		glDisable(GL_CULL_FACE);	// To color both faces
		// there are at most 6 intersections in a cube
		if (intersection.size() == 3) {
			if (tex) {
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				glEnable(GL_TEXTURE_2D);
				glBegin(GL_TRIANGLES);
				glTexCoord2f(0.0, 0.0);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glTexCoord2f(1.0, 0.0);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glTexCoord2f(0.5, 1.0);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glEnd();
				glDisable(GL_TEXTURE_2D);
			}
			else {
				glBegin(GL_TRIANGLES);
				glColor4f(0, 0, 1, 1);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glColor4f(0, 1, 0, 1);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glColor4f(1, 0, 0, 1);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glEnd();
			}
		}
		else if(intersection.size() == 4) {
			point normal = { a, b, c };
			intersection = sortVerticies(intersection, normal);
			if (tex) {
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				glEnable(GL_TEXTURE_2D);
				glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glTexCoord2f(1.0, 0.0);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glTexCoord2f(1.0, 1.0);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glTexCoord2f(0.0, 1.0);
				glVertex3f(intersection[3].x, intersection[3].y, intersection[3].z);
				glEnd();
				glDisable(GL_TEXTURE_2D);
			}
			else {
				glBegin(GL_QUADS);
				glColor4f(0, 0, 1, 1);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glColor4f(0, 1, 0, 1);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glColor4f(1, 0, 0, 1);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glColor4f(1, 1, 0, 1);
				glVertex3f(intersection[3].x, intersection[3].y, intersection[3].z);
				glEnd();
			}
		}
		else if (intersection.size() == 5) {
			point normal = { a, b, c };
			intersection = sortVerticies(intersection, normal);
			if (tex) {
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				glEnable(GL_TEXTURE_2D);
				glBegin(GL_POLYGON);
				glTexCoord2f(0.5, 1.0);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glTexCoord2f(0.98, 0.65);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glTexCoord2f(0.8, 0.1);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glTexCoord2f(0.21, 0.1);
				glVertex3f(intersection[3].x, intersection[3].y, intersection[3].z);
				glTexCoord2f(0.02, 0.65);
				glVertex3f(intersection[4].x, intersection[4].y, intersection[4].z);
				glEnd();
				glDisable(GL_TEXTURE_2D);
			}
			else {
				glBegin(GL_POLYGON);
				glColor4f(0, 0, 1, 1);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glColor4f(0, 1, 0, 1);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glColor4f(1, 0, 0, 1);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glColor4f(1, 1, 0, 1);
				glVertex3f(intersection[3].x, intersection[3].y, intersection[3].z);
				glColor4f(0, 1, 1, 1);
				glVertex3f(intersection[4].x, intersection[4].y, intersection[4].z);
				glEnd();
			}
		}
		else if (intersection.size() == 6) {
			point normal = { a, b, c };
			intersection = sortVerticies(intersection, normal);
			if (tex) {
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				glEnable(GL_TEXTURE_2D);
				glBegin(GL_POLYGON);
				glTexCoord2f(1.0, 0.5);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glTexCoord2f(0.75, 0.07);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glTexCoord2f(0.25, 0.07);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glTexCoord2f(0.0, 0.5);
				glVertex3f(intersection[3].x, intersection[3].y, intersection[3].z);
				glTexCoord2f(0.25, 0.93);
				glVertex3f(intersection[4].x, intersection[4].y, intersection[4].z);
				glTexCoord2f(0.75, 0.93);
				glVertex3f(intersection[5].x, intersection[5].y, intersection[5].z);
				glEnd();
				glDisable(GL_TEXTURE_2D);
			}
			else {
				glBegin(GL_POLYGON);
				glColor4f(0, 0, 1, 1);
				glVertex3f(intersection[0].x, intersection[0].y, intersection[0].z);
				glColor4f(0, 1, 0, 1);
				glVertex3f(intersection[1].x, intersection[1].y, intersection[1].z);
				glColor4f(1, 1, 0, 1);
				glVertex3f(intersection[2].x, intersection[2].y, intersection[2].z);
				glColor4f(1, 0, 0, 1);
				glVertex3f(intersection[3].x, intersection[3].y, intersection[3].z);
				glColor4f(1, 0, 1, 1);
				glVertex3f(intersection[4].x, intersection[4].y, intersection[4].z);
				glColor4f(0, 1, 1, 1);
				glVertex3f(intersection[5].x, intersection[5].y, intersection[5].z);
				glEnd();
			}
		}
		glEnable(GL_CULL_FACE);
	}
}

void showSkyBox(GLuint * skybox) {
	if (env) {
		glDisable(GL_CULL_FACE);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glEnable(GL_TEXTURE_2D);

		glBindTexture(GL_TEXTURE_2D, skybox[0]);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-10.0, 10.0, -10.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(10.0, 10.0, -10.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(10.0, 10.0, 10.0);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-10.0, 10.0, 10.0);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, skybox[1]);
		glBegin(GL_QUADS);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(-10.0, -10.0, -10.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(10.0, -10.0, -10.0);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(10.0, -10.0, 10.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(-10.0, -10.0, 10.0);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, skybox[2]);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-10.0, -10.0, -10.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(-10.0, 10.0, -10.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(-10.0, 10.0, 10.0);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-10.0, -10.0, 10.0);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, skybox[3]);
		glBegin(GL_QUADS);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(10.0, -10.0, -10.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(10.0, 10.0, -10.0);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(10.0, 10.0, 10.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(10.0, -10.0, 10.0);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, skybox[4]);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-10.0, -10.0, 10.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(10.0, -10.0, 10.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(10.0, 10.0, 10.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-10.0, 10.0, 10.0);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, skybox[5]);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-10.0, -10.0, -10.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(10.0, -10.0, -10.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(10.0, 10.0, -10.0);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-10.0, 10.0, -10.0);
		glEnd();

		glDisable(GL_TEXTURE_2D);
		glEnable(GL_CULL_FACE);
	}
}