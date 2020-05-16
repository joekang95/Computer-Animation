/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

double R_STRUCT = 1.0 / 7.0;	// Rest length for structual force 1 meter / (8 vertices - 1) gaps
double R_SHEAR = R_STRUCT * sqrt(2.0);	// Rest length for shear force on same face (1 : 1 : sqrt(2))
double R_SHEAR_LONG = R_STRUCT * sqrt(3.0); // Rest length for shear force on different face
double R_BEND = 2.0 / 7.0;		// Rest length for bend force

/****************************************************
Function force
Used to caculate the total force in a kind of spring
Total force = Hook force + Damping Force
Spring kind is determined by rest (rest length)
R_STRUCT: Structual Force
R_SHEAR, R_SHEAR_LONG: Shear Force
R_BEND: Bend Force
****************************************************/
void force(point pa, point pb, double Kh, point va, point vb, double Kd, double rest, point& a) {
	/*****************************************
	Hook's Law : F = -k(|L| - R) x L/|L|
	L: vector pointing from B to A
	R: spring rest length
	------------------------------------------
	Damping : F = -k(|va -vb|.L)/|L| x L/|L|
	L: vector pointing from B to A
	*****************************************/
	double length = 0.0;
	point L, H = { 0.0, 0.0, 0.0 };
	pDIFFERENCE(pa, pb, L);					// L
	pNORMALIZE(L);							// L/|L|
	double hook = -Kh * (length - rest);	// -k(|L| - R)
	pMULTIPLY(L, hook, H);					// -k(|L| - R) x L/|L|
	pSUM(H, a, a);

	double dot = 0.0;
	point V, D = { 0.0, 0.0, 0.0 };
	pDIFFERENCE(pa, pb, L);					// L
	pDIFFERENCE(va, vb, V);					// |va -vb|
	DOTPRODUCT(V, L, dot);					// |va -vb|.L
	pNORMALIZE(L);							// L/|L|
	double damp = -Kd * dot / length;		// -k(|va -vb|.L)/|L|
	pMULTIPLY(L, damp, D);					// -k(|va -vb|.L)/|L| x L/|L|
	pSUM(D, a, a);
}

/****************************************
Function structualForce
Compute structual force for a vertex
There are at most 6 directions for 3 axis
****************************************/
void structualForce(world *jello, int i, int j, int k, point& a) {

	double Kh = jello->kElastic;
	double Kd = jello->dElastic;
	point current = jello->p[i][j][k];
	point current_v = jello->v[i][j][k];
	point neighbor, neighbor_v;
	// if there is left neighbor
	if (i > 0) {
		neighbor = jello->p[i - 1][j][k];
		neighbor_v = jello->v[i - 1][j][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_STRUCT, a);
	}
	// if there is right neighbor
	if (i < 7) {
		neighbor = jello->p[i + 1][j][k];
		neighbor_v = jello->v[i + 1][j][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_STRUCT, a);
	}
	// if there is bottom neighbor
	if (j > 0) {
		neighbor = jello->p[i][j - 1][k];
		neighbor_v = jello->v[i][j - 1][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_STRUCT, a);
	}
	// if there is top neighbor
	if (j < 7) {
		neighbor = jello->p[i][j + 1][k];
		neighbor_v = jello->v[i][j + 1][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_STRUCT, a);
	}
	// if there is back neighbor
	if (k > 0) {
		neighbor = jello->p[i][j][k - 1];
		neighbor_v = jello->v[i][j][k - 1];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_STRUCT, a);
	}
	// if there is front neighbor
	if (k < 7) {
		neighbor = jello->p[i][j][k + 1];
		neighbor_v = jello->v[i][j][k + 1];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_STRUCT, a);
	}
}

/********************************************
Function shearForce
Compute shear force for a vertex
There are at most 20 directions for 5 faces
i +-, j+- | k+- : 2 * 2 * 2 = 8
i remain, j+-, k+-: 1 * 2 * 2 = 4
i +-, j+-, k+-: 2 * 2 * 2 = 8
********************************************/
void shearForce(world *jello, int i, int j, int k, point& a) {

	double Kh = jello->kElastic;
	double Kd = jello->dElastic;
	point current = jello->p[i][j][k];
	point current_v = jello->v[i][j][k];
	point neighbor, neighbor_v;

	// if there are left neighbors
	if (i > 0) {
		// if there are bottom-left neighbors
		if (j > 0) {
			neighbor = jello->p[i - 1][j - 1][k];
			neighbor_v = jello->v[i - 1][j - 1][k];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
			// if there is back-bottom-left neighbor
			if (k > 0) {
				neighbor = jello->p[i - 1][j - 1][k - 1];
				neighbor_v = jello->v[i - 1][j - 1][k - 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
			// if there is front-bottom-left neighbor
			if (k < 7) {
				neighbor = jello->p[i - 1][j - 1][k + 1];
				neighbor_v = jello->v[i - 1][j - 1][k + 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}

		}
		// if there are top-left neighbors
		if (j < 7) {
			neighbor = jello->p[i - 1][j + 1][k];
			neighbor_v = jello->v[i - 1][j + 1][k];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
			// if there is back-top-left neighbor
			if (k > 0) {
				neighbor = jello->p[i - 1][j + 1][k - 1];
				neighbor_v = jello->v[i - 1][j + 1][k - 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
			// if there is front-top-left neighbor
			if (k < 7) {
				neighbor = jello->p[i - 1][j + 1][k + 1];
				neighbor_v = jello->v[i - 1][j + 1][k + 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
		}
		// if there is back-left neighbor (back-bottom-left and back-top-left checked)
		if (k > 0) {
			neighbor = jello->p[i - 1][j][k - 1];
			neighbor_v = jello->v[i - 1][j][k - 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
		// if there is front-left neighbor (front-bottom-left and front-top-left checked)
		if (k < 7) {
			neighbor = jello->p[i - 1][j][k + 1];
			neighbor_v = jello->v[i - 1][j][k + 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
	}
	// if there are right neighbors
	if (i < 7) {
		// if there are bottom-right neighbors
		if (j > 0) {
			neighbor = jello->p[i + 1][j - 1][k];
			neighbor_v = jello->v[i + 1][j - 1][k];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
			// if there is back-bottom-right neighbor
			if (k > 0) {
				neighbor = jello->p[i + 1][j - 1][k - 1];
				neighbor_v = jello->v[i + 1][j - 1][k - 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
			// if there is front-bottom-right neighbor
			if (k < 7) {
				neighbor = jello->p[i + 1][j - 1][k + 1];
				neighbor_v = jello->v[i + 1][j - 1][k + 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
		}
		// if there are top-right neighbors
		if (j < 7) {
			neighbor = jello->p[i + 1][j + 1][k];
			neighbor_v = jello->v[i + 1][j + 1][k];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
			// if there is back-top-right neighbor
			if (k > 0) {
				neighbor = jello->p[i + 1][j + 1][k - 1];
				neighbor_v = jello->v[i + 1][j + 1][k - 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
			// if there is front-top-right neighbor
			if (k < 7) {
				neighbor = jello->p[i + 1][j + 1][k + 1];
				neighbor_v = jello->v[i + 1][j + 1][k + 1];
				force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR_LONG, a);
			}
		}
		// if there is back-right neighbor (back-bottom-right and back-top-right checked)
		if (k > 0) {
			neighbor = jello->p[i + 1][j][k - 1];
			neighbor_v = jello->v[i + 1][j][k - 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
		// if there is front-right neighbor (front-bottom-right and front-top-right checked)
		if (k < 7) {
			neighbor = jello->p[i + 1][j][k + 1];
			neighbor_v = jello->v[i + 1][j][k + 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
	}
	// if there are bottom neighbors (back-bottom-right/left and front-bottom-right/left checked)
	if (j > 0) {
		// if there is back-bottom neighbor 
		if (k > 0) {
			neighbor = jello->p[i][j - 1][k - 1];
			neighbor_v = jello->v[i][j - 1][k - 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
		// if there is front-bottom neighbor
		if (k < 7) {
			neighbor = jello->p[i][j - 1][k + 1];
			neighbor_v = jello->v[i][j - 1][k + 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
	}
	// if there are top neighbors (back-top-right/left and front-top-right/left checked)
	if (j < 7) {
		// if there is back-top neighbor 
		if (k > 0) {
			neighbor = jello->p[i][j + 1][k - 1];
			neighbor_v = jello->v[i][j + 1][k - 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
		// if there is front-top neighbor
		if (k < 7) {
			neighbor = jello->p[i][j + 1][k + 1];
			neighbor_v = jello->v[i][j + 1][k + 1];
			force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_SHEAR, a);
		}
	}
}

/****************************************
Function bendForce
Compute bend force for a vertex
There are at most 6 directions for 3 axis
****************************************/
void bendForce(world *jello, int i, int j, int k, point& a) {
	double Kh = jello->kElastic;
	double Kd = jello->dElastic;
	point current = jello->p[i][j][k];
	point current_v = jello->v[i][j][k];
	point neighbor, neighbor_v;
	// if there are left 2 neighbors 
	if (i > 1) {
		neighbor = jello->p[i - 2][j][k];
		neighbor_v = jello->v[i - 2][j][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_BEND, a);
	}
	// if there are right 2 neighbors 
	if (i < 6) {
		neighbor = jello->p[i + 2][j][k];
		neighbor_v = jello->v[i + 2][j][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_BEND, a);
	}
	// if there are bottom 2 neighbors 
	if (j > 1) {
		neighbor = jello->p[i][j - 2][k];
		neighbor_v = jello->v[i][j - 2][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_BEND, a);
	}
	// if there are top 2 neighbors 
	if (j < 6) {
		neighbor = jello->p[i][j + 2][k];
		neighbor_v = jello->v[i][j + 2][k];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_BEND, a);
	}
	// if there are back 2 neighbors 
	if (k > 1) {
		neighbor = jello->p[i][j][k - 2];
		neighbor_v = jello->v[i][j][k - 2];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_BEND, a);
	}
	// if there are front 2 neighbors 
	if (k < 6) {
		neighbor = jello->p[i][j][k + 2];
		neighbor_v = jello->v[i][j][k + 2];
		force(current, neighbor, Kh, current_v, neighbor_v, Kd, R_BEND, a);
	}
}

/***********************************************************
Function externalForce
Compute external force for a vertex
Need to first compute the actual position in cube dimension
since force field grid covers the entire scene bounding box
Then obtain the 8 nearest forces and 3D-interpolate them.
************************************************************/
void externalForce(world *jello, int i, int j, int k, point& a) {
	/*****************************************
	Bounding box: 4 x 4 x 4. Center: Origin
	x = -2 + 4 * (1.0 * i / (jello.resolution-1));
	i = (x + 2) / 4 *  (jello.resolution-1)
	*****************************************/

	// compute actual position
	int length = jello->resolution - 1, real_i, real_j, real_k;
	real_i = int((jello->p[i][j][k].x + 2) / 4 * length);
	real_j = int((jello->p[i][j][k].y + 2) / 4 * length);
	real_k = int((jello->p[i][j][k].z + 2) / 4 * length);

	// clamp position (negative shouldn't exist but in case)
	real_i = (real_i >= length) ? length - 1 : ((real_i < 0) ? 0 : real_i);
	real_j = (real_j >= length) ? length - 1 : ((real_j < 0) ? 0 : real_j);
	real_k = (real_k >= length) ? length - 1 : ((real_k < 0) ? 0 : real_k);


	// obtain forces of 8 nearest nodes
	// back face
	point BOTTOM_LEFT = jello->forceField[real_i * jello->resolution * jello->resolution + real_j * jello->resolution + real_k];
	point BOTTOM_RIGHT = jello->forceField[(real_i + 1) * jello->resolution * jello->resolution + real_j * jello->resolution + real_k];
	point TOP_LEFT = jello->forceField[real_i * jello->resolution * jello->resolution + (real_j + 1) * jello->resolution + real_k];
	point TOP_RIGHT = jello->forceField[(real_i + 1) * jello->resolution * jello->resolution + (real_j + 1) * jello->resolution + real_k];
	// front face
	point BOTTOM_LEFT_2 = jello->forceField[real_i * jello->resolution * jello->resolution + real_j * jello->resolution + (real_k + 1)];
	point BOTTOM_RIGHT_2 = jello->forceField[(real_i + 1) * jello->resolution * jello->resolution + real_j * jello->resolution + (real_k + 1)];
	point TOP_LEFT_2 = jello->forceField[real_i * jello->resolution * jello->resolution + (real_j + 1) * jello->resolution + (real_k + 1)];
	point TOP_RIGHT_2 = jello->forceField[(real_i + 1) * jello->resolution * jello->resolution + (real_j + 1) * jello->resolution + (real_k + 1)];

	// Obtain the distance between force position (int) and vertex position (double)
	// First transform force position back to world coordinate 
	// Then find the distance between two positions 
	// Lastly, compute the portion of that distance in the cube
	point p = { (jello->p[i][j][k].x - (-2 + 4 * real_i / length)) / (4.0 / length),
				(jello->p[i][j][k].y - (-2 + 4 * real_j / length)) / (4.0 / length),
				(jello->p[i][j][k].z - (-2 + 4 * real_k / length)) / (4.0 / length) };
	// 3D interpolation
	/********************************************
	TOP_LEFT_2        ________   TOP_RIGHT_2
					/|       /|
				   / |      / |
	TOP_LEFT      /__|____ /  |  TOP_RIGHT
	BOTTOM_LEFT_2 |  |____|__/   BOTTOM_RIGHT_2
				  | /     | /
	BOTTOM_LEFT   |/______|/     BOTTOM_RIGHT

	********************************************/
	pMULTIPLY(BOTTOM_LEFT, (1 - p.x) * (1 - p.y) * (1 - p.z), BOTTOM_LEFT);
	pMULTIPLY(TOP_LEFT, (1 - p.x) * p.y * (1 - p.z), TOP_LEFT);
	pMULTIPLY(BOTTOM_RIGHT, p.x * (1 - p.y) * (1 - p.z), BOTTOM_RIGHT);
	pMULTIPLY(TOP_RIGHT, p.x * p.y * (1 - p.z), TOP_RIGHT);

	pSUM(a, BOTTOM_LEFT, a);
	pSUM(a, TOP_LEFT, a);
	pSUM(a, BOTTOM_RIGHT, a);
	pSUM(a, TOP_RIGHT, a);

	pMULTIPLY(BOTTOM_LEFT_2, (1 - p.x) * (1 - p.y) * p.z, BOTTOM_LEFT_2);
	pMULTIPLY(TOP_LEFT_2, (1 - p.x) * p.y * p.z, TOP_LEFT_2);
	pMULTIPLY(BOTTOM_RIGHT_2, p.x * (1 - p.y) * p.z, BOTTOM_RIGHT_2);
	pMULTIPLY(TOP_RIGHT_2, p.x * p.y * p.z, TOP_RIGHT_2);

	pSUM(a, BOTTOM_LEFT_2, a);
	pSUM(a, TOP_LEFT_2, a);
	pSUM(a, BOTTOM_RIGHT_2, a);
	pSUM(a, TOP_RIGHT_2, a);
}

/***********************************************************
Function collisionWall
Compute collision force from wall for a vertex
************************************************************/
void collisionWall(world *jello, int i, int j, int k, point& a) {

	// check if one coordinate of vertex is out of bound
	bool Xin = (jello->p[i][j][k].x > -2) && (jello->p[i][j][k].x < 2) ? true : false;
	bool Yin = (jello->p[i][j][k].y > -2) && (jello->p[i][j][k].y < 2) ? true : false;
	bool Zin = (jello->p[i][j][k].z > -2) && (jello->p[i][j][k].z < 2) ? true : false;

	if (!Xin || !Yin || !Zin) {

		double Kh = jello->kCollision;
		double Kd = jello->dCollision;

		// find the surface point, difference = normal * magnitude
		point surface = jello->p[i][j][k];
		surface.x = (jello->p[i][j][k].x <= -2) ? -2 : ((jello->p[i][j][k].x >= 2) ? 2 : surface.x);
		surface.y = (jello->p[i][j][k].y <= -2) ? -2 : ((jello->p[i][j][k].y >= 2) ? 2 : surface.y);
		surface.z = (jello->p[i][j][k].z <= -2) ? -2 : ((jello->p[i][j][k].z >= 2) ? 2 : surface.z);

		/*****************************************
		Hook's Law : F = -k(|L| - R) x L/|L|
		= -k(|L|) x L/|L| = -k x L
		------------------------------------------
		Damping : F = -k(|va -vb|.L)/|L| x L/|L|
		= -k(|va|.L)/|L| x L/|L|
		*****************************************/
		double length = 0.0, hook = 0.0, dot = 0.0;
		point L, H, D;
		pDIFFERENCE(jello->p[i][j][k], surface, L);		// L
		pMULTIPLY(L, -Kh, H);							// -k x L
		pSUM(H, a, a);

		pDIFFERENCE(jello->p[i][j][k], surface, L);		// L
		DOTPRODUCT(jello->v[i][j][k], L, dot);			// |va -vb|.L
		pNORMALIZE(L);									// L/|L|
		double damp = -Kd * dot / length;				// -k(|va|.L)/|L|
		pMULTIPLY(L, damp, D);							// -k(|va|.L)/|L| x L/|L|
		pSUM(D, a, a);
	}
}

/***********************************************************
Function collisionPlane
Compute collision force from inclined plane for a vertex
************************************************************/
void collisionPlane(world * jello, int i, int j, int k, point& f) {

	// if inclined plane exists
	if (jello->incPlanePresent) {

		// inclined plane has equation a * x + b * y + c * z + d = 0;
		double a = jello->a, b = jello->b, c = jello->c, d = jello->d;

		/*************************************************************
		Projected point on the plane (x0 - at, y0 - bt, z0 - ct)
		t = (ax0 + by0 + cz0 + d) / (a** + b** + c**)
		Distance from point to plane = | w . n | / |n|
		= |a(x - x0) + b(y - y0) + c(z - z0)| / sqrt(a** + b** + c**)
		= |ax0 + by0 + cz0 + d| / sqrt(a** + b** + c**)
		**************************************************************/

		point normal = { a, b, c };							// normal = coefficent
		point p = jello->p[i][j][k];						// vector w from plane to point = (x, y, z) - p
		double t = 0.0, length = 0.0;
		DOTPRODUCT(normal, p, t);
		t += d;
		t /= (a * a + b * b + c * c);
		point projected = { p.x - a * t, p.y - b * t, p.z - c * t };
		point direction;
		pDIFFERENCE(p, projected, direction);
		double same;
		DOTPRODUCT(normal, direction, same);

		if (same < 0) {
			/*****************************************
			Hook's Law : F = -k(|L| - R) x L/|L|
			= -k(|L| - 0) x L/|L| = -k x L
			------------------------------------------
			Damping : F = -k(|va -vb|.L)/|L| x L/|L|
			= -k(|va|.L)/|L| x L/|L|
			*****************************************/
			double Kh = jello->kCollision;
			double Kd = jello->dCollision;
			double hook = 0.0, dot = 0.0;
			point L, H, D;
			point v = jello->v[i][j][k];

			pNORMALIZE(normal);
			pDIFFERENCE(p, projected, L);					// L
			pMULTIPLY(L, -Kh, H);							// -k x L
			pSUM(H, f, f);

			pDIFFERENCE(p, projected, L);					// L
			DOTPRODUCT(v, L, dot);							// |va|.L
			pNORMALIZE(L);									// L/|L|
			double damp = -Kd * dot / length;				// -k(|va|.L)/|L|
			pMULTIPLY(L, damp, D);							// -k(|va|.L)/|L| x L/|L|
			pSUM(D, f, f);
		}
	}
}

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8]) {
	/* for you to implement ... */
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			for (int k = 0; k < 8; k++) {

				/******************************************
				F_total = F_hook + F_damping + F_forcefield
				F_internal = Sigma(F_hook + F_damping)
				F_external = F_forcefield
				******************************************/

				// compute internal forces
				point internal = { 0.0, 0.0, 0.0 };
				structualForce(jello, i, j, k, internal);
				shearForce(jello, i, j, k, internal);
				bendForce(jello, i, j, k, internal);
				pCPY(internal, a[i][j][k]);

				// compute external forces
				point external = { 0.0, 0.0, 0.0 };
				externalForce(jello, i, j, k, external);
				pSUM(external, a[i][j][k], a[i][j][k]);

				// compute collision forces from wall 
				point collide = { 0.0, 0.0, 0.0 };
				collisionWall(jello, i, j, k, collide);
				pSUM(collide, a[i][j][k], a[i][j][k]);

				// compute collision forces for inclined plane
				point collideP = { 0.0, 0.0, 0.0 };
				collisionPlane(jello, i, j, k, collideP);
				pSUM(collideP, a[i][j][k], a[i][j][k]);

				// camera Looking at (gluLookAt) : R * cos(Phi) * cos(Theta), R * sin(Phi) * cos(Theta), R * sin(Theta);
				// if push/pull activated, left mouse button = push, middle mouse button = pull.
				if (pushnpull) {
					if (g_iLeftMouseButton) {
						point push = { 0.5 * cos(Phi) * cos(Theta), 0.5 * sin(Phi) * cos(Theta), 0.5 * sin(Theta) };
						pDIFFERENCE(a[i][j][k], push, a[i][j][k]);
					}
					if (g_iMiddleMouseButton) {
						point pull = { 0.5 * cos(Phi) * cos(Theta), 0.5 * sin(Phi) * cos(Theta), 0.5 * sin(Theta) };
						pSUM(pull, a[i][j][k], a[i][j][k]);
					}
				}


				// a = 1/m x F_total
				pMULTIPLY(a[i][j][k], (1 / jello->mass), a[i][j][k]);
			}
		}
	}
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello) {
	int i, j, k;
	point a[8][8][8];

	computeAcceleration(jello, a);

	for (i = 0; i <= 7; i++) {
		for (j = 0; j <= 7; j++) {
			for (k = 0; k <= 7; k++) {

				jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
				jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
				jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
				jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
				jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
				jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

			}
		}
	}
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello) {
	point F1p[8][8][8], F1v[8][8][8],
		F2p[8][8][8], F2v[8][8][8],
		F3p[8][8][8], F3v[8][8][8],
		F4p[8][8][8], F4v[8][8][8];

	point a[8][8][8];


	struct world buffer;

	int i, j, k;

	buffer = *jello; // make a copy of jello

	computeAcceleration(jello, a);

	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++) {
				pMULTIPLY(jello->v[i][j][k], jello->dt, F1p[i][j][k]);
				pMULTIPLY(a[i][j][k], jello->dt, F1v[i][j][k]);
				pMULTIPLY(F1p[i][j][k], 0.5, buffer.p[i][j][k]);
				pMULTIPLY(F1v[i][j][k], 0.5, buffer.v[i][j][k]);
				pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
				pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
			}

	computeAcceleration(&buffer, a);

	for (i = 0; i <= 7; i++) {
		for (j = 0; j <= 7; j++) {
			for (k = 0; k <= 7; k++) {
				// F2p = dt * buffer.v;
				pMULTIPLY(buffer.v[i][j][k], jello->dt, F2p[i][j][k]);
				// F2v = dt * a(buffer.p,buffer.v);     
				pMULTIPLY(a[i][j][k], jello->dt, F2v[i][j][k]);
				pMULTIPLY(F2p[i][j][k], 0.5, buffer.p[i][j][k]);
				pMULTIPLY(F2v[i][j][k], 0.5, buffer.v[i][j][k]);
				pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
				pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
			}
		}
	}

	computeAcceleration(&buffer, a);

	for (i = 0; i <= 7; i++) {
		for (j = 0; j <= 7; j++) {
			for (k = 0; k <= 7; k++) {
				// F3p = dt * buffer.v;
				pMULTIPLY(buffer.v[i][j][k], jello->dt, F3p[i][j][k]);
				// F3v = dt * a(buffer.p,buffer.v);     
				pMULTIPLY(a[i][j][k], jello->dt, F3v[i][j][k]);
				pMULTIPLY(F3p[i][j][k], 1.0, buffer.p[i][j][k]);
				pMULTIPLY(F3v[i][j][k], 1.0, buffer.v[i][j][k]);
				pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
				pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
			}
		}
	}

	computeAcceleration(&buffer, a);


	for (i = 0; i <= 7; i++) {
		for (j = 0; j <= 7; j++) {
			for (k = 0; k <= 7; k++) {
				// F3p = dt * buffer.v;
				pMULTIPLY(buffer.v[i][j][k], jello->dt, F4p[i][j][k]);
				// F3v = dt * a(buffer.p,buffer.v);     
				pMULTIPLY(a[i][j][k], jello->dt, F4v[i][j][k]);

				pMULTIPLY(F2p[i][j][k], 2, buffer.p[i][j][k]);
				pMULTIPLY(F3p[i][j][k], 2, buffer.v[i][j][k]);
				pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F1p[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F4p[i][j][k], buffer.p[i][j][k]);
				pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], jello->p[i][j][k], jello->p[i][j][k]);

				pMULTIPLY(F2v[i][j][k], 2, buffer.p[i][j][k]);
				pMULTIPLY(F3v[i][j][k], 2, buffer.v[i][j][k]);
				pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F1v[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F4v[i][j][k], buffer.p[i][j][k]);
				pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], jello->v[i][j][k], jello->v[i][j][k]);
			}
		}
	}
	return;
}
