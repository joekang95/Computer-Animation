/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  Joe Yu-Ho Chang

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int width, height, nrChannels;
GLuint texture, skybox[6];

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton, g_iMiddleMouseButton, g_iRightMouseButton;

// number of images saved to disk so far
int sprite = 0, counter = 0, autoscreen = 0;

// these variables control what is displayed on screen
int shear = 0, bend = 0, structural = 1, pause = 0, viewingMode = 1, saveScreenToFile = 0,  plane = 1, tex = 1, pushnpull = 0, env = 0;

struct world jello;

int windowWidth, windowHeight;

/***********************************************
function loadTexture
read image file and bind it to texture handele
***********************************************/
void loadTexture(GLuint * texture, const char * filename) {
	// Read texture image
	unsigned char *data = stbi_load(filename, &width, &height, &nrChannels, 3);
	// Set the Texture Parameters
	glBindTexture(GL_TEXTURE_2D, *texture);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	if (data) {
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
	}
	else {
		std::cout << "Failed to load texture:" << filename << std::endl;
	}
	stbi_image_free(data);
}


void myinit() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, 1.0, 0.01, 1000.0);

	// set background color to grey
	glClearColor(0.5, 0.5, 0.5, 0.0);

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	// load inclined plane texture
	glGenTextures(1, &texture);
	loadTexture(&texture, "textures/Wood.jpg");

	// load skybox texture
	glGenTextures(6, skybox);
	loadTexture(&skybox[0], "textures/SkyFront.jpg");
	loadTexture(&skybox[1], "textures/SkyBack.jpg");
	loadTexture(&skybox[2], "textures/SkyLeft.jpg");
	loadTexture(&skybox[3], "textures/SkyRight.jpg");
	loadTexture(&skybox[4], "textures/SkyTop.jpg");
	loadTexture(&skybox[5], "textures/SkyBottom.jpg");

	return;
}

void reshape(int w, int h) {
	// Prevent a divide by zero, when h is zero.
	// You can't make a window of zero height.
	if (h == 0)
		h = 1;

	glViewport(0, 0, w, h);

	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the perspective
	double aspectRatio = 1.0 * w / h;
	gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	windowWidth = w;
	windowHeight = h;

	glutPostRedisplay();
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// camera parameters are Phi, Theta, R
	gluLookAt(R * cos(Phi) * cos(Theta), R * sin(Phi) * cos(Theta), R * sin(Theta),
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0);


	/* Lighting */
	/* You are encouraged to change lighting parameters or make improvements/modifications
	   to the lighting model .
	   This way, you will personalize your assignment and your assignment will stick out.
	*/

	// global ambient light
	GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };

	// light 's ambient, diffuse, specular
	GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

	GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
	GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

	GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
	GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

	GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
	GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

	GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
	GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

	GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
	GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

	GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

	GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
	GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

	// light positions and directions
	GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
	GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
	GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
	GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
	GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
	GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
	GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
	GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };

	// jelly material color

	GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };
	GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

	/* set up lighting */
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	// set up cube color
	glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
	glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
	glMaterialf(GL_FRONT, GL_SHININESS, 120);

	// macro to set up light i
#define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)

	LIGHTSETUP(0);
	LIGHTSETUP(1);
	LIGHTSETUP(2);
	LIGHTSETUP(3);
	LIGHTSETUP(4);
	LIGHTSETUP(5);
	LIGHTSETUP(6);
	LIGHTSETUP(7);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	// show the cube
	showCube(&jello);

	glDisable(GL_LIGHTING);

	glBindTexture(GL_TEXTURE_2D, texture);
	showPlane(&jello);  // Cannot be within GL_LIGHTING or else color is ignored

	// show the bounding box
	showBoundingBox();

	showSkyBox(skybox);
	glutSwapBuffers();
}

void doIdle() {
	char s[20] = "picxxxx.ppm";
	int i;

	// save screen to file
	s[3] = 48 + (sprite / 1000);
	s[4] = 48 + (sprite % 1000) / 100;
	s[5] = 48 + (sprite % 100) / 10;
	s[6] = 48 + sprite % 10;

	if (saveScreenToFile == 1) {
		if (autoscreen && counter % 5 == 0 && counter != 0) {
			Phi += 1.3 * 0.01;
			if (Phi > 2 * pi) { Phi -= 2 * pi; }
			if (Phi < 0) { Phi += 2 * pi; }
			if (sprite <= 50) {
				// saveScreenToFile = 0; // save only once, change this if you want continuous image generation (i.e. animation)
				saveScreenshot(windowWidth, windowHeight, s);
			}
			else if (sprite <= 100) {
				tex = 0;
				saveScreenshot(windowWidth, windowHeight, s);
			}
			else if (sprite <= 150) {
				plane = 0;
				viewingMode = 0;
				saveScreenshot(windowWidth, windowHeight, s);
			}
			else if (sprite <= 200) {
				viewingMode = 1;
				plane = 1;
				tex = 1;
				env = 1;
				saveScreenshot(windowWidth, windowHeight, s);
			}
			if (sprite == 200) {
				autoscreen = 0;
				saveScreenToFile = 0;
			}
			sprite++;
		}
		if (autoscreen == 0) {
			saveScreenToFile = 0; // save only once, change this if you want continuous image generation (i.e. animation)
			saveScreenshot(windowWidth, windowHeight, s);
			sprite++;
		}
		counter++;
	}

	if (sprite >= 300) { // allow only 300 snapshots

		exit(0);
	}

	if (pause == 0) {
		// insert code which appropriately performs one step of the cube simulation:
		if (jello.integrator[0] == 'E') {
			Euler(&jello);
		}
		else if (jello.integrator[0] == 'R') {
			RK4(&jello);
		}
	}

	glutPostRedisplay();
}

int main(int argc, char ** argv) {
	if (argc < 2) {
		printf("Oops! You didn't say the jello world file!\n");
		printf("Usage: %s [worldfile]\n", argv[0]);
		exit(0);
	}

	readWorld(argv[1], &jello);

	glutInit(&argc, argv);

	/* double buffered window, use depth testing, 640x480 */
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	windowWidth = 640;
	windowHeight = 480;
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Jello cube"); 
	/*For Mac OSX 10.15 if the program is displaying on the bottom left corner*/
	// glutReshapeWindow(399, 399);

	/* tells glut to use a particular display function to redraw */
	glutDisplayFunc(display);

	/* replace with any animate code */
	glutIdleFunc(doIdle);

	/* callback for mouse drags */
	glutMotionFunc(mouseMotionDrag);

	/* callback for window size changes */
	glutReshapeFunc(reshape);

	/* callback for mouse movement */
	glutPassiveMotionFunc(mouseMotion);

	/* callback for mouse button changes */
	glutMouseFunc(mouseButton);

	/* register for keyboard events */
	glutKeyboardFunc(keyboardFunc);

	/* do initialization */
	myinit();

	/* forever sink in the black hole */
	glutMainLoop();

	return(0);
}

