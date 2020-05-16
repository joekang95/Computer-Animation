<Please submit this file with your solution.>

CSCI 520, Assignment 1

Joe Yu-Ho Chang

================

<Description of what you have accomplished>

1. Animate the movement of a jello cube based on a realistic physical model (Done)
   - physics.cpp computeAcceleration(struct world * jello, struct point a[8][8][8])
   - computed internal force = structual + shear + bend
   - computed hook and damping force for three kinds of springs
2. Collision detection and response (Done)
   - computed collision forces from wall
   - computed hook and damping force for collision force
3. Implement an arbitrary non-homogeneous time-independent force field, with appropriate force-field interpolation (Done)
   - computed external force = force field
   - computed hook and damping force for external force
4. Interactively position the camera, use proper lighting, and render the cube in wireframe and triangle mode (Done)
   - key 'esc': exit
   - key 'e': reset camera position and viewing mode (triangle)
   - key 'v': change viewing mode (wireframe/triange)
   - key 'h': hide/show shear springs
   - key 's': hide/show structual springs
   - key 'b': hide/show bend springs
   - key 'p': pause animation
   - key 'z': zoom in camera
   - key 'x': zoom out camera
   - key 'space': take a screenshot
   - key '1': hide/show inclined plane
   - key '2': hide/show inclined plane texture
   - key '3': activate/deactivate equal user force push/pull from mouse
   - key '4': hide/show skybox
   
5. Read the description of the world from a world file and simulate the cube according to this information (Done)
   - readWorld in jello.cpp / input.cpp
6. Run at interactive frame rates (>15fps at 640x480) (Done)
7. Be reasonably commented and written in an understandable manner (Done)
8. Be submitted along with JPEG frames for the required animation (Done)
   - inside animation folder
9. Be submitted along with a README file (Done)

Note: 
1.If the program is displaying on the bottom left corner (for mac osx 10.15), please uncomment line 321 in jello.cpp
  OR simply resize the window using mouse to drag the window's corners
2.If want to auto save screenshots, set saveScreenToFile = 1 and autoscreen = 1 in jello.cpp


<Also, explain any extra credit that you have implemented.>

/*physics.cpp collisionP(world * jello, int i, int j, int k, point& f)*/
1. Inclined plane collision

/*showCube.cpp showPlane(struct world * jello)*/ 
2. Inclined plane hide/show with key '1'
3. Inclined plane texture on/off with key '2' 
(used stb_image.h to read and load image file. loadTexture(GLuint * texture, const char * filename) in jello.cpp)
4. Since there are 4 kinds of inclined plane, I adjusted jello.w
   - jello.w has an inclined plane with 3 intersections
   - jello2.w has an inclined plane with 4 intersections
   - jello3.w has an inclined plane with 5 intersections
   - jello4.w has an inclined plane with 6 intersections

/*computeAcceleration(struct world * jello, struct point a[8][8][8])*/
4. Activate/Deactivate mouse push and pull with key '3' 
(Apply the user force equally to all the simulation points)
5. Push cube further toward where camera is looking at with 'left mouse button'
6. Pull cube nearer toward the camera with 'middle mouse button'
Note: Pressing too long will cause the force to be significant large.


/*showCube.cpp showSkyBox(GLuint * skybox)*/ 
7. Skybox hide/show with key '4'
(used stb_image.h to read and load image files. loadTexture(GLuint * texture, const char * filename) in jello.cpp)