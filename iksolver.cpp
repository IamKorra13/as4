#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iostream>     // std::cout
#include <algorithm>    // std::max

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>
#include <cfloat>

#include "arm.cpp"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

class Viewport;

class Viewport {
public:
    int w, h; // width and height
};

Viewport	viewport;
int windowID;



/* Main display function. */
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);		// clear the color buffer
    glMatrixMode(GL_MODELVIEW);			        // indicate we are specifying camera transformations
    glLoadIdentity();
    
    glPushMatrix();
    
    /// stuff here
    
    glPopMatrix();
    
    glFlush();
    glutSwapBuffers();					// swap buffers (we earlier set double buffer)
}


void initScene(){
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent
}

/*Resizes Viewport.*/
void reshape(int w, int h) {
    viewport.w = w;
    viewport.h = h;
    
    glViewport (0,0,viewport.w,viewport.h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // gluOrtho2D(10, viewport.w, 10, viewport.h);
    glOrtho(-4, 4, -4, 4, 4, -4);
    
}

void keyBoardFunc(unsigned char key, int x, int y) {
    switch(key)
    {
        case 32: //spacebar
            exit(0);
    }
}


int main (int argc, char **argv) {
    // initialize the arm and joints
    Arm arm = Arm();
    Joint* joint1 = new Joint(); arm.list_joints.push_back(joint1);
    Joint* joint2 = new Joint(); arm.list_joints.push_back(joint2);
    Joint* joint3 = new Joint(); arm.list_joints.push_back(joint3);
    Joint* joint4 = new Joint(); arm.list_joints.push_back(joint4);
    
    // GLUT initialization
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    
    // Initialize the viewport size
    viewport.w = 400;
    viewport.h = 400;
    
    //The size and position of the window
    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0,0);
    windowID = glutCreateWindow(argv[0]);  // saving the ID of the window possibly for quiting on spacebar
    
    
    //shading
    glShadeModel(GL_FLAT);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    
    float diffuse0[]={1.0, 0.0, 0.0, 1.0};
    float ambient0[]={1.0, 0.0, 0.0, 1.0};
    float specular0[]={1.0, 0.0, 0.0, 1.0};
    float light0_pos[]={-1.0, -2.0, -3,0, -1.0};
    
    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);
    
    glEnable(GL_LIGHT0);
    
    
    initScene();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyBoardFunc);
    glutMainLoop();
    
    return 0;
}