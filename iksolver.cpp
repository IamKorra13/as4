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

vector<Arm*> list_arm = vector<Arm*>();
// The initial location of point P
Vector p = Vector(); 
//Goal point 
Vector g = Vector();


void drawArm(Arm* arm) {
    glPushMatrix();
    glTranslatef(arm->base(0), arm->base(1), arm->base(2));
    glutSolidSphere(0.5f, 20, 20);
    
    GLUquadricObj *quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricOrientation(quad, GLU_OUTSIDE);
    gluQuadricNormals(quad, GLU_SMOOTH);
    
    for (int i = 0; i < arm->list_joints.size(); i++) {
        Joint* joint = arm->list_joints[i];
        
        glColor3f(1.0, 0.0, 1.0);
        glRotatef(joint->rotation(0), 1.0f, 0.0f, 0.0f);
        glRotatef(joint->rotation(1), 0.0f, 1.0f, 0.0f);
        glRotatef(joint->rotation(2), 0.0f, 0.0f, 1.0f);
        gluCylinder(quad, 0.2, 0.2, joint->length, 20, 20);
        glTranslatef(0, 0, joint->length);
        
        // Draw the sphere to show the joint position
        if (i < 3) {
            glutSolidSphere(0.5, 20, 20);
        }
    }
    //draw P
    glColor3f(0.0, 0.0, 1.0);
    glutSolidSphere(0.5, 20, 20);

    glPopMatrix();
}

void drawGoal() {
    
}
/* Main display function. */
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);		// clear the color buffer
    glMatrixMode(GL_MODELVIEW);			        // indicate we are specifying camera transformations
    glLoadIdentity();
    
    glPushMatrix();
    
    /// stuff here
    drawArm(list_arm[0]);
    drawGoal();
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
    int num = 12;
    glOrtho(-num, num, -num, num, num, -num);
    
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
    Arm* arm = new Arm();

    Joint* j1 = new Joint(); arm->list_joints.push_back(j1);
    Joint* j2 = new Joint(); arm->list_joints.push_back(j2);
    Joint* j3 = new Joint(); arm->list_joints.push_back(j3);
    Joint* j4 = new Joint(); arm->list_joints.push_back(j4);

    //j1->rotation << 30.0f, 60.0f, 80.0f;

    vector<Vector3f> bigTheta = vector<Vector3f>();
    bigTheta.push_back(j1->rotation);
    bigTheta.push_back(j2->rotation);
    bigTheta.push_back(j3->rotation);
    bigTheta.push_back(j4->rotation);
    
    list_arm.push_back(arm);

    arm->list_joints[0]->print();

    cout << endl;
    arm->list_joints[0]->rodrigues();
    cout << endl << endl;
    arm->list_joints[0]->transformation();
    cout << endl;
    arm->F(bigTheta);

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