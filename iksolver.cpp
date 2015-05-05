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
    glTranslatef(arm->base.x, arm->base.y, 0.0f);
    glutSolidSphere(0.5f, 20, 20);
    
    GLUquadricObj *quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricOrientation(quad, GLU_OUTSIDE);
    gluQuadricNormals(quad, GLU_SMOOTH);
    
    for (int i = 0; i < arm->list_joints.size(); i++) {
        Joint* joint = arm->list_joints[i];
        
        glColor3f(1.0, 0.0, 1.0);
        glRotatef(joint->rotation.x, 1.0f, 0.0f, 0.0f);
        glRotatef(joint->rotation.y, 0.0f, 1.0f, 0.0f);
        glRotatef(joint->rotation.z, 0.0f, 0.0f, 1.0f);
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

Matrix3f rodriguez(Joint* j) {
        Vector3f R(j->rotation.x, j->rotation.y, j->rotation.z);
        R.normalize();

        Matrix3f crossProd(3,3);
        crossProd(0,0) = 0.0f; crossProd(0, 1) =  -(R(2)); crossProd(0,2) = R(1);
        crossProd(1,0) = R(2); crossProd(1, 1) = 0.0; crossProd(1,2) = -(R(0));
        crossProd(2,0) = -(R(1)); crossProd(2, 1) = R(0); crossProd(2,2) = 0.0;

        Matrix3f crossProd_squ(3,3);
        crossProd_squ = crossProd * crossProd;

        float theta = j->rotation.magnitude();

        return (R * R.transpose()) + sin(theta) * crossProd - cos(theta)*crossProd_squ;
    }



int main (int argc, char **argv) {
    // initialize the arm and joints
    Arm* arm = new Arm();
    vector<Vector> bigTheta;

    //All angles ri = 0
    Vector r1 = Vector(); r1.x = 30; r1.y = 60; r1.z = 90; bigTheta.push_back(r1);
    Vector r2 = Vector(); bigTheta.push_back(r2);
    Vector r3 = Vector(); bigTheta.push_back(r3);
    Vector r4 = Vector(); bigTheta.push_back(r4);
    //Rotation should be a matrix
    Joint* j1 = new Joint(); j1->p.y = 2.0; j1->rotation = r1; arm->list_joints.push_back(j1);
    Joint* j2 = new Joint(); j2->p.y = 4.0; j2->rotation = r2; arm->list_joints.push_back(j2);
    Joint* j3 = new Joint(); j3->p.y = 6.0; j3->rotation = r3; arm->list_joints.push_back(j3);
    Joint* j4 = new Joint(); j4->p.y = 8.0; j4->rotation = r4; arm->list_joints.push_back(j4);
    
    list_arm.push_back(arm);

    r1.normalize(); r2.normalize(); r3.normalize(); r4.normalize();

    Matrix3f R1; R1 = rodriguez(j1); cout << R1;


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