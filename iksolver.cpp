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

VectorXf bigTheta(12);

Vector3f error;


void updateJoints(Arm* arm, VectorXf bigTheta) {
    cout << "Updating the rotations" << endl;
    arm->list_joints[0]->rotation << bigTheta(0), bigTheta(1), bigTheta(2);
    cout << "rotation 1:" << endl << arm->list_joints[0]->rotation << endl;
    
    arm->list_joints[1]->rotation << bigTheta(3), bigTheta(4), bigTheta(5);
    cout << "rotation 2:" << endl << arm->list_joints[1]->rotation << endl;
    
    arm->list_joints[2]->rotation << bigTheta(6), bigTheta(7), bigTheta(8);
    cout << "rotation 3:" << endl << arm->list_joints[2]->rotation << endl;
    
    arm->list_joints[3]->rotation << bigTheta(9), bigTheta(10), bigTheta(11);
    cout << "rotation 4:" << endl << arm->list_joints[3]->rotation << endl;
}

void solver(Arm* arm) {
    error = arm->C(bigTheta);
    VectorXf newBigTheta(12);
    int i = 0;
     // while(error.norm() >= 0.001f) {
        if (error.norm() >= 0.001f) {
        newBigTheta = arm->update(bigTheta);
        updateJoints(arm, newBigTheta);

        //update step size
        Vector3f newError = arm->C(newBigTheta);
        if(newError.norm()/error.norm() > 1.0f) {
            arm->step_size = arm->step_size/2.0f;
        }
        bigTheta = newBigTheta;
        error = newError;

        // goal is out of reach
        if(arm->step_size <= 0.003f) {
            cout << "Out of reach" << endl;
            // break;
        }
    }
    cout << "Arm ending position: " << arm->F(bigTheta);
    return;
}

void drawArm(Arm* arm) {
    cout << "Print";
    glPushMatrix();
    cout << "About to translate the arm base" << endl;
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
            glutSolidSphere(0.3, 20, 20);
        }
    }
    //draw P
    glColor3f(0.0, 0.0, 1.0);
    glutSolidSphere(0.5, 20, 20);

    glPopMatrix();
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
    glPopMatrix();
    solver(list_arm[0]);
    
    glFlush();
    glutSwapBuffers();					// swap buffers (we earlier set double buffer)

     glutPostRedisplay();
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
    int num = 5;
    glOrtho(-num, num, -num, num, num, -num);
    
}

void keyBoardFunc(unsigned char key, int x, int y) {
    switch(key)
    {
        case 32: //spacebar
            exit(0);
    }
}

/*
// This function is called when there is nothing else to do.
void idle ()
{
    glutPostRedisplay();
}

// This function gets called every 10ms
void timer(int i)
{
    glutTimerFunc(10, timer, i);
    glutPostRedisplay();
}*/

void updateBigTheta(Arm* arm) {
    bigTheta << arm->list_joints[0]->rotation(0),
        arm->list_joints[0]->rotation(1),
        arm->list_joints[0]->rotation(2),
        arm->list_joints[1]->rotation(0),
        arm->list_joints[1]->rotation(1),
        arm->list_joints[1]->rotation(2),
        arm->list_joints[2]->rotation(0),
        arm->list_joints[2]->rotation(1),
        arm->list_joints[2]->rotation(2),
        arm->list_joints[3]->rotation(0),
        arm->list_joints[3]->rotation(1),
        arm->list_joints[3]->rotation(2);
}





void processSpecialKeys(int key, int x, int y) {
    int mod = glutGetModifiers();
    switch(key) {
        case GLUT_KEY_RIGHT:
            if(mod == GLUT_ACTIVE_SHIFT) {
                Vector3f error = list_arm[0]->C(bigTheta);
                bigTheta = list_arm[0]->update(bigTheta);
                updateJoints(list_arm[0], bigTheta);
                // translate_x += 0.3f;
            }
            else {
                Vector3f error = list_arm[0]->C(bigTheta);
                bigTheta = list_arm[0]->update(bigTheta);
                updateJoints(list_arm[0], bigTheta);

                // rotate_x -= 10.0f;myD
            }
            display();
            break;
    }
}

int main (int argc, char **argv) {
    // initialize the arm and joints
    Arm* arm = new Arm();

    Joint* j1 = new Joint(); j1->length = 1.0f; arm->list_joints.push_back(j1);
    Joint* j2 = new Joint(); j2->length = 1.0f; arm->list_joints.push_back(j2);
    Joint* j3 = new Joint(); j3->length = 1.0f; arm->list_joints.push_back(j3);
    Joint* j4 = new Joint(); j4->length = 1.0f; arm->list_joints.push_back(j4);

    //j1->rotation << 30.0f, 60.0f, 80.0f;

    bigTheta << arm->list_joints[0]->rotation(0),
        arm->list_joints[0]->rotation(1),
        arm->list_joints[0]->rotation(2),
        arm->list_joints[1]->rotation(0),
        arm->list_joints[1]->rotation(1),
        arm->list_joints[1]->rotation(2),
        arm->list_joints[2]->rotation(0),
        arm->list_joints[2]->rotation(1),
        arm->list_joints[2]->rotation(2),
        arm->list_joints[3]->rotation(0),
        arm->list_joints[3]->rotation(1),
        arm->list_joints[3]->rotation(2);

    list_arm.push_back(arm);

    ///////

    Vector3f goal(-2.0f, -3.0f, -1.0f);
    arm->goal = goal;
    arm->step_size = 0.5f;
/*
    cout << endl;

    arm->rodrigues(arm->list_joints[0]->rotation);
    cout << endl << endl;
    arm->list_joints[0]->transformation();
    cout << endl;
    arm->F(bigTheta);
    MatrixXf j(3, 12); j = arm->jacobian(bigTheta);
    arm->C(bigTheta);
    arm->psuedo_inv_jacobian(bigTheta);
    arm->update(bigTheta);
*/
    //////


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

/*
    solver(arm);

    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyBoardFunc);
    glutIdleFunc(idle);
    glutTimerFunc(60, timer, 0);*/

    // solver(arm);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyBoardFunc);
    glutSpecialFunc(processSpecialKeys);

    glutMainLoop();
    
    

    return 0;
}

