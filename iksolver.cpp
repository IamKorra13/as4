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
Vector p = Vector(); // The initial location of point P
vector<Vector3f> goals = vector<Vector3f>();

VectorXf bigTheta(12); // the rotations of the arm


void updateJoints(Arm* arm, VectorXf bigTheta) {
    arm->list_joints[0]->rotation << bigTheta(0), bigTheta(1), bigTheta(2);
    arm->list_joints[1]->rotation << bigTheta(3), bigTheta(4), bigTheta(5);
    arm->list_joints[2]->rotation << bigTheta(6), bigTheta(7), bigTheta(8);
    arm->list_joints[3]->rotation << bigTheta(9), bigTheta(10), bigTheta(11);
}

void solver(Arm* arm) {
    Vector3f error;
    error = arm->C(bigTheta);
    VectorXf newBigTheta(12);
    float step_tolerance = 0.00005f;
    int i = 0;

    // get a new goal if reached old one
    if(error.norm() <= 0.1f && goals.size() > 0) {
        arm->step_size = 0.5f;
        arm->goal = goals.back(); goals.pop_back();
    }

    if (error.norm() >= 0.1f) {
        newBigTheta = arm->update(bigTheta);
        updateJoints(arm, newBigTheta);

        //update step size
        Vector3f newError = arm->C(newBigTheta);
        if(newError.norm()/error.norm() > 1.0f) {
            arm->step_size = arm->step_size/2.0f;
        }

            // goal is out of reach and there's still a new goal
    if(arm->step_size <= step_tolerance && goals.size() > 0) {
        cout << "Out of reach" << endl;
        arm->step_size = 0.08f;
        arm->goal = goals.back(); goals.pop_back();
        // break;
    }

    // goal is just out of reach
    if(arm->step_size <= step_tolerance) {
        return;
    }
        bigTheta = newBigTheta;
        error = newError;


    }
    cout << "Current goal:" << endl << arm->goal << endl;
    cout << "Num goals left: " << goals.size() << endl;
    cout << "Step size: " << arm->step_size << endl;
    cout << "Arm ending position: " << endl << arm->F(bigTheta) << endl << "=======" << endl;


    return;
}

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
    // initialize goals
    
    
    Vector3f goal1(0.0f, 11.0f, 0.0f); goals.push_back(goal1); // right 
    Vector3f goal2(0.0f, 20.0f, 10.0f); goals.push_back(goal2); // down 
    Vector3f goal3(20.0f, 10.0f, 0.0f); goals.push_back(goal3); // left
    Vector3f goal4(0.0f, -3.0f, -11.0f); goals.push_back(goal4);
    
    //Vector3f goal3(200.0f, 100.0f, 0.0f); goals.push_back(goal3);
    //Vector3f goal4(0.0f, -30.0f, -110.0f); goals.push_back(goal4);
    //Vector3f goal2(0.0f, 200.0f, 100.0f); goals.push_back(goal2);
    //Vector3f goal1(0.0f, 100.0f, 0.0f); goals.push_back(goal1);
    
    
       

    // initialize the arm and joints
    Arm* arm = new Arm();

    Joint* j1 = new Joint(); j1->length = 2.0f; arm->list_joints.push_back(j1);
    Joint* j2 = new Joint(); j2->length = 3.0f; arm->list_joints.push_back(j2);
    Joint* j3 = new Joint(); j3->length = 1.0f; arm->list_joints.push_back(j3);
    Joint* j4 = new Joint(); j4->length = 5.0f; arm->list_joints.push_back(j4);

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

    arm->goal = goals.back(); goals.pop_back();
    arm->step_size = 0.08f;


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
    glutSpecialFunc(processSpecialKeys);

    glutMainLoop();
    
    

    return 0;
}

