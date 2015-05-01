
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iostream>     // std::
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

#include "triangle.cpp"
#include "curve.cpp"
#include "vector.cpp"


using namespace std;

class Viewport;

class Viewport {
public:
    int w, h; // width and height
};

Viewport	viewport;
int windowID;

//List of all the Surface Patches
vector<Triangle> list_triangles;
vector<vector<Vector> > points_to_Render;
vector<Vector> curlevel;


// user settings
bool wireframe = false;
bool isAdaptive = false;
bool flatShading = false;


float rotate_x = 0.0f;
float rotate_y = 0.0f;
float scale = 1.0f;
float translate_x = 0.0f;
float translate_y = 0.0f;


void initScene(){
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent
}

/*Resizes Viewport.*/
void myReshape(int w, int h) {
    viewport.w = w;
    viewport.h = h;
    
    glViewport (0,0,viewport.w,viewport.h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // gluOrtho2D(10, viewport.w, 10, viewport.h);
    glOrtho(-4, 4, -4, 4, 4, -4);
    
}

void drawTriangle(Triangle tri) {
    Vector v1 = tri.v1;
    Vector v2 = tri.v2;
    Vector v3 = tri.v3;
    
    glBegin(GL_TRIANGLES);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    glNormal3f(v1.normal[0], v1.normal[1], v1.normal[2]);
    glVertex3f(v1.x, v1.y, v1.z);
    glNormal3f(v2.normal[0], v2.normal[1], v2.normal[2]);
    glVertex3f(v2.x, v2.y, v2.z);
    glNormal3f(v3.normal[0], v3.normal[1], v3.normal[2]);
    glVertex3f(v3.x, v3.y, v3.z);
    
    glEnd();
    
}

/* Main display function. */
void myDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);		// clear the color buffer
    glMatrixMode(GL_MODELVIEW);			        // indicate we are specifying camera transformations
    glLoadIdentity();
    
    glPushMatrix();
    
    glRotatef(rotate_y, 1.0, 0.0, 0.0);
    glRotatef(rotate_x, 0.0, 1.0, 0.0);
    glScalef(scale, scale, scale);
    glTranslatef(translate_x, translate_y, 0.0);
    //glScalef(2.0, 2.0, 2.0);
    /* Draws the triangles.*/
    for (int i = 0; i < list_triangles.size(); i++) {
        drawTriangle(list_triangles[i]);
    }
    glPopMatrix();
    
    glFlush();
    glutSwapBuffers();					// swap buffers (we earlier set double buffer)
}


void keyBoardFunc(unsigned char key, int x, int y) {
        switch(key)
        {
            case 32: //spacebar
                exit(0);
        }
}

//For the move tool
void processSpecialKeys(int key, int x, int y) {
    int mod = glutGetModifiers();
    switch(key) {
        case GLUT_KEY_UP:
            if(mod == GLUT_ACTIVE_SHIFT) {
                translate_y += 0.3f;
            }
            else {
                rotate_y += 10.0f;
            }
            myDisplay();
            break;
        case GLUT_KEY_DOWN:
            if(mod == GLUT_ACTIVE_SHIFT) {
                translate_y -= 0.3f;
            }
            else {
                rotate_y -= 10.0f;
            }
            myDisplay();
            break;
        case GLUT_KEY_RIGHT:
            if(mod == GLUT_ACTIVE_SHIFT) {
                translate_x += 0.3f;
            }
            else {
                rotate_x -= 10.0f;
            }
            myDisplay();
            break;
        case GLUT_KEY_LEFT:
            if(mod == GLUT_ACTIVE_SHIFT) {
                translate_x -= 0.3f;
            }
            else {
                rotate_x += 10.0f;
            }
            myDisplay();
            break;
    }
}




/* Uniform Subdivision. */
vector<Vector> bezcurveinterp(Curve curve, float u) {
    // first, split each of the three segments # to form two new ones AB and BC
    //A B and C are vectors with (x,y,z)
    Vector A, B, C, D, E, temp1, temp2 = Vector();
    
    A = curve.pt1 * (1.0f-u) + curve.pt2 * u;
    B = curve.pt2 * (1.0f-u) + curve.pt3 * u;
    C = curve.pt3 * (1.0f-u) + curve.pt4 * u;
    
    
    // now, split AB and BC to form a new segment DE
    D = A * (1.0f - u) + B * u;
    E = B * (1.0f - u) + C * u;
    
    // finally, pick the right point on DE, # this is the point on the curve
    Vector p = D * (1.0f - u) + E * u;
    Vector dPdu = (E - D) * 3.0f; // derivative
    
    vector<Vector> result;
    result.push_back(p); result.push_back(dPdu);
    return result;
}


/* populates triangle list */
void makeTriangles() {
    for (int i = 0; i < numSubdivisions; i++) {
        for (int j = 0; j < numSubdivisions; j++) {
            Triangle triangle1 = Triangle(points_to_Render[i][j], points_to_Render[i][j+1],
                                          points_to_Render[i+1][j]);
            Triangle triangle2 = Triangle(points_to_Render[i+1][j], points_to_Render[i][j+1],
                                          points_to_Render[i+1][j+1]);
            list_triangles.push_back(triangle1); list_triangles.push_back(triangle2);
        }
    }
    
    //reset all the lists
    points_to_Render = vector<vector<Vector> >(); curlevel = vector<Vector>();
}


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {    
    
    //This initializes glut
    glutInit(&argc, argv);
    
    //This tells glut to use a double-buffered window with red, green, and blue channels 
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    
    // Initalize theviewport size
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
    
    initScene();							// quick function to set up scene
    glutDisplayFunc(myDisplay);			// function to run when its time to draw something
    glutReshapeFunc(myReshape);       // function to run when the window gets resized
    glutKeyboardFunc(keyBoardFunc);
    glutSpecialFunc(processSpecialKeys);
    
    
    glutMainLoop();							// infinite loop that will keep drawing and resizing
    // and whatever else
    return 0;
}







