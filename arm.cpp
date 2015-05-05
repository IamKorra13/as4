#include "arm.h"

Joint::Joint() {
    length = 2.0f;
    rotation = Vector();
    p = Vector();
    
}

Arm::Arm() {
    list_joints = vector<Joint*>();
    base = Vector();
    target = Vector(4.0f, 0.0f, 0.0f);
}

