#include "arm.h"

Joint::Joint() {
    length = 2.0f;
    angle = 0.0f;
    
}

Arm::Arm() {
    list_joints = vector<Joint*>();
    base = Vector();
    target = Vector(4.0f, 0.0f, 0.0f);
}

