#include "arm.h"

Joint::Joint() {
    length = 1.0f;
    angle = 0.0f;
}

Arm::Arm() {
    list_joints = vector<Joint*>();
}