// InverseKinematics.h
#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "RRRManipulator.h"

namespace IK {
    bool inverseKinematicsAlgebraic(const RRRManipulator &robot, double x, double y, double phi,
                                    double &theta1, double &theta2, double &theta3);
}

#endif // INVERSEKINEMATICS_H
