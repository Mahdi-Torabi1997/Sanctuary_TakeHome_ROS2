#ifndef FORWARDKINEMATICS_HPP
#define FORWARDKINEMATICS_HPP

#include "RRRManipulator.h"
#include <Eigen/Dense>

// Computes the homogeneous transformation matrix for the RRR manipulator.
// The transforms used are:
//
// T01 = [ c1 -s1 0 0; s1 c1 0 0; 0 0 1 0; 0 0 0 1 ]
// T12 = [ c2 -s2 0 L1; s2 c2 0 0; 0 0 1 0; 0 0 0 1 ]
// T23 = [ c3 -s3 0 L2; s3 c3 0 0; 0 0 1 0; 0 0 0 1 ]
// T3ee = [ 1 0 0 L3; 0 1 0 0; 0 0 1 0; 0 0 0 1 ]
//
// Overall: T0ee = T01*T12*T23*T3ee.
Eigen::Matrix4d computeForwardKinematics(const RRRManipulator &robot);

// Extracts the end-effector pose (x, y, phi) from T0ee.
void getEndEffectorPose(const RRRManipulator &robot, double &x, double &y, double &phi);

#endif // FORWARDKINEMATICS_HPP
