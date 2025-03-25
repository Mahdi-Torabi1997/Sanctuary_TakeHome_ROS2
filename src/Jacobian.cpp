#include "computeJacobian.h"
#include <Eigen/Dense>

namespace IK {

    Eigen::Matrix3d computeJacobian(const RRRManipulator& robot) {
        double L1 = robot.getL1();
        double L2 = robot.getL2();
        double L3 = robot.getL3();
        double theta1 = robot.getTheta1();
        double theta2 = robot.getTheta2();
        double theta3 = robot.getTheta3();

        // Compute required sine and cosine values
        double c1 = cos(theta1), s1 = sin(theta1);
        double c12 = cos(theta1 + theta2), s12 = sin(theta1 + theta2);
        double c123 = cos(theta1 + theta2 + theta3), s123 = sin(theta1 + theta2 + theta3);

        // Compute Jacobian elements
        double J11 = -L1 * s1 - L2 * s12 - L3 * s123;
        double J12 = -L2 * s12 - L3 * s123;
        double J13 = -L3 * s123;

        double J21 = L1 * c1 + L2 * c12 + L3 * c123;
        double J22 = L2 * c12 + L3 * c123;
        double J23 = L3 * c123;

        Eigen::Matrix3d J;
        J << J11, J12, J13,
                J21, J22, J23,
                1,   1,   1;  // Orientation contribution

        return J;
    }

} // namespace IK
