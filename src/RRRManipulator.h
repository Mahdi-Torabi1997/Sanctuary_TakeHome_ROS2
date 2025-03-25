// RRRManipulator.h
#ifndef RRRMANIPULATOR_H
#define RRRMANIPULATOR_H

class RRRManipulator {
private:
    double L1, L2, L3;
    double theta1, theta2, theta3;
public:
    RRRManipulator(double L1 = 0.3, double L2 = 0.3, double L3 = 0.1);
    void setLinkLengths(double l1, double l2, double l3);
    void setJointAngles(double t1, double t2, double t3);
    double getL1() const;
    double getL2() const;
    double getL3() const;
    double getTheta1() const;
    double getTheta2() const;
    double getTheta3() const;
};

#endif // RRRMANIPULATOR_H
