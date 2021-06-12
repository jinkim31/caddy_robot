#ifndef MOBILE_KINEMATICS_H
#define MOBILE_KINEMATICS_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;

class mobile_kinematics
{
public:
  mobile_kinematics();
  void init(double radius_r, double radius_l, double phi, double k_p);
  void calKinematics(double omega, double v);

  double m_radiusR;
  double m_radiusL;
  double m_phi;
  double m_k_p;
  double m_wL;
  double m_wR;
};

#endif // MOBILE_KINEMATICS_H
