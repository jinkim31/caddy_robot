#include "../include/caddy_navigation/mobile_kinematics.h"

mobile_kinematics::mobile_kinematics()
{
  m_radiusR = 0.5; // right wheel radius
  m_radiusL = 0.5; // left  wheel radius
  m_phi     = 1; // anguler velocity gain
  m_k_p     = 1; // linear  velocity gain
}

void mobile_kinematics::init(double radius_r, double radius_l, double phi, double k_p)
{
  m_radiusR = radius_r;
  m_radiusL = radius_l;
  m_phi     = phi;
  m_k_p     = k_p;

}

void mobile_kinematics::calKinematics(double omega, double v)
{

  Eigen::MatrixXf zeta(2,1);
  Eigen::MatrixXf cal;
  zeta(0,0) = m_phi * v;
  zeta(1,0) = m_k_p * omega;
  std::cout<<"zeta: "<<std::endl<<zeta<<std::endl<<std::endl;

  Eigen::Matrix2f A;
  A << ((double)m_radiusR/2.0), ((double)m_radiusL/2.0),
       ((double)m_radiusR/2.0), (-(double)m_radiusL/2.0);

  std::cout<<"A: "<<std::endl<<A<<std::endl;
  std::cout<<"inverse A: "<<std::endl<<A.inverse()<<std::endl;

  cal = A.inverse() * zeta;
  m_wR = cal(0,0);
  m_wL = cal(1,0);
  std::cout<<"cal: "<<std::endl<<(cal)<<std::endl<<endl;
  cout<<"omega R: "<<m_wR<<"   omega L: "<<m_wL<<endl;
}
