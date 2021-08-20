#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

#include <iostream>
#include <math.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>

#define KATT 0.5
#define KREP 1
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)
#define VIRTUALOBS 10

using namespace std;

typedef struct
{
    float deg = 0.0;
    float dist = 99;
}BeamInfo;

class potential_field
{
public:

  potential_field();
  void calPF(sensor_msgs::LaserScan data, double goal2X, double goal2Y);
  void setPF(double kAtt, double kRep, double rhozero, int kernelSize);
  void calVelocity(double theta_now);

  double m_goalX;
  double m_goalY;
  double m_kAtt ;
  double m_kRep ;
  double m_rhoZero;
  sensor_msgs::LaserScan m_lidarData;
  double m_thetha_d;
  double m_omega;
  double m_v;
  vector<BeamInfo> minFilter;
  vector<BeamInfo> m_dist;
  BeamInfo m_virtualObsR;
  BeamInfo m_virtualObsL;
  bool m_virtualFlagR = false;
  bool m_virtualFlagL = false;
  int m_kernelSize;



private:
  void potentialSum();
  void attractive();
  bool repulsive();

  double m_repX ;
  double m_repY ;
  double m_attX ;
  double m_attY ;
  double m_potentialSumX;
  double m_potentialSumY;



};

#endif // POTENTIAL_FIELD_H
