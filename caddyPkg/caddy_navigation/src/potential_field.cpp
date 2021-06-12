#include "../include/caddy_navigation/potential_field.h"

bool cmp(const BeamInfo &p1, const BeamInfo &p2){
    if(p1.deg < p2.deg){
        return true;
    }
    else if(p1.deg == p2.deg){
        return p1.deg < p2.deg;
    }
    else{
        return false;
    }
}


potential_field::potential_field()
{
    m_kAtt = KATT;
    m_kRep = KREP;
    m_rhoZero = 1;
    m_kernelSize  = 5;
}

void potential_field::setPF(double kAtt, double kRep, double rhozero, int kernelSize)
{
    m_kAtt        = kAtt;
    m_kRep        = kRep;
    m_rhoZero     = rhozero;
    m_kernelSize  = kernelSize;
}

void potential_field::calPF(sensor_msgs::LaserScan data, double goal2X, double goal2Y)
{
    m_goalX = goal2X;
    m_goalY = goal2Y;

    m_repX = 0.0;
    m_repY = 0.0;
    m_attX = 0.0;
    m_attY = 0.0;

    m_lidarData = data;
    attractive();
    repulsive();
    potentialSum();

}

void potential_field::potentialSum()
{

    m_potentialSumX = m_attX + m_repX;
    m_potentialSumY = m_attY + m_repY;
    cout<<"m_attX: "<<m_attX<<"       m_attY: "<<m_attY<<endl;
    cout<<"m_repX: "<<m_repX<<"       m_repY: "<<m_repY<<endl;
    cout<<"m_potentialSumX: "<<m_potentialSumX<<"       m_potentialSumY: "<<m_potentialSumX<<endl;
}

void potential_field::attractive()
{
    // 거리센서
    cout<<"m_goalX: "<<m_goalX<< "          m_goalY: "<< m_goalY<<endl;
    m_attX = -m_kAtt*(-m_goalX);
    m_attY = -m_kAtt*(-m_goalY);
}

bool potential_field::repulsive()
{
    m_dist.clear();


    // deg: -180 ~ 180
    int count = m_lidarData.scan_time / m_lidarData.time_increment;

    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(m_lidarData.angle_min + m_lidarData.angle_increment * i);

        if(degree < 0 ){
            degree = (180 + degree);
        }
        else{
            degree = -(180 - degree);
        }

        BeamInfo data;
        data.deg = degree;
        data.dist = m_lidarData.ranges[i] >= m_lidarData.range_max ? m_lidarData.range_max : m_lidarData.ranges[i];

        m_dist.push_back(data);
    }

    sort(m_dist.begin(),m_dist.end(),cmp);


    // minFilter
    vector<BeamInfo> beamdata;
    bool active = false;
    BeamInfo minValue;
    for(int i = 0; i < m_dist.size()-m_kernelSize; i++)
    {
        int activeCnt = 0;
        int inactiveCnt = 0;
        minValue.dist  = 99.0;

        for(int k = 0; k < m_kernelSize; k++) // kernel 만들고 최소 값 구하기
        {
            //                      cout<<"kernel["<<k<<"]: "<<kernel.dist<<endl;
            BeamInfo kernel = m_dist[i+k];
            kernel = m_dist[i+k];
            if(kernel.dist > m_rhoZero)
            {
                inactiveCnt++;
                continue;
            }
            else
                activeCnt++;


            if(kernel.dist < minValue.dist)
                minValue = kernel;
        }

        if(active == true)
        {
            if(inactiveCnt >= m_kernelSize)
            {
                active = false;
            }
            else
            {
                if(minValue.dist < beamdata[beamdata.size()-1].dist)
                {
                    beamdata[beamdata.size()-1] = minValue;
                }
            }
        }
        else
        {
            if(activeCnt >= m_kernelSize)
            {
                active = true;
                BeamInfo newData;
                newData.deg = 0;
                newData.dist = 16;
                beamdata.push_back(newData);
            }
        }
    }

    minFilter.clear();

    static bool obsR = false;
    static bool obsL = false;
    bool cObsR = false;
    bool cObsL = false;
    static bool tesmpObsR =false;
    static bool tesmpObsL =false;

    cout<<"m_kRep: "<<m_kRep<<endl;

    for(int i = 0; i < beamdata.size(); i++)
    {
        double rho = beamdata[i].dist;
        if(rho < m_rhoZero && rho != 0.0)
        {
            cout<<" ************************ beamdata[i].deg: "<<beamdata[i].deg<< " ************************ "<<endl;
            //            if(beamdata[i].deg < -45)
            //                cObsR = true;

            //            if(beamdata[i].deg > 45)
            //                cObsL = true;

            //            if(beamdata[i].deg > (109 - VIRTUALOBS) && beamdata[i].deg < 109)
            //            {
            //                m_virtualObsL = beamdata[i];
            //                obsL = true;
            //            }
            //            else if(beamdata[i].deg < (-103 + VIRTUALOBS) && beamdata[i].deg>-103)
            //            {
            //                m_virtualObsR = beamdata[i];
            //                obsR = true;
            //            }


            rho -=0.1; // it is o
            minFilter.push_back(beamdata[i]);
            double product = m_kRep*(1/rho - 1/m_rhoZero)*(1/(rho*rho*rho));

//                        m_repX -= product*rho*sin(DEG2RAD(beamdata[i].deg));
//                        m_repY += product*rho*cos(DEG2RAD(beamdata[i].deg));

//                        m_repX += product*rho*sin(DEG2RAD(beamdata[i].deg));
//                        m_repY += product*rho*cos(DEG2RAD(beamdata[i].deg));

            m_repX -= (double)product*(double)rho*cos(DEG2RAD(beamdata[i].deg));
            m_repY -= (double)product*(double)rho*sin(DEG2RAD(beamdata[i].deg));
        }
    }
    cout<<"cObsL: "<<cObsL<<"       tesmpObsL: "<<tesmpObsL<<"      obsL: "<<obsL<<endl;
    cout<<"cObsR: "<<cObsR<<"       tesmpObsR: "<<tesmpObsR<<"      obsR: "<<obsL<<endl;
    //    // virtual Obstacle
    //    {
    //        double rho = m_virtualObsR.dist;

    //        if(/*cObsR == false && */tesmpObsR == true && obsR == false && rho < m_rhoZero)
    //        {
    //            double product = m_kRep*(1/rho - 1/m_rhoZero)*(1/(rho*rho*rho));
    //            m_repX += product*rho*sin(DEG2RAD(m_virtualObsR.deg));
    //            m_repY += product*rho*cos(DEG2RAD(m_virtualObsR.deg));
    //            m_virtualFlagR = true;
    //        }
    //        else
    //            m_virtualFlagR = false;
    ////            m_virtualObsR.dist = 15;

    //        rho = m_virtualObsL.dist;

    //        if(/*cObsL == false && */tesmpObsL == true && obsL == false && rho < m_rhoZero)
    //        {
    //            double product = m_kRep*(1/rho - 1/m_rhoZero)*(1/(rho*rho*rho));
    //            m_repX += product*rho*sin(DEG2RAD(m_virtualObsL.deg));
    //            m_repY += product*rho*cos(DEG2RAD(m_virtualObsL.deg));
    //            m_virtualFlagL = true;
    //        }
    //        else
    //            m_virtualFlagL = false;
    ////            m_virtualObsL.dist = 15;
    //    }


    //    tesmpObsR = obsR;
    //    tesmpObsL = obsL;

}

// theta_now is radian unit
void potential_field::calVelocity(double theta_now) // angular velocity and linear velocity
{
    double  dUx = (m_potentialSumX/(sqrt(pow(m_potentialSumX, 2) + pow(m_potentialSumY, 2))));
    double  dUy = (m_potentialSumY/(sqrt(pow(m_potentialSumX, 2) + pow(m_potentialSumY, 2))));

    cout<<"dUx: "<<dUx<<"     dUy"<<dUy<<endl;
    m_thetha_d = atan2(dUy, dUx);
    m_omega = (m_thetha_d - theta_now);                    // 회전하는 속도
    m_v = (dUx*cos(theta_now) + dUy*sin(theta_now));   // 직진하는 속도
}




//double rhozero = 1.5;
//vector<double> dist(181);
//std::srand(std::time(nullptr));

//// deg: -90 ~ 90
//for(auto& beam : _oLidar2D)
//{
//    double noise = (float)(rand())/((float)(RAND_MAX/(1)));
//    double beamdist = beam.second->_dRange;
//    if(noise < 0.05) // max
//    {
//        beamdist = SENSOR_DIST;
//    }
//    else if (noise < 0.1) // random
//    {
//        double random_noise = ((float)(rand())/((float)(RAND_MAX/(1)))) * SENSOR_DIST;
//        beamdist = random_noise;
//    }
//    else // gaussian
//    {
//        double g_noise = _gSampler.Generate(0.0, 0.01);
//        double noise_data = ((double)beamdist + g_noise);
//        beamdist =  noise_data < 0 ? 0.01 : noise_data;
//    }
//    dist[beam.second->_oInfo.deg + 90] =  (double)beamdist;
//    Beamdist[beam.second->_oInfo.deg + 90] =  (double)beamdist;
//}

//vector<BeamInfo_> beamdata;
//bool active = false;

//for(int i = 0; i < dist.size() - 5; i++)
//{
//    int activeCnt = 0;
//    int inactiveCnt = 0;

//    double minValue = 99.0;

//    double minIdx = 0.0;
//    double kernel[5] = {dist[i+0],
//                        dist[i+1],
//                        dist[i+2],
//                        dist[i+3],
//                        dist[i+4]};



//    for(int k = 0; k < 5; k++)
//    {
//        if(kernel[k] > rhozero)
//        {
//            inactiveCnt++;
//            continue;
//        }
//        else
//            activeCnt++;


//        if(kernel[k] < minValue)
//        {
//            minValue = kernel[k];
//            minIdx = i + k -90.0;
//        }
//    }

//    if(active == true)
//    {
//        if(inactiveCnt >= 5)
//        {
//            active = false;
//        }
//        else
//        {
//            if(minValue < beamdata[beamdata.size()-1].dist)
//            {
//                //                    cout<<"minValue: "<<minValue<<endl;
//                //                    cout<<"beamdata[beamdata.size()-1].dist: "<<beamdata[beamdata.size()-1].dist;
//                //                    cout<<"         beamdata[beamdata.size()-1].deg: "<<beamdata[beamdata.size()-1].deg<<endl;
//                beamdata[beamdata.size()-1].deg = minIdx;
//                beamdata[beamdata.size()-1].dist = minValue;
//            }
//        }
//    }
//    else
//    {
//        if(activeCnt >= 5)
//        {
//            active = true;
//            BeamInfo_ newData;
//            newData.deg = 0.0;
//            newData.dist = 10.0;
//            beamdata.push_back(newData);
//        }
//    }
//}
////    cout<<"beamdata.size(): "<<beamdata.size()<<endl;
//for(int i = 0; i < beamdata.size(); i++)
//{
//    double rho = beamdata[i].dist;
//    if(rho < rhozero)
//    {
//        rho -= 0.1;
//        drawMinN.push_back(beamdata[i].deg);

//        double product = kRep*(1/rho - 1/rhozero)*(1/(rho*rho*rho));

//        repX -= product*rho*cos(deg2rad(beamdata[i].deg));
//        repY -= product*rho*sin(deg2rad(beamdata[i].deg));
//    }
//}



