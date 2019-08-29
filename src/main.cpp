#include <ros/ros.h>
#include "t_serial.h"
#include "e2box_imu_9dofv4.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>

using namespace std;

e2box_imu_9dofv4 m_e2box_imu;

ros::Publisher imu_pub;
//std_msgs::Float64MultiArray imu_data;
sensor_msgs::Imu imu_data;
sensor_msgs::Imu imu_data_temp;
sensor_msgs::Imu imu_data_prev;

//ofstream outFile("data_output.txt");

int m_iImuIndex;

void OnReceiveImu(void)
{
    int n = m_e2box_imu.serial.GetLength();
    unsigned char *pBuffer = m_e2box_imu.serial.GetBuffer();

    if(n>=10){
        for(int i=0; i<n; ++i){
            m_e2box_imu.ExtractData(pBuffer[i]);
            if(m_e2box_imu.data_acquisition){
                m_e2box_imu.serial.Reset();
                m_e2box_imu.HandlingDataIMU();
                m_e2box_imu.data_acquisition = false;
                break;
            }
        }
    }
}

double gap_ang_vel_x = 0.0;
double gap_ang_vel_y = 0.0;
double gap_ang_vel_z = 0.0;
double gap_acc_x = 0.0;
double gap_acc_y = 0.0;
double gap_acc_z = 0.0;

void publishImuData(void)
{

    if(!m_e2box_imu.data_acquisition){

        imu_data.header.seq = m_e2box_imu.m_dwordCounterChecksumPass;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu_link";

        // e2box_imu_9dofV4 quaternion order : z, y, x, w
        imu_data.orientation.x = m_e2box_imu.m_dQuaternion[2];
        imu_data.orientation.y = m_e2box_imu.m_dQuaternion[1];
        imu_data.orientation.z = m_e2box_imu.m_dQuaternion[0];
        imu_data.orientation.w = m_e2box_imu.m_dQuaternion[3];

        // imu_data_temp
        imu_data_temp.angular_velocity.x = m_e2box_imu.m_dAngRate[0]*M_PI/180.0;
        imu_data_temp.angular_velocity.y = m_e2box_imu.m_dAngRate[1]*M_PI/180.0;
        imu_data_temp.angular_velocity.z = m_e2box_imu.m_dAngRate[2]*M_PI/180.0;

        imu_data_temp.linear_acceleration.x = m_e2box_imu.m_dAccel[0]*9.80665;
        imu_data_temp.linear_acceleration.y = m_e2box_imu.m_dAccel[1]*9.80665;
        imu_data_temp.linear_acceleration.z = m_e2box_imu.m_dAccel[2]*9.80665;

        // imu_data_prev
        imu_data_prev.angular_velocity.x = imu_data_temp.angular_velocity.x;
        imu_data_prev.angular_velocity.y = imu_data_temp.angular_velocity.y;
        imu_data_prev.angular_velocity.z = imu_data_temp.angular_velocity.z;

        imu_data_prev.linear_acceleration.x = imu_data_temp.linear_acceleration.x;
        imu_data_prev.linear_acceleration.y = imu_data_temp.linear_acceleration.y;
        imu_data_prev.linear_acceleration.z = imu_data_temp.linear_acceleration.z;

        // imu_data
        gap_ang_vel_x = fabs(imu_data_prev.angular_velocity.x - imu_data_temp.angular_velocity.x);
        if(gap_ang_vel_x > 0.3){
            imu_data_temp.angular_velocity.x = imu_data_prev.angular_velocity.x;
        }
        gap_ang_vel_y = fabs(imu_data_prev.angular_velocity.y - imu_data_temp.angular_velocity.y);
        if(gap_ang_vel_y > 0.3){
            imu_data_temp.angular_velocity.y = imu_data_prev.angular_velocity.y;
        }
        gap_ang_vel_z = fabs(imu_data_prev.angular_velocity.z - imu_data_temp.angular_velocity.z);
        if(gap_ang_vel_z > 0.3){
            imu_data_temp.angular_velocity.z = imu_data_prev.angular_velocity.z;
        }

        imu_data.angular_velocity.x = (imu_data_temp.angular_velocity.x + imu_data_prev.angular_velocity.x) / 2.0;
        imu_data.angular_velocity.y = (imu_data_temp.angular_velocity.y + imu_data_prev.angular_velocity.y) / 2.0;
        imu_data.angular_velocity.z = (imu_data_temp.angular_velocity.z + imu_data_prev.angular_velocity.z) / 2.0;

        gap_acc_x = fabs(imu_data_prev.linear_acceleration.x - imu_data_temp.linear_acceleration.x);
        if(gap_acc_x > 2.0){
            imu_data_temp.linear_acceleration.x = imu_data_prev.linear_acceleration.x;
        }
        gap_acc_y = fabs(imu_data_prev.linear_acceleration.y - imu_data_temp.linear_acceleration.y);
        if(gap_acc_y > 2.0){
            imu_data_temp.linear_acceleration.y = imu_data_prev.linear_acceleration.y;
        }
        gap_acc_z = fabs(imu_data_prev.linear_acceleration.z - imu_data_temp.linear_acceleration.z);
        if(gap_acc_z > 2.0){
            imu_data_temp.linear_acceleration.z = imu_data_prev.linear_acceleration.z;
        }

        imu_data.linear_acceleration.x = (imu_data_temp.linear_acceleration.x + imu_data_prev.linear_acceleration.x) / 2.0;
        imu_data.linear_acceleration.y = (imu_data_temp.linear_acceleration.y + imu_data_prev.linear_acceleration.y) / 2.0;
        imu_data.linear_acceleration.z = (imu_data_temp.linear_acceleration.z + imu_data_prev.linear_acceleration.z) / 2.0;

        /////
        /*
        imu_data.angular_velocity.x = m_e2box_imu.m_dAngRate[0]*M_PI/180.0;
        imu_data.angular_velocity.y = m_e2box_imu.m_dAngRate[1]*M_PI/180.0;
        imu_data.angular_velocity.z = m_e2box_imu.m_dAngRate[2]*M_PI/180.0;

        imu_data.linear_acceleration.x = m_e2box_imu.m_dAccel[0]*9.80665;
        imu_data.linear_acceleration.y = m_e2box_imu.m_dAccel[1]*9.80665;
        imu_data.linear_acceleration.z = m_e2box_imu.m_dAccel[2]*9.80665;
        */
        /////

        //outFile << imu_data.linear_acceleration.x << "\t" << imu_data.linear_acceleration.y << "\t" << imu_data.linear_acceleration.z << "\t" <<
        //           imu_data.angular_velocity.x << "\t" << imu_data.angular_velocity.y << "\t" << imu_data.angular_velocity.z << endl;

        imu_pub.publish(imu_data);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "e2box_imu");
    ros::NodeHandle nh;

    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param("baudrate", baudrate, 115200);

    if(!m_e2box_imu.serial.Open(const_cast<char*>(port.c_str()), baudrate)){
        cout << "device is not opened! " << endl;
        return 0;
    }    

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    ros::Rate loop_rate(100);

    while(ros::ok()){
        OnReceiveImu();
        publishImuData();

        loop_rate.sleep();     
    }

    return 0;
}
