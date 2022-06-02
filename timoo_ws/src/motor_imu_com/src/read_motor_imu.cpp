//
// Created by thiszzz on 2022/5/26.
//
#include <ros/ros.h>
#include <iostream>
#include <string>
#include "motor_imu_com.h"
#include <fstream>
#include<sensor_msgs/Imu.h>

int main(int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_moter_imu");
    //声明节点句柄
    ros::NodeHandle n("~");
    //读取参数
    string Motor_dev , Imu_dev;
    int Motor_Speed , Motor_baud , Imu_baud;

    n.param<string>("Motor_dev",Motor_dev,"/dev/ttyUSB0");
    n.param<string>("Imu_dev", Imu_dev,"/dev/ttyUSB0");
    n.param<int>("Motor_Speed", Motor_Speed,15);
    n.param<int>("Motor_baud", Motor_baud,115200);
    n.param<int>("Imu_baud", Imu_baud,1000000);

    //-------------------Motor
    Moter_Imu_Com Motor_IMU(Motor_dev , Imu_dev , Motor_baud , Imu_baud , Moter_Imu_Com::Motor);
    if(Motor_IMU.meMode == Moter_Imu_Com::Motor || Motor_IMU.meMode == Moter_Imu_Com::Motor_IMU)
    {
        Motor_IMU.Set_Motor_Model(false , Motor_Speed);
        //Motor.Motor_Stop();
    }
    //--------------------IMU
    ros::Publisher timoo_imu_pub = n.advertise<sensor_msgs::Imu> ("TM_IMU", 1000);
    ros::Rate loop_rate(2000);

    int Motor_Freq_count = 0;
    while(ros::ok())
    {
        if(Motor_IMU.meMode == Moter_Imu_Com::Motor || Motor_IMU.meMode == Moter_Imu_Com::Motor_IMU)
        {
            Motor_Freq_count++ ;
            // 285.7Hz
            if(Motor_Freq_count == 7)
            {
                Motor_IMU.Read_Moter_Angel();
                Motor_Freq_count = 0;
            }
        }

        if(Motor_IMU.meMode == Moter_Imu_Com::IMU || Motor_IMU.meMode == Moter_Imu_Com::Motor_IMU)
        {
            Motor_IMU.Read_IMU();
            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = Motor_IMU.Imu_Time;
            imu_data.header.frame_id = "timoo";
            //四元数位姿,所有数据设为固定值
            imu_data.orientation.x = 0;
            imu_data.orientation.y = 0;
            imu_data.orientation.z = 0;
            imu_data.orientation.w = 1;
            //角速度
            imu_data.angular_velocity.x = Motor_IMU.Rate_x;
            imu_data.angular_velocity.y = Motor_IMU.Rate_y;
            imu_data.angular_velocity.z = Motor_IMU.Rate_z;
            //线加速度
            imu_data.linear_acceleration.x = Motor_IMU.Acce_x;
            imu_data.linear_acceleration.y = Motor_IMU.Acce_y;
            imu_data.linear_acceleration.z = Motor_IMU.Acce_z;

            timoo_imu_pub.publish(imu_data);
        }


        loop_rate.sleep();
    }
    ROS_INFO_STREAM("ROS EXIT");
    if(Motor_IMU.meMode == Moter_Imu_Com::Motor || Motor_IMU.meMode == Moter_Imu_Com::Motor_IMU)
        Motor_IMU.Motor_Close();
    return 0;
}