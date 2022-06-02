#ifndef MOTOR_IMU_COM_H
#define MOTOR_IMU_COM_H

#include <iostream>
#include <string>
#include <unordered_map>
#include <serial/serial.h>
#include <fstream>
#include <ros/ros.h>

using namespace std;

class Moter_Imu_Com
{
    public:

    enum eMode{
        Motor=0,
        IMU=1,
        Motor_IMU=2,
    };


    Moter_Imu_Com(string motor_dev , string imu_dev , int Motor_baud , int Imu_baud , const eMode Mode);
    ~Moter_Imu_Com();

    //串口配置
    void Moter_Com_Init();
    void Imu_Com_Init();

    //电机相关控制
    void Set_Motor_Model(bool torque_mode , int iqcontrol);
    void Read_Moter_Angel();
    void Motor_Close();
    void Motor_Start();
    void Motor_Stop();
    void Save_Motor_Angle(string file , double & time, float& Motor_Angel);

    //IMU
    void Read_IMU();
    void Pub_IMU();

    public:
    float mfMotor_Angel;  //0~360
    double Rate_x , Rate_y , Rate_z;
    double Acce_x , Acce_y , Acce_z;
    eMode meMode;
    ros::Time Imu_Time;

    private:
    unordered_map<string,uint8_t> mMotor_command =
                           { {"close" , 0x80} ,         {"stop" , 0x81} ,
                             {"start", 0x88} ,          {"read_angle" , 0x94} ,
                             {"set_zero" , 0x19} ,      {"torque_mode" , 0xA1} ,
                             {"speed_mode" , 0xA2}};
    // CRC校验 
    char mcCRC_Byte = 0x3E;
    char mcMotor_ID = 0x00;
    // 串口相关配置
    string msMoter_SERIAL =  "/dev/ttyUSB0";
    string msIMU_SERIAL =  "/dev/ttyUSB1";
    int  mnMotorBAUD = 115200;
    int  mnImuBAUD = 1000000;
    serial::Serial motor_sp , imu_sp;

    //
    ofstream  mMotor_ofs;
    bool mbCreat_File = true;

    //IMU系数
    double Rate_L = 0.00048828125;
    double Acce_L = 0.009525;

};


#endif