#include "motor_imu_com.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

Moter_Imu_Com::Moter_Imu_Com(string motor_dev , string imu_dev , int Motor_baud , int Imu_baud , const eMode Mode)
{
	msMoter_SERIAL = motor_dev;
	msIMU_SERIAL = imu_dev;
    mnMotorBAUD = Motor_baud;
    mnImuBAUD = Imu_baud;
    meMode = Mode;
    cout << Rate_L << endl;

    if(meMode == Motor)
    {
        Moter_Com_Init();
    }
    else if(meMode == IMU)
    {
        Imu_Com_Init();
    }
    else if(meMode == Motor_IMU)
    {
        Moter_Com_Init();
        Imu_Com_Init();
    }
}

Moter_Imu_Com::~Moter_Imu_Com()
{
    if(meMode == Motor)
    {
        motor_sp.close();
    }
    else if(meMode == IMU)
    {
        imu_sp.close();
    }
    else if(meMode == Motor_IMU)
    {
        motor_sp.close();
        imu_sp.close();
    }
}

//初始化IMU串口配置
void Moter_Imu_Com::Imu_Com_Init()
{
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    imu_sp.setPort(msMoter_SERIAL);     //设置接口
    imu_sp.setBaudrate(mnImuBAUD);        //设置波特率
    imu_sp.setTimeout(to);

    ///打开串口
    try
    {
        imu_sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Imu port.");
        return;
    }
    //判断串口是否打开成功
    if(imu_sp.isOpen())
    {
        ROS_INFO_STREAM("Imu USB Is Opened.");
    }
    else
    {
        ROS_INFO_STREAM("Imu USB Is Not Opened.");
        return;
    }
}


//初始化Motor串口配置
void Moter_Imu_Com::Moter_Com_Init()
{
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    motor_sp.setPort(msMoter_SERIAL);     //设置接口
    motor_sp.setBaudrate(mnMotorBAUD);        //设置波特率
    motor_sp.setTimeout(to);

    ///打开串口
    try
    {
        motor_sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Motor port.");
        return;
    }
    //判断串口是否打开成功
    if(motor_sp.isOpen())
    {
        ROS_INFO_STREAM("Motor USB Is Opened.");
    }
    else
    {
        ROS_INFO_STREAM("Motor USB Is Not Opened.");
        return;
    }
}

//设置电机模式，分为扭矩模式和速度模式
void Moter_Imu_Com::Set_Motor_Model(bool torque_mode , int iqcontrol)
{
    if(torque_mode == true)
    {
        ROS_INFO_STREAM("Mode:Torque. Set electric:  " << iqcontrol << "A");
        if(iqcontrol > 32 || iqcontrol < -32)
        {
            ROS_ERROR_STREAM("Now is Torque Mode , but Set Value False");
            return;
        }
        iqcontrol = iqcontrol*2000.0/32.0;
        auto command = mMotor_command.find("torque_mode");
        uint8_t buffer[8] = {0x3E , command->second , 0x01 , 0x02 , 0x00 , 0x00 , 0x00 , 0x00};
        buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
        buffer[5] = *(uint8_t *)(&iqcontrol);
        buffer[6] = *((uint8_t *)(&iqcontrol)+1);
        buffer[7] = buffer[5] + buffer[6];
        motor_sp.write(buffer, 8);
    }
    else
    {
        ROS_INFO_STREAM("Mode:Speed. Set Speed:  " << iqcontrol << "du/s");
        if(iqcontrol > 360 || iqcontrol < -360)
        {
            ROS_ERROR_STREAM("Now is Speed Mode , but Set Value False");
            return;
        }
        iqcontrol = iqcontrol * 100;
        auto command = mMotor_command.find("speed_mode");
        uint8_t buffer[10] = {0x3E , command->second , 0x01 , 0x04 , 0x00 ,0x20 , 0x4E , 0x00 , 0x00 , 0x6E};
        buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
        buffer[5] = *(uint8_t *)(&iqcontrol);
        buffer[6] = *((uint8_t *)(&iqcontrol)+1);
        buffer[7] = *((uint8_t *)(&iqcontrol)+2);
        buffer[8] = *((uint8_t *)(&iqcontrol)+3);
        buffer[9] = buffer[5] + buffer[6] + buffer[7] + buffer[8];
        motor_sp.write(buffer, 10);
    }
    Motor_Start();
}

// 电机关闭，清除命令
void Moter_Imu_Com::Motor_Close()
{
    auto command = mMotor_command.find("close");
    uint8_t buffer[5] = {0x3E , command->second , 0x01 , 0x00 , 0x00};
    buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
    motor_sp.write(buffer, 5);
}

// 电机启动
void Moter_Imu_Com::Motor_Start()
{
    auto command = mMotor_command.find("start");
    uint8_t buffer[5] = {0x3E , command->second , 0x01 , 0x00 , 0x00};
    buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
    motor_sp.write(buffer, 5);
}

// 电机停止
void Moter_Imu_Com::Motor_Stop()
{
    auto command = mMotor_command.find("stop");
    uint8_t buffer[5] = {0x3E , command->second , 0x01 , 0x00 , 0x00};
    buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
    motor_sp.write(buffer, 5);
}

// 电机角度数据读取
void Moter_Imu_Com::Read_Moter_Angel()
{
    // 首先发送读取命令
    auto command = mMotor_command.find("read_angle");
    uint8_t buffer[5] = {0x3E , command->second , 0x01 , 0x00 , 0x00};
    buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
    motor_sp.write(buffer, 5);

    //获取缓冲区内的字节数 读取角度
    size_t num = motor_sp.available();
    if(num!=0)
    {
        uint8_t buffer[100];
        if(num > 100)
            num = 100;

        //读出数据 一次性把接收的数据全接收了
        num = motor_sp.read(buffer, num);
        // 获取时间
        double time =ros::Time::now() .toSec() ;

        uint16_t Angle;
        //16进制的方式打印到屏幕
        if(buffer[0] == 0x3E && buffer[1] == 0x94 && buffer[2] == 0x01 && buffer[3] == 0x02
        && buffer[4] == (buffer[0] + buffer[1] + buffer[2] + buffer[3]))
        {
            Angle = buffer[6]<<8 | buffer[5];
            //Angle = *(uint16_t *)Angle;
            mfMotor_Angel  = Angle*0.01;
            ROS_INFO_STREAM("Read Angle OK." << mfMotor_Angel);
            Save_Motor_Angle("/home/thiszzz/timmo_ws/Motor_Angel_Data.txt", time, mfMotor_Angel);
        }
        else
        {
            ROS_WARN_STREAM("Read Angle Error.");
            for (int i = 0; i < num; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::hex << (buffer[5] + buffer[6]) << " ";
            std::cout << endl;
        }
    }
}


void Moter_Imu_Com::Save_Motor_Angle(string file , double& time, float& Motor_Angel)
{
    if(mbCreat_File)
    {
        mMotor_ofs.open(file,ios::trunc);
        mbCreat_File = false;
    }
    mMotor_ofs.open(file,ios::out|ios::app);
    if(!mMotor_ofs.is_open())
        ROS_WARN_STREAM("Creat File False.");
    mMotor_ofs << std::setprecision(19);
    mMotor_ofs << time << "," ;
    mMotor_ofs << std::setprecision(5);
    mMotor_ofs << Motor_Angel <<'\n' ;
    mMotor_ofs.close();

}


//  读取IMU数据
void Moter_Imu_Com::Read_IMU()
{
    static bool Read_One_Byte_Flag = true ;
    static int read_flag = 0 , read_num =0;
    static bool CRC_FLAG1 = false , CRC_FLAG2 = false;

    static bool Read_A2 = false, Read_A1 = false;
    // 获取IMU时间
    Imu_Time = ros::Time::now();
    // 开始校验 校验A2
    if(Read_One_Byte_Flag)
    {
        uint8_t buffer[1];
        imu_sp.read(buffer, 1);
        read_num++;
        if(buffer[0] == 0x0E && read_flag == 0)
        {
            cout << "CRC A2 1 OK" <<endl;
            //read_flag = 1;
            read_num = 0;
            std::cout << std::hex << (buffer[0] & 0xff) << endl;
        }
        else if(read_flag == 0 && buffer[0] == 0xA2 && read_num == 1)
        {
            cout << "CRC A2 2 OK" <<endl;
            read_flag = 2;
            std::cout << std::hex << (buffer[0] & 0xff) << endl;
        }
        else if(read_flag == 2)
        {
            std::cout << std::hex << (buffer[0] & 0xff) << " ";
            if(read_num == 43)
            {
                cout << endl;
                read_flag = 0;
                read_num = 0 ;
                Read_One_Byte_Flag = false;
//                Read_A2 = false;
//                Read_A1 = false;
                CRC_FLAG1 = true;
                cout << "CRC A2 3 OK" <<endl;
            }
        }
    }
    else if(CRC_FLAG1)
    {
        uint8_t error_buffer[2];
        imu_sp.read(error_buffer, 2);
        if(error_buffer[0] == 0x0E)
        {
//            std::cout << std::hex << (error_buffer[0] & 0xff) << " ";
            if(error_buffer[1] == 0xA1)
            {
                CRC_FLAG1 = false;
                CRC_FLAG2 = true;
                Read_A1 = true;
                Read_A2 = false;
//                std::cout << std::hex << (error_buffer[1] & 0xff) << " ";
            }
            else if(error_buffer[1] == 0xA2)
            {
                CRC_FLAG1 = false;
                CRC_FLAG2 = true;
                Read_A1 = false;
                Read_A2 = true;
//                std::cout << std::hex << (error_buffer[1] & 0xff) << " ";
            }
        }
        else
        {
            cout << "CRC Error" <<endl;
            while(1);
        }
    }
    if(Read_A1 == true)
    {
        uint8_t buffer[18];
        size_t num = imu_sp.read(buffer, 18);
        if(num != 0)
        {
            int16_t _gyro_x = buffer[0] | buffer[1] << 8 ;
            int16_t _gyro_y = buffer[2] | buffer[3] << 8  ;
            int16_t _gyro_z = buffer[4] | buffer[5] << 8 ;

            int16_t _Acce_x = buffer[6] | buffer[7] << 8 ;
            int16_t _Acce_y = buffer[8] | buffer[9] << 8 ;
            int16_t _Acce_z = buffer[10] | buffer[11] << 8 ;

            Rate_x = _gyro_x * Rate_L;
            Rate_y = _gyro_y * Rate_L;
            Rate_z = _gyro_z * Rate_L;

            Acce_x = _Acce_x * Acce_L;
            Acce_y = _Acce_y * Acce_L;
            Acce_z = _Acce_z * Acce_L;

//            cout << "Rate_x:" <<  Rate_x << "  Rate_y:" <<  Rate_y << "  Rate_z:" <<  Rate_z << endl;
//            cout << "Acce_x:" <<  Acce_x << "  Acce_y:" <<  Acce_y << "  Acce_z:" <<  Acce_z << endl;
        }
//        cout << endl;
        CRC_FLAG1 = true;
    }
    else if(Read_A2 == true)
    {
        uint8_t buffer[42];
        size_t num = imu_sp.read(buffer, 42);
        if(num != 0)
        {
            int16_t _gyro_x = buffer[0] | buffer[1] << 8 ;
            int16_t _gyro_y = buffer[2] | buffer[3] << 8 ;
            int16_t _gyro_z = buffer[4] | buffer[5] << 8 ;

            int16_t _Acce_x = buffer[6] | buffer[7] << 8 ;
            int16_t _Acce_y = buffer[8] | buffer[9] << 8 ;
            int16_t _Acce_z = buffer[10] | buffer[11] << 8 ;

            Rate_x = _gyro_x * Rate_L;
            Rate_y = _gyro_y * Rate_L;
            Rate_z = _gyro_z * Rate_L;

            Acce_x = _Acce_x * Acce_L;
            Acce_y = _Acce_y * Acce_L;
            Acce_z = _Acce_z * Acce_L;
//            cout << "Rate_x:" <<  Rate_x << "  Rate_y:" <<  Rate_y << "  Rate_z:" <<  Rate_z << endl;
//            cout << "Acce_x:" <<  Acce_x << "  Acce_y:" <<  Acce_y << "  Acce_z:" <<  Acce_z << endl;
        }
//        cout << endl;
        CRC_FLAG1 = true;
    }
}

void Moter_Imu_Com::Pub_IMU()
{
}