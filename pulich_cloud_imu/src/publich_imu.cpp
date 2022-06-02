#include<iostream>
#include<string>
#include<vector>
#include<fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include<sensor_msgs/Imu.h>

using namespace std;

int main(int argc ,char ** argv)
{
    ros::init(argc , argv , "publish_imu");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu> ("FM_IMU", 100); 
	ros::Rate loop_rate(1000);  

	// 读取IMU数据
	ifstream read_imu;
	read_imu.open(argv[1]);
	if(!read_imu.is_open())
		ROS_ERROR("Can't  Open Imu File");
	vector<double> vTimeStamps;
	vector<vector<double>> vAcc, vGyro,vQura;

    vTimeStamps.reserve(500000);
    vAcc.reserve(500000);
    vGyro.reserve(500000);
	vQura.reserve(500000);

	ROS_INFO("Star Read IMU");
	while (!read_imu.eof())
	{
		string s;
        getline(read_imu,s);
        if (s[0] == 'S')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[11];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) 
			{
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
			// 读取最后一个数据
            item = s.substr(0, pos);
            data[10] = stod(item);

            vTimeStamps.push_back(data[0]);
            vAcc.push_back({data[1],data[2],data[3]});
            vGyro.push_back({data[4],data[5],data[6]});
			vQura.push_back({data[7],data[8],data[9],data[10]});
        }
	}
	read_imu.close();
	ROS_INFO("Read IMU OK!");
	ROS_INFO("IMU DATA SIZE:%ld", vTimeStamps.size() );

	long int Imu_Num  = 0;
	while (ros::ok())  
	{  
		//ROS_INFO_STREAM_THROTTLE(0.5, "Message print every 0.5s"); 
		ROS_INFO("Imu_Num:%ld/%ld", Imu_Num,vTimeStamps.size() );
		if(Imu_Num > (vTimeStamps.size() -1) )
		{
			ROS_WARN("Publish IMU Data OK");
			return 0;
		}
		sensor_msgs::Imu imu_data;
		imu_data.header.stamp = ros::Time::now();
		imu_data.header.frame_id = "imu";
		//四元数位姿,所有数据设为固定值
		imu_data.orientation.x = vQura[Imu_Num][0];
		imu_data.orientation.y = vQura[Imu_Num][1];
		imu_data.orientation.z = vQura[Imu_Num][2];
		imu_data.orientation.w = vQura[Imu_Num][3];
		//线加速度
		imu_data.linear_acceleration.x = vAcc[Imu_Num][0]; 
		imu_data.linear_acceleration.y = vAcc[Imu_Num][1];
		imu_data.linear_acceleration.z = vAcc[Imu_Num][2];
		//角速度
		imu_data.angular_velocity.x = vGyro[Imu_Num][0]; 
		imu_data.angular_velocity.y = vGyro[Imu_Num][1]; 
		imu_data.angular_velocity.z = vGyro[Imu_Num][2];

		imu_pub.publish(imu_data);  

		Imu_Num ++;
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	return 0; 
    
}