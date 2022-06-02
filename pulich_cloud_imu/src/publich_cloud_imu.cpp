#include<iostream>
#include<string>
#include<fstream>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions//pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include<sensor_msgs/Imu.h>

using namespace std;

int main(int argc ,char ** argv)
{
    std::string topic,path,frame_id;
    int pointcloud_hz=5;

    ros::init(argc , argv , "publish_pointcloud_imu");
    ros::NodeHandle nh;

    nh.param<std::string>("path", path, "/home/crp/catkin_ws/test.pcd");
	nh.param<std::string>("frame_id", frame_id, "camera");
	nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("pointcloud_hz", pointcloud_hz, 5);

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("FM_PointCloud", 10); 
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu> ("FM_IMU", 100); 

	pcl::PointCloud<pcl::PointXYZ> cloud;  
	sensor_msgs::PointCloud2 fw_pointcloud;  
    // 加载PCD文件里面的点云数据
	pcl::io::loadPCDFile (path, cloud);  
    // 转换成ROS下的数据类型 最终通过topic发布
	pcl::toROSMsg(cloud,fw_pointcloud);

	fw_pointcloud.header.stamp=ros::Time::now();
	fw_pointcloud.header.frame_id  =frame_id;

	cout<<"path = "<<path<<endl;
	cout<<"frame_id = "<<frame_id<<endl;
	cout<<"topic = "<<topic<<endl;
	cout<<"pointcloud_hz = "<<pointcloud_hz<<endl;

	ros::Rate loop_rate(pointcloud_hz);  

	while (ros::ok())  
	{  
		pcl_pub.publish(fw_pointcloud);  
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	return 0; 
    
}