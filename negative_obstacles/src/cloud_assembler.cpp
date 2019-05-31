#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

//-----------------
#include <math.h>
#include <algorithm> // std::max
#include <cmath>
#include <fstream>
#include <iostream>
//---------------------

using namespace laser_assembler;
// using namespace pcl;

class CloudAssemb
{
public:
  // publicadores
  CloudAssemb();
//   int writeCount;

  pcl::PointCloud<pcl::PointXYZ> RoadRec, acum_cloud;

  // Filter the cloud based on recieved data
  void loop_function();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_cloudAcum;
  pcl::PointCloud<pcl::PointXYZ> CloudXYZ_LD0, CloudXYZ_LD1, CloudXYZ_LD2, CloudXYZ_LD3;
  sensor_msgs::PointCloud2 CloudMsg_Acum;

  pcl::PointCloud<pcl::PointXYZ> Inter1, Inter2;

  void getCloudsFromSensors();
};


CloudAssemb::CloudAssemb()
{
  pub_cloudAcum = nh_.advertise<sensor_msgs::PointCloud2>("Cloud_Acum", 100);
}

void CloudAssemb::loop_function()
{
  getCloudsFromSensors();
  pcl::toROSMsg(RoadRec, CloudMsg_Acum);
  pub_cloudAcum.publish(CloudMsg_Acum);
}

void CloudAssemb::getCloudsFromSensors()
{
  //----------Assemble_0-------------------------------------
  ros::service::waitForService("assemble_scans0");
  ros::ServiceClient client_0 = nh_.serviceClient<AssembleScans2>("assemble_scans0");
  AssembleScans2 srv_0;
  srv_0.request.begin = ros::Time(0, 0);
  srv_0.request.end = ros::Time::now();
  if (client_0.call(srv_0))
  {
    // pub_cloud0.publish(srv.response.cloud);
    pcl::fromROSMsg(srv_0.response.cloud, CloudXYZ_LD0);
  }
  else
  {
    printf("Service call failed\n");
  }

  //----------Assemble_1-------------------------------------
  ros::service::waitForService("assemble_scans1");
  ros::ServiceClient client_1 = nh_.serviceClient<AssembleScans2>("assemble_scans1");
  AssembleScans2 srv_1;
  srv_1.request.begin = ros::Time(0, 0);
  srv_1.request.end = ros::Time::now();
  if (client_1.call(srv_1))
  {
    // pub_cloud0.publish(srv.response.cloud);
    pcl::fromROSMsg(srv_1.response.cloud, CloudXYZ_LD1);
  }
  else
  {
    printf("Service call failed\n");
  }

  //----------Assemble_2-------------------------------------

  ros::service::waitForService("assemble_scans2_");
  ros::ServiceClient client_2 = nh_.serviceClient<AssembleScans2>("assemble_scans2_");
  AssembleScans2 srv_2;
  srv_2.request.begin = ros::Time(0, 0);
  srv_2.request.end = ros::Time::now();
  if (client_2.call(srv_2))
  {
    // pub_cloud0.publish(srv_2.response.cloud);
    pcl::fromROSMsg(srv_2.response.cloud, CloudXYZ_LD2);
  }
  else
  {
    printf("Service call failed\n");
  }

  //----------Assemble_3-------------------------------------
  ros::service::waitForService("assemble_scans3");
  ros::ServiceClient client_3 = nh_.serviceClient<AssembleScans2>("assemble_scans3");
  AssembleScans2 srv_3;
  srv_3.request.begin = ros::Time(0, 0);
  srv_3.request.end = ros::Time::now();
  if (client_3.call(srv_3))
  {
    // pub_cloud3.publish(srv3.response.cloud);
    pcl::fromROSMsg(srv_3.response.cloud, CloudXYZ_LD3);
  }
  else
    printf("Service call failed\n");

  //------------------Assemble all an publish---------------

  Inter1 = CloudXYZ_LD0 + CloudXYZ_LD1; // 1-2 | first 2
  Inter2 = CloudXYZ_LD2 + CloudXYZ_LD3; // 3-4 | last 2

  RoadRec = Inter1 + Inter2;      // all clouds

//   pcl::PointXYZ minPt, maxPt;
//   pcl::getMinMax3D(Inter1, minPt, maxPt);
//   // ROS_INFO("Minim:%f  Maxim:%f", minPt.z, maxPt.z);

//   /*trying to set total cloud*/
//   if (acum_cloud.points.size() == 0)
//   {
//     acum_cloud = RoadRec;
//   }
//   else
//   {
//     acum_cloud = acum_cloud + RoadRec;
//   }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CloudAssemb");
  CloudAssemb reconstruct;

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.loop_function();

    ros::spinOnce();
    rate.sleep();
  }

//   if ((acum_cloud.points.size() != 0))
//   {
//     writeCount++;
//     char filename[100];
//     sprintf(filename, "/media/daniela/Dados/pcd_files/test_%d.pcd", writeCount);
//     // pcl::io::savePCDFileASCII(filename, acum_cloud);
//     // pcl::io::savePCDFile("thisisatest.pcd", acum_cloud, true);
//     // ROS_INFO("Saved %lu points in point cloud", acum_cloud.points.size());
//   }

  return 0;
}
