#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <novatel_gps_msgs/Inspva.h>

//-----------------
#include <math.h>
#include <algorithm>  // std::max
#include <cmath>
#include <fstream>
#include <iostream>
//---------------------

#include <pcl/sample_consensus/sac_model_plane.h>

using namespace laser_assembler;
// using namespace pcl;

class RoadReconst
{
public:
  // publicadores
  RoadReconst();

  // Filter the cloud based on recieved data
  void loop_function();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_cloud0, pub_cloud3, pub_cloudTotal, pub_road_rec;
  pcl::PointCloud<pcl::PointXYZ> CloudXYZ_LD0, CloudXYZ_LD1, CloudXYZ_LD2, CloudXYZ_LD3, CloudXYZ_Total;
  sensor_msgs::PointCloud2 CloudMsg_LD0, CloudMsg_LD3, CloudMsg_Total, Cloud_Reconst;

  pcl::PointCloud<pcl::PointXYZ> Inter1, Inter2, InterM, RoadRec;

  void getCloudsFromSensors();
};

RoadReconst::RoadReconst()
{
  // Publish clouds
  sensor_msgs::PointCloud2 CloudMsg_Simple;
  pub_road_rec = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("road_reconstruction", 1);
  pub_cloudTotal = nh_.advertise<sensor_msgs::PointCloud2>("cloud_Total", 100);

}

void RoadReconst::loop_function()
{
  // Buscar os parametros

  getCloudsFromSensors();
  pcl::toROSMsg(CloudXYZ_Total, CloudMsg_Total);
  pub_cloudTotal.publish(CloudMsg_Total);
}

void RoadReconst::getCloudsFromSensors()
{
  //----------Assemble_0-------------------------------------
  ros::service::waitForService("assemble_scans0");
  ros::ServiceClient client_0 = nh_.serviceClient<AssembleScans2>("assemble_scans0");
  AssembleScans2 srv_0;
  srv_0.request.begin = ros::Time(0, 0);
  srv_0.request.end = ros::Time::now();
  if (client_0.call(srv_0))
  {
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
    pcl::fromROSMsg(srv_3.response.cloud, CloudXYZ_LD3);
  }
  else
    printf("Service call failed\n");

  //------------------Assemble all an publish---------------

  Inter1 = CloudXYZ_LD0 + CloudXYZ_LD1;  // 1-2 | first 2
  Inter2 = CloudXYZ_LD2 + CloudXYZ_LD3;  // 3-4 | last 2
  InterM = Inter1 + CloudXYZ_LD2;  // first 3 clouds
  RoadRec = Inter1 + Inter2;       // all clouds
  pcl::toROSMsg(InterM, Cloud_Reconst);
  pub_road_rec.publish(Cloud_Reconst);

}


pcl::SampleConsensusModelPlane< PointT >::SampleConsensusModelPlane

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RoadReconst");
  RoadReconst reconstruct;

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.loop_function();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
