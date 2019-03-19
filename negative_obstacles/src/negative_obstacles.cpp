#include <novatel_gps_msgs/Inspva.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

//-----------------
#include <math.h>
#include <algorithm>  // std::max
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
//---------------------

/* RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class NegObstc
{
public:
  NegObstc();
  void loop_function();

private:
  ros::NodeHandle nh_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Reconst;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_inliers;
  pcl::PointCloud<pcl::PointXYZ> Cloud_check_size;
  // sensor_msgs::PointCloud2ConstPtr Cloud_msg_in;
  sensor_msgs::PointCloud2 CloudMsg_plane;
  ros::Publisher pub_plane;

  ros::Subscriber sub;
  void find_plane();

  void GetPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // pcl::fromROSMsg(cloud_msg, Cloud_Reconst);
    // Cloud_msg_in = cloud_msg;
    // pcl::fromPCLPointCloud2(*(Cloud_msg_in), *(Cloud_Reconst));

    pcl::fromROSMsg(*cloud_msg.get(), *Cloud_Reconst);
  }
};

NegObstc::NegObstc()
{
  pub_plane = nh_.advertise<sensor_msgs::PointCloud2>("cloud_plane", 100);
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Cloud_inliers.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void NegObstc::loop_function()
{
  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);
  // ros::topic::waitForMessage("/road_reconstruction",nh_);
  Cloud_check_size = (*Cloud_Reconst);
  if (Cloud_check_size.points.size() != 0)
  {
    find_plane();
    pcl::toROSMsg(*Cloud_inliers, CloudMsg_plane);
    pub_plane.publish(CloudMsg_plane);
  }
}

void NegObstc::find_plane()
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients(true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(Cloud_Reconst);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    // return (-1);
  }

  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
  //           << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

  // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
  // for (size_t i = 0; i < inliers->indices.size(); ++i)
  //   std::cerr << inliers->indices[i] << "    " << Cloud_Reconst->points[inliers->indices[i]].x << " "
  //             << Cloud_Reconst->points[inliers->indices[i]].y << " " << Cloud_Reconst->points[inliers->indices[i]].z
  //             << std::endl;

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*Cloud_Reconst, inliers->indices, *Cloud_inliers);
  ROS_WARN("Finding ransac planes");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NegObstc");
  NegObstc reconstruct;

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.loop_function();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
