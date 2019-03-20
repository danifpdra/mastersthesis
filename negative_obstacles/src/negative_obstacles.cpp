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
#include <algorithm> // std::max
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
// #include <pcl/visualization/pcl_visualizer.h>

/*visualizer*/
#include <simple_grasping/cloud_tools.h>
#include <simple_grasping/object_support_segmentation.h>
#include <simple_grasping/shape_extraction.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include "shape_msgs/SolidPrimitive.h"

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
  // pcl::PointCloud<pcl::PointXYZ>::Ptr extract_out;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_color, cloud_out_color;
  pcl::PointXYZ minPt, maxPt, origin_pt;
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose pose;

  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  double h;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  sensor_msgs::PointCloud2 CloudMsg_plane;

  ros::Publisher pub_plane;
  ros::Subscriber sub;
  ros::Publisher marker_pub;
  visualization_msgs::Marker plane_marker;

  void find_plane();

  void GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::fromROSMsg(*cloud_msg.get(), *Cloud_Reconst);
  }
};

NegObstc::NegObstc()
{
  pub_plane = nh_.advertise<sensor_msgs::PointCloud2>("cloud_plane", 100);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Cloud_inliers.reset(new pcl::PointCloud<pcl::PointXYZ>);
  coefficients.reset(new pcl::ModelCoefficients);
  inliers.reset(new pcl::PointIndices);
  cloud_in_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_out_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void NegObstc::loop_function()
{
  Cloud_check_size = (*Cloud_Reconst);
  if (Cloud_check_size.points.size() != 0)
  {
    find_plane();
    pcl::toROSMsg(*Cloud_inliers, CloudMsg_plane);
    pub_plane.publish(CloudMsg_plane);
    marker_pub.publish(plane_marker);
  }
}

void NegObstc::find_plane()
{
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

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*Cloud_Reconst, inliers->indices, *Cloud_inliers);
  // ROS_WARN("Finding ransac planes");

  pcl::copyPointCloud(*Cloud_inliers, *cloud_in_color);
  origin_pt.x = 0;
  origin_pt.y = 0;
  origin_pt.z = 0;

  plane_marker.ns = "plane";
  plane_marker.header.frame_id = "ground";
  // simple_grasping::extractUnorientedBoundingBox(*cloud_in_color,shape,pose);
  // simple_grasping::extractShape(*cloud_in_color, *cloud_out_color, shape, plane_marker.pose);

  plane_marker.type = visualization_msgs::Marker::CUBE;

  pcl::getMinMax3D(*Cloud_inliers, minPt, maxPt);
  // h = simple_grasping::distancePointToPlane(origin_pt, coefficients);

  plane_marker.pose.position.x = 20;
  // plane_marker.pose.position.y = (maxPt.y - minPt.y) / 2;
  plane_marker.pose.position.z = (maxPt.z + minPt.z) / 2;
  plane_marker.pose.orientation.w = 1;

  plane_marker.scale.x = 20;   // shape.dimensions[0];
  plane_marker.scale.y = 20;   // shape.dimensions[1];
  plane_marker.scale.z = 0.01; // shape.dimensions[2];

  plane_marker.color.r = 1;
  plane_marker.color.a = 0.8;
}

int main(int argc, char **argv)
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
