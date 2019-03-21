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
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
//---------------------

/* RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
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

/*octomap*/
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class NegObstc
{
public:
  NegObstc();
  void loop_function();

private:
  ros::NodeHandle nh_;

  /*pcl*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Reconst, Cropped_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_inliers, Cloud_negative;
  pcl::PointCloud<pcl::PointXYZ> Cloud_check_size, Cloud_check_sqr;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr extract_out;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_color, cloud_out_color;
  pcl::PointXYZ minPt, maxPt, minR, maxR;
  pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_reconst;
  /*others*/
  double h,pace;
  size_t count_points;
  // Eigen::Vector3f min_pt, max_pt;

  /*publishers and subscribers*/
  ros::Publisher pub_plane;
  ros::Subscriber sub;
  ros::Publisher marker_pub;
  // ros::Publisher octomap_publisher;
  /*messages*/
  visualization_msgs::Marker plane_marker;
  sensor_msgs::PointCloud2 CloudMsg_plane;
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose pose;
  // octomap_msgs::Octomap octree_msg;

  void find_plane();
  void octree_builder();

  void GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::fromROSMsg(*cloud_msg.get(), *Cloud_Reconst);
  }
};

/**
 * @brief Construct a new Neg Obstc:: Neg Obstc object
 *
 */
NegObstc::NegObstc()
{
  /*publishers and subscribers*/
  pub_plane = nh_.advertise<sensor_msgs::PointCloud2>("cloud_plane", 100);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);
  // octomap_publisher = nh_.advertise<octomap_msgs::Octomap>("octree_reconst", 1);
  /*initialize pointers*/
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

/**
 * @brief function to calculate and draw the medium road plane
 *
 */
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
  }

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*Cloud_Reconst, inliers->indices, *Cloud_inliers);
  pcl::getMinMax3D(*Cloud_inliers, minPt, maxPt);
  h = minPt.z + (maxPt.z - minPt.z) / 2;
  // ROS_WARN("Finding ransac planes");
  // pcl::copyPointCloud(*Cloud_inliers, *cloud_in_color);
  // simple_grasping::extractUnorientedBoundingBox(*cloud_in_color,shape,pose);
  // simple_grasping::extractShape(*cloud_in_color, *cloud_out_color, shape, plane_marker.pose);
  // h = simple_grasping::distancePointToPlane(origin_pt, coefficients);

  plane_marker.ns = "plane";
  plane_marker.header.frame_id = "ground";
  plane_marker.type = visualization_msgs::Marker::CUBE;
  plane_marker.pose.position.x = 10;
  // plane_marker.pose.position.y = (maxPt.y - minPt.y) / 2;
  plane_marker.pose.position.z = h;
  plane_marker.pose.orientation.w = 1;
  plane_marker.scale.x = 20;   // shape.dimensions[0];
  plane_marker.scale.y = 20;   // shape.dimensions[1];
  plane_marker.scale.z = 0.01; // shape.dimensions[2];
  plane_marker.color.r = 1;
  plane_marker.color.a = 0.8;
}

void NegObstc::octree_builder()
{
  // float voxelSize = 0.01f;  // voxel resolution
  // pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(voxelSize);
  // octree.setInputCloud(Cloud_Reconst);
  // octree.addPointsFromInputCloud();
  // octree.getBoundingBox();
  // getVoxelBounds(octree, min_pt, max_pt);

  // // octomap_msgs::binaryMapToMsg(octree, octree_msg);
  // // octomap_publisher.publish(octree_msg);

  // pcl::VoxelGrid<pcl::PointXYZ> vg;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredVox(new pcl::PointCloud<pcl::PointXYZ>);
  // vg.setInputCloud(Cloud_Reconst);
  // vg.setLeafSize(0.06f, 0.06f, 0.06f);
  // vg.filter(*cloud_filteredVox);

  /* TENTATIVA DE DIVISÃO ESPACIAL*/
  pcl::getMinMax3D(*Cloud_Reconst, minR, maxR);
  pace=(maxR.y - minR.y) / 100;
  for (int Y = minR.y; Y <= maxR.y-pace; Y = Y + pace)
  {
    boxFilter.setMin(Eigen::Vector4f(minR.x, Y, h, 1.0));
    boxFilter.setMax(Eigen::Vector4f(minR.x + 10, Y+pace, h + 10, 1.0));
    boxFilter.setInputCloud(Cloud_Reconst);
    boxFilter.filter(*Cropped_cloud);
    Cloud_check_sqr = (*Cropped_cloud);
    count_points = Cloud_check_sqr.points.size();
    ROS_WARN("Number of points in square: %d", count_points);
    if ()
    {
    }
  }
}

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
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
