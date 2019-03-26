#include <novatel_gps_msgs/Inspva.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

//-----------------
#include <math.h>
#include <algorithm>  // std::max
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
#include <pcl/common/transforms.h>
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

// /*octomap*/
// #include <octomap_msgs/GetOctomap.h>
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap_ros/conversions.h>
// #include <pcl/octree/octree_search.h>

class NegObstc
{
public:
  NegObstc();
  void loop_function();

private:
  ros::NodeHandle nh_;

  /*pcl*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Reconst, Cropped_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_inliers, Transformed_cloud;
  pcl::PointCloud<pcl::PointXYZ> Cloud_check_size, Cloud_check_sqr, Cloud_negative, Cloud_inliers_to_save;
  pcl::PointXYZ minPt, maxPt, minR, maxR, randPt;
  pcl::SACSegmentation<pcl::PointXYZ> seg;  // Create the segmentation object
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_reconst;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_color, cloud_out_color;

  /*others*/
  float h_plane, pace, X_min, Y_min, Z_min, X_max, Y_max, Z_max;
  size_t count_points;
  int N, i, writeCount;
  // Eigen::Vector3f min_pt, max_pt;

  /*publishers and subscribers*/
  ros::Publisher pub_plane;
  ros::Publisher pub_negative;
  ros::Subscriber sub;
  ros::Publisher marker_pub;
  ros::Publisher marker_pub_cubelist;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  // ros::Publisher octomap_publisher;

  /*messages*/
  visualization_msgs::Marker plane_marker;
  visualization_msgs::Marker cubelist_marker;
  sensor_msgs::PointCloud2 CloudMsg_plane;
  sensor_msgs::PointCloud2 CloudMsg_negative;
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose pose;
  // octomap_msgs::Octomap octree_msg;

  void find_plane();
  void spatial_segmentation();

  void GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::fromROSMsg(*cloud_msg.get(), *Cloud_Reconst);
  }

  ros::Timer timer = nh_.createTimer(ros::Duration(50), &NegObstc::accum_pcl, this, false, true);

  void accum_pcl(const ros::TimerEvent &event)
  {
    if ((Cloud_inliers_to_save.points.size() != 0))
    {
      writeCount++;
      char filename[100];
      sprintf(filename, "/media/daniela/Dados/pcd_files/planefittingcloud_%d.pcd", writeCount);
      // pcl::io::savePCDFileASCII(filename, Cloud_inliers_to_save);
      // pcl::io::savePCDFile("thisisatest.pcd", acum_cloud, true);
      ROS_INFO("Saved %lu points in point cloud", Cloud_inliers_to_save.points.size());
    }
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
  pub_negative = nh_.advertise<sensor_msgs::PointCloud2>("cloud_negative", 100);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("plane_marker", 1, true);
  marker_pub_cubelist = nh_.advertise<visualization_msgs::Marker>("cubelist_marker", 1, true);
  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);
  // listener.lookupTransform("/ground", "/world", ros::Time(0), transform);
  // octomap_publisher = nh_.advertise<octomap_msgs::Octomap>("octree_reconst", 1);

  /*initialize pointers*/
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Cloud_inliers.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Cropped_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  coefficients.reset(new pcl::ModelCoefficients);
  inliers.reset(new pcl::PointIndices);
  // cloud_in_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  // cloud_out_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

/**
 * @brief Loop function to calculate negative obstacles
 *
 */
void NegObstc::loop_function()
{
  Cloud_check_size = (*Cloud_Reconst);
  if (Cloud_check_size.points.size() != 0)
  {
    find_plane();
    pcl::toROSMsg(*Cloud_inliers, CloudMsg_plane);
    pub_plane.publish(CloudMsg_plane);
    marker_pub.publish(plane_marker);
    spatial_segmentation();
    marker_pub_cubelist.publish(cubelist_marker);
    // pcl::toROSMsg(Cloud_negative, CloudMsg_negative);
    // pub_negative.publish(CloudMsg_negative);
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
  seg.setDistanceThreshold(0.3);
  seg.setMaxIterations(1000);

  seg.setInputCloud(Cloud_Reconst);
  seg.segment(*inliers, *coefficients);

  // pcl::computeMeanAndCovarianceMatrix(),

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
  }

  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
  //           << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*Cloud_Reconst, inliers->indices, *Cloud_inliers);
  pcl::getMinMax3D(*Cloud_inliers, minPt, maxPt);
  h_plane = minPt.z + (maxPt.z - minPt.z) / 2;
  // h_plane = std::abs(coefficients->values[3] / sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2)
  // + pow(coefficients->values[2], 2)));
  // ROS_WARN("Finding ransac planes");
  // pcl::copyPointCloud(*Cloud_inliers, *cloud_in_color);
  // simple_grasping::extractUnorientedBoundingBox(*cloud_in_color,shape,pose);
  // simple_grasping::extractShape(*cloud_in_color, *cloud_out_color, shape, plane_marker.pose);
  // h_plane= simple_grasping::distancePointToPlane(origin_pt, coefficients);

  plane_marker.ns = "plane";
  plane_marker.header.frame_id = "ground";
  plane_marker.type = visualization_msgs::Marker::CUBE;
  plane_marker.pose.position.x = 10;
  // plane_marker.pose.position.y = (maxPt.y - minPt.y) / 2;
  plane_marker.pose.position.z = h_plane;
  plane_marker.pose.orientation.x = M_PI / 2 - atan2(coefficients->values[2], coefficients->values[1]);
  plane_marker.pose.orientation.y = M_PI / 2 - atan2(coefficients->values[2], coefficients->values[0]);
  plane_marker.pose.orientation.w = 1;
  plane_marker.scale.x = 20;    // shape.dimensions[0];
  plane_marker.scale.y = 20;    // shape.dimensions[1];
  plane_marker.scale.z = 0.01;  // shape.dimensions[2];
  plane_marker.color.r = 1.0;
  plane_marker.color.a = 0.8;

  /*trying to set total cloud*/
  if (Cloud_inliers_to_save.points.size() == 0)
  {
    Cloud_inliers_to_save = *Cloud_inliers;
  }
  else
  {
    Cloud_inliers_to_save = Cloud_inliers_to_save + *Cloud_inliers;
    // ROS_INFO("Escrevi na nova cloud e o seu tamanho é %lu",acum_cloud.points.size());
  }
}

void NegObstc::spatial_segmentation()
{
  N = 40 * 10;
  i = 0;
  cubelist_marker.ns = "cubelist";
  cubelist_marker.header.frame_id = "ground";
  cubelist_marker.type = visualization_msgs::Marker::CUBE_LIST;
  cubelist_marker.points.resize(N);
  cubelist_marker.colors.resize(N);
  cubelist_marker.scale.x = 1;  // shape.dimensions[0];
  cubelist_marker.scale.y = 1;  // shape.dimensions[1];
  cubelist_marker.scale.z = 1;

  /* TENTATIVA DE DIVISÃO ESPACIAL*/
  pcl::getMinMax3D(*Cloud_Reconst, minR, maxR);
  // pace = (maxR.y - minR.y) / 20;  // adjust pace
  pace = 1;
  for (int n_linhas = 0; n_linhas < 10; n_linhas++)
  {
    for (int Y = -20; Y <= 20 - pace; Y = Y + pace)
    {
      Cropped_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      X_min = n_linhas;  // X_min = transform.getOrigin().x() + n_linhas;
      X_max = X_min + 1;
      Y_min = Y;
      Y_max = Y + pace;
      Z_min = h_plane - 50;
      Z_max = h_plane + 50;

      pcl_ros::transformPointCloud("ground", ros::Time(0), *Cloud_Reconst, "map", *Transformed_cloud, NegObstc::listener);
      boxFilter.setMin(Eigen::Vector4f(X_min, Y_min, Z_min, 1.0));
      boxFilter.setMax(Eigen::Vector4f(X_max, Y_max, Z_max, 1.0));
      // boxFilter.setTranslation(
      //     Eigen::Vector3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()));
      // boxFilter.setRotation(
      //     Eigen::Vector3f(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z()));
      boxFilter.setInputCloud(Transformed_cloud);
      boxFilter.filter(*Cropped_cloud);
      Cloud_check_sqr = (*Cropped_cloud);
      count_points = Cloud_check_sqr.points.size();
      ROS_WARN("Number of points in square: %lu", count_points);

      // if (count_points < 3)  // adjust number
      // {
      //   for (int i = 0; i < 20; i++) /*put 20 random points within empty cuboids*/
      //   {
      //     srand(time(NULL));
      //     randPt.x = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (X_max - X_min))) + X_min;
      //     randPt.y = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (Y_max - Y_min))) + Y_min;
      //     randPt.z = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (Z_max - Z_min))) + Z_min;
      //     Cloud_negative.push_back(randPt);
      //   }
      // }

      cubelist_marker.points[i].x = n_linhas + 0.5;
      cubelist_marker.points[i].y = Y + pace / 2;
      cubelist_marker.points[i].z = h_plane + 0.5;
      cubelist_marker.colors[i].b = 0;
      cubelist_marker.colors[i].a = 0.5;

      if (count_points > 100)
      {
        cubelist_marker.colors[i].r = 0.0;
        cubelist_marker.colors[i].g = 1.0;
      }
      else if (count_points >= 75 && count_points < 100)
      {
        cubelist_marker.colors[i].r = 0.29;
        cubelist_marker.colors[i].g = 1.0;
      }
      else if (count_points >= 50 && count_points < 75)
      {
        cubelist_marker.colors[i].r = 0.58;
        cubelist_marker.colors[i].g = 1.0;
      }
      else if (count_points >= 25 && count_points < 50)
      {
        cubelist_marker.colors[i].r = 0.90;
        cubelist_marker.colors[i].g = 1.0;
      }
      else if (count_points >= 10 && count_points < 25)
      {
        cubelist_marker.colors[i].r = 1.0;
        cubelist_marker.colors[i].g = 0.80;
      }
      else if (count_points >= 5 && count_points < 10)
      {
        cubelist_marker.colors[i].r = 1.0;
        cubelist_marker.colors[i].g = 0.5;
      }
      else
      {
        cubelist_marker.colors[i].r = 1.0;
        cubelist_marker.colors[i].g = 0.0;
      }
      i++;
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
