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
#include <visualization_msgs/Marker.h>

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"

#include "negative_obstacles/negative_obstacles.h"

class NegObstc
{
public:
  NegObstc();
  void loop_function();

private:
  ros::NodeHandle nh_;

  /*pcl*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Reconst;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Transformed_cloud;
  pcl::PointCloud<pcl::PointXYZ> Cloud_check_size;
  /*others*/
  double pace;
  int N, i, j, writeCount, lin, col, nc, nl;
  Eigen::MatrixXd densityMatrix;
  color color_grad, color_grad_x, color_grad_y, color_grad_d;
  std::vector<signed char> density_points, grad_points, grad_x_points, grad_y_points, grad_dir_points;

  /*publishers and subscribers*/
  ros::Subscriber sub;
  ros::Publisher marker_pub;
  ros::Publisher map_pub, grad_pub, grad_x_pub, grad_y_pub, grad_dir_pub;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  /*messages*/
  std_msgs::Header header;
  nav_msgs::MapMetaData info, info_grad;
  nav_msgs::OccupancyGrid densityGrid, gradGrid, gradXGrid, gradYGrid, gradDirGrid;

  void spatial_segmentation();

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
  map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("map_pub", 1, true);
  grad_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_pub", 1, true);
  grad_x_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_x_pub", 1, true);
  grad_y_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_y_pub", 1, true);
  grad_dir_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_dir_pub", 1, true);
  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);

  /*initialize pointers*/
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
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
    spatial_segmentation();

    map_pub.publish(densityGrid);
    grad_pub.publish(gradGrid);
    grad_x_pub.publish(gradXGrid);
    grad_y_pub.publish(gradYGrid);
    grad_dir_pub.publish(gradDirGrid);
  }
}

void NegObstc::spatial_segmentation()
{
  pace = 0.2;
  nl = 40 / pace;
  nc = 40 / pace;
  N = nc * nl;
  i = j = 0;
  lin = col = 0;

  /*clearing grid data*/
  density_points.clear();
  density_points.resize(N);

  grad_points.clear();
  grad_points.resize((nc - 1) * (nl - 1));

  grad_x_points.clear();
  grad_x_points.resize((nc - 1) * (nl - 1));

  grad_y_points.clear();
  grad_y_points.resize((nc - 1) * (nl - 1));

  grad_dir_points.clear();
  grad_dir_points.resize((nc - 1) * (nl - 1));

  /*initalizing messages for OccupancyGrid construction*/
  info.height = nc;
  info.width = nl;
  info_grad.height = nc - 1;
  info_grad.width = nl - 1;
  info.resolution = info_grad.resolution = pace;
  header.frame_id = "moving_axis";
  info.map_load_time = header.stamp = ros::Time(0);
  info.origin.position.x = info_grad.origin.position.x = 0;
  info.origin.position.y = info_grad.origin.position.y = -20;
  info.origin.position.z = info_grad.origin.position.z = 0;
  info.origin.orientation.x = info_grad.origin.orientation.x = 0;
  info.origin.orientation.y = info_grad.origin.orientation.y = 0;
  info.origin.orientation.z = info_grad.origin.orientation.z = 0;

  /*initializing grids with constructed messages*/
  densityGrid.header = gradGrid.header = gradXGrid.header = gradYGrid.header = gradDirGrid.header = header;
  densityGrid.info = info;
  gradGrid.info = gradXGrid.info = gradYGrid.info = gradDirGrid.info = info_grad;

  /*initialize matrix to save density*/
  densityMatrix.resize(nl, nc);
  densityMatrix.setZero(nl, nc);

  /*cubelist marker with gradient colorbar*/
  gradient_2d grad[nc - 1][nl - 1];

  pcl_ros::transformPointCloud("moving_axis", ros::Time(0), *Cloud_Reconst, "map", *Transformed_cloud,
                               NegObstc::listener);

  // for (pcl::PointCloud<pcl::PointXYZ>::iterator cloud_it = Transformed_cloud->begin();
  //  cloud_it != Transformed_cloud->end(); ++cloud_it)
  for (const pcl::PointXYZ &point : *Transformed_cloud)
  {
    if (point.x <= 40 && point.x >= 0 && point.y >= -20 && point.y <= 20)
    {
      lin = (int)floor(point.x / pace);
      col = (int)floor(point.y / pace) + 20 / pace;

      if (lin < nl && col < nc)
        densityMatrix(lin, col) += 1;
      if (lin + col * nl < N)
        density_points[lin + col * nl] += 2;
    }
  }

  densityGrid.data = density_points;

  // calculate gradient matrix
  j = 0;
  for (int c = 0; c < nc - 1; c++)
  {
    for (int l = 0; l < nl - 1; l++)
    {
      grad[l][c].vertical = densityMatrix(l + 1, c) - densityMatrix(l, c);
      grad[l][c].horizontal = densityMatrix(l, c + 1) - densityMatrix(l, c);
      // grad[l][c].grad_tot =
      //     sqrt(pow(static_cast<double>(grad[l][c].vertical), 2) + pow(static_cast<double>(grad[l][c].horizontal),
      //     2));
      grad[l][c].grad_tot =
          abs(static_cast<double>(grad[l][c].vertical)) + abs(static_cast<double>(grad[l][c].horizontal));
      grad[l][c].direction =
          atan2(static_cast<double>(grad[l][c].horizontal), static_cast<double>(grad[l][c].vertical));

      /*Gx*/
      // grad_x_points[j] = grad[l][c].vertical;
      if (grad[l][c].vertical > 100 * pace)
      {
        grad_x_points[j] = 100;
      }
      else if (grad[l][c].vertical >= 80 * pace && grad[l][c].vertical < 100 * pace)
      {
        grad_x_points[j] = 80;
      }
      else if (grad[l][c].vertical >= 60 * pace && grad[l][c].vertical < 80 * pace)
      {
        grad_x_points[j] = 60;
      }
      else if (grad[l][c].vertical >= 40 * pace && grad[l][c].vertical < 60 * pace)
      {
        grad_x_points[j] = 40;
      }
      else if (grad[l][c].vertical >= 20 * pace && grad[l][c].vertical < 40 * pace)
      {
        grad_x_points[j] = 20;
      }
      else if (grad[l][c].vertical >= 0 * pace && grad[l][c].vertical < 20 * pace)
      {
        grad_x_points[j] = 0;
      }
      else if (grad[l][c].vertical >= -20 * pace && grad[l][c].vertical < 0)
      {
        grad_x_points[j] = 20;
      }
      else if (grad[l][c].vertical >= -40 * pace && grad[l][c].vertical < -20 * pace)
      {
        grad_x_points[j] = 40;
      }
      else if (grad[l][c].vertical >= -60 * pace && grad[l][c].vertical < -40 * pace)
      {
        grad_x_points[j] = 60;
      }
      else if (grad[l][c].vertical >= -80 * pace && grad[l][c].vertical < -60 * pace)
      {
        grad_x_points[j] = 80;
      }
      else if (grad[l][c].vertical < -80 * pace)
      {
        grad_x_points[j] = 100;
      }
      else
      {
        grad_x_points[j] = 0;
      }

      /*Gy*/
      if (grad[l][c].horizontal > 100 * pace)
      {
        grad_y_points[j] = 100;
      }
      else if (grad[l][c].horizontal >= 80 * pace && grad[l][c].horizontal < 100 * pace)
      {
        grad_y_points[j] = 80;
      }
      else if (grad[l][c].horizontal >= 60 * pace && grad[l][c].horizontal < 80 * pace)
      {
        grad_y_points[j] = 60;
      }
      else if (grad[l][c].horizontal >= 40 * pace && grad[l][c].horizontal < 60 * pace)
      {
        grad_y_points[j] = 40;
      }
      else if (grad[l][c].horizontal >= 20 * pace && grad[l][c].horizontal < 40 * pace)
      {
        grad_y_points[j] = 20;
      }
      else if (grad[l][c].horizontal >= 0 && grad[l][c].horizontal < 20 * pace)
      {
        grad_y_points[j] = 0;
      }
      else if (grad[l][c].horizontal >= -20 * pace && grad[l][c].horizontal < 0)
      {
        grad_y_points[j] = 20;
      }
      else if (grad[l][c].horizontal >= -40 * pace && grad[l][c].horizontal < -20 * pace)
      {
        grad_y_points[j] = 40;
      }
      else if (grad[l][c].horizontal >= -60 * pace && grad[l][c].horizontal < -40 * pace)
      {
        grad_y_points[j] = 60;
      }
      else if (grad[l][c].horizontal >= -80 * pace && grad[l][c].horizontal < -60 * pace)
      {
        grad_y_points[j] = 80;
      }
      else if (grad[l][c].horizontal < -80 * pace)
      {
        grad_y_points[j] = 100;
      }
      else
      {
        grad_y_points[j] = 0;
      }

      /*G*/

      if (grad[l][c].grad_tot > 200 * pace)
      {
        grad_points[j] = 100;
      }
      else if (grad[l][c].grad_tot >= 180 * pace && grad[l][c].grad_tot < 200 * pace)
      {
        grad_points[j] = 80;
      }
      else if (grad[l][c].grad_tot >= 160 * pace && grad[l][c].grad_tot < 180 * pace)
      {
        grad_points[j] = 60;
      }
      else if (grad[l][c].grad_tot >= 140 * pace && grad[l][c].grad_tot < 160 * pace)
      {
        grad_points[j] = 40;
      }
      else if (grad[l][c].grad_tot >= 120 * pace && grad[l][c].grad_tot < 140 * pace)
      {
        grad_points[j] = 20;
      }
      else if (grad[l][c].grad_tot >= 100 * pace && grad[l][c].grad_tot < 120 * pace)
      {
        grad_points[j] = 0;
      }
      else if (grad[l][c].grad_tot >= 80 * pace && grad[l][c].grad_tot < 100 * pace)
      {
        grad_points[j] = 20;
      }
      else if (grad[l][c].grad_tot >= 60 * pace && grad[l][c].grad_tot < 80 * pace)
      {
        grad_points[j] = 40;
      }
      else if (grad[l][c].grad_tot >= 40 * pace && grad[l][c].grad_tot < 60 * pace)
      {
        grad_points[j] = 60;
      }
      else if (grad[l][c].grad_tot >= 20 * pace && grad[l][c].grad_tot < 40 * pace)
      {
        grad_points[j] = 80;
      }
      else if (grad[l][c].grad_tot < 20 * pace)
      {
        grad_points[j] = 100;
      }
      else
      {
        grad_points[j] = 0;
      }

      /*Gradient direction*/
      if (grad[l][c].direction > M_PI)
      {
        grad_dir_points[j] = 100;
      }
      else if (grad[l][c].direction >= (4 / 5) * M_PI && grad[l][c].direction < M_PI)
      {
        grad_dir_points[j] = 80;
      }
      else if (grad[l][c].direction >= (3 / 5) * M_PI && grad[l][c].direction < (4 / 5) * M_PI)
      {
        grad_dir_points[j] = 60;
      }
      else if (grad[l][c].direction >= (2 / 5) * M_PI && grad[l][c].direction < (3 / 5) * M_PI)
      {
        grad_dir_points[j] = 40;
      }
      else if (grad[l][c].direction >= (1 / 5) * M_PI && grad[l][c].direction < (2 / 5) * M_PI)
      {
        grad_dir_points[j] = 20;
      }
      else if (grad[l][c].direction >= 0 && grad[l][c].direction < (1 / 5) * M_PI)
      {
        grad_dir_points[j] = 0;
      }
      else if (grad[l][c].direction >= -(2 / 5) * M_PI && grad[l][c].direction < -(1 / 5) * M_PI)
      {
        grad_dir_points[j] = 20;
      }
      else if (grad[l][c].direction >= -(3 / 5) * M_PI && grad[l][c].direction < -(2 / 5) * M_PI)
      {
        grad_dir_points[j] = 40;
      }
      else if (grad[l][c].direction >= -(4 / 5) * M_PI && grad[l][c].direction < -(3 / 5) * M_PI)
      {
        grad_dir_points[j] = 60;
      }
      else if (grad[l][c].direction >= -M_PI && grad[l][c].direction < -(4 / 5) * M_PI)
      {
        grad_dir_points[j] = 80;
      }
      else if (grad[l][c].direction < -M_PI)
      {
        grad_dir_points[j] = 100;
      }
      else
      {
        grad_dir_points[j] = 0;
      }

      // ROS_WARN("Levels: Gx=%d, Gy=%d, G=%d, theta=%d", grad_x_points[j], grad_y_points[j], grad_points[j],
      // grad_dir_points[j]);

      j++;
    }
  }

  gradGrid.data = grad_points;
  gradXGrid.data = grad_x_points;
  gradYGrid.data = grad_y_points;
  gradDirGrid.data = grad_dir_points;
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
