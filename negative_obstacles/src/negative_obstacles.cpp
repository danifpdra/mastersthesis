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
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <visualization_msgs/Marker.h>

struct gradient_2d
{
  int vertical;
  int horizontal;
  double grad_tot;
  double direction;
};

struct color
{
  double r;
  double g;
  double b;
};

color colorbar(int level)
{
  color color_rgb;

  switch (level)
  {
    case -5:
      color_rgb.r = 0;
      color_rgb.g = 0;
      color_rgb.b = 0.5;
      break;
    case -4:
      color_rgb.r = 0;
      color_rgb.g = 0;
      color_rgb.b = 0.84;
      break;
    case -3:
      color_rgb.r = 0;
      color_rgb.g = 0.27;
      color_rgb.b = 1;
      break;
    case -2:
      color_rgb.r = 0;
      color_rgb.g = 0.64;
      color_rgb.b = 1;
      break;
    case -1:
      color_rgb.r = 0.137;
      color_rgb.g = 1;
      color_rgb.b = 0.82;
      break;
    case 0:
      color_rgb.r = 0.5;
      color_rgb.g = 1;
      color_rgb.b = 0.5;
      break;
    case 1:
      color_rgb.r = 0.82;
      color_rgb.g = 1;
      color_rgb.b = 0.137;
      break;
    case 2:
      color_rgb.r = 1;
      color_rgb.g = 0.64;
      color_rgb.b = 0;
      break;
    case 3:
      color_rgb.r = 1;
      color_rgb.g = 0.27;
      color_rgb.b = 0;
      break;
    case 4:
      color_rgb.r = 0.84;
      color_rgb.g = 0;
      color_rgb.b = 0;
      break;
    case 5:
      color_rgb.r = 0.5;
      color_rgb.g = 0;
      color_rgb.b = 0;
      break;
    default:
      color_rgb.r = 0.53;
      color_rgb.g = 0.53;
      color_rgb.b = 0.53;
  }

  return color_rgb;
}

class NegObstc
{
public:
  NegObstc();
  void loop_function();
  // color colobar(int level);

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
  /*others*/
  float h_plane, pace, X_min, Y_min, Z_min, X_max, Y_max, Z_max;
  size_t count_points;
  int N, i, j, writeCount, lin, col, nc, nl;
  Eigen::MatrixXd matriz;
  color color_grad, color_grad_x, color_grad_y, color_grad_d;
  int level_g, level_gx, level_gy, level_gd;

  // Eigen::Vector3f min_pt, max_pt;

  /*publishers and subscribers*/
  ros::Publisher pub_plane;
  ros::Publisher pub_negative;
  ros::Subscriber sub;
  ros::Publisher marker_pub;
  ros::Publisher marker_pub_cubelist;
  ros::Publisher marker_pub_gradient, marker_pub_grad_x, marker_pub_grad_y, marker_pub_grad_direction;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  // ros::Publisher octomap_publisher;

  /*messages*/
  visualization_msgs::Marker plane_marker;
  visualization_msgs::Marker cubelist_marker;
  visualization_msgs::Marker gradient_marker, grad_x_marker, grad_y_marker, grad_direction_marker;
  sensor_msgs::PointCloud2 CloudMsg_plane;
  sensor_msgs::PointCloud2 CloudMsg_negative;
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
  marker_pub_gradient = nh_.advertise<visualization_msgs::Marker>("gradient_marker", 1, true);
  marker_pub_grad_x = nh_.advertise<visualization_msgs::Marker>("grad_x_marker", 1, true);
  marker_pub_grad_y = nh_.advertise<visualization_msgs::Marker>("grad_y_marker", 1, true);
  marker_pub_grad_direction = nh_.advertise<visualization_msgs::Marker>("grad_direction_marker", 1, true);
  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);

  /*initialize pointers*/
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Cloud_inliers.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Cropped_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  coefficients.reset(new pcl::ModelCoefficients);
  inliers.reset(new pcl::PointIndices);
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
    marker_pub_gradient.publish(gradient_marker);
    marker_pub_grad_x.publish(grad_x_marker);
    marker_pub_grad_y.publish(grad_y_marker);
    marker_pub_grad_direction.publish(grad_direction_marker);
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

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
  }

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*Cloud_Reconst, inliers->indices, *Cloud_inliers);
  pcl::getMinMax3D(*Cloud_inliers, minPt, maxPt);
  h_plane = minPt.z + (maxPt.z - minPt.z) / 2;

  plane_marker.ns = "plane";
  plane_marker.header.frame_id = "ground";
  plane_marker.type = visualization_msgs::Marker::CUBE;
  plane_marker.pose.position.x = 10;
  // plane_marker.pose.position.y = (maxPt.y - minPt.y) / 2;
  plane_marker.pose.position.z = h_plane;
  plane_marker.pose.orientation.x = M_PI / 2 - atan2(coefficients->values[2], coefficients->values[1]);
  plane_marker.pose.orientation.y = M_PI / 2 - atan2(coefficients->values[2], coefficients->values[0]);
  plane_marker.pose.orientation.w = 1;
  plane_marker.scale.x = 50;    // shape.dimensions[0];
  plane_marker.scale.y = 50;    // shape.dimensions[1];
  plane_marker.scale.z = 0.01;  // shape.dimensions[2];
  plane_marker.color.b = 1.0;
  plane_marker.color.g = 0.5;
  plane_marker.color.a = 0.3;

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
  pace = 0.5;
  nl = 40 / pace;
  nc = 40 / pace;
  N = nc * nl;
  i = j = 0;
  lin = col = 0;

  matriz.resize(nl, nc);
  // int matriz[nc][nl];

  cubelist_marker.ns = "cubelist";
  cubelist_marker.header.frame_id = "moving_axis";
  cubelist_marker.type = visualization_msgs::Marker::CUBE_LIST;
  cubelist_marker.points.resize(N);
  cubelist_marker.colors.resize(N);
  cubelist_marker.scale.x = pace;  // shape.dimensions[0];
  cubelist_marker.scale.y = pace;  // shape.dimensions[1];
  cubelist_marker.scale.z = pace;

  /*cubelist marker with gradient colorbar*/
  gradient_2d grad[nc - 1][nl - 1];

  gradient_marker.ns = "cubelist_gradient";
  grad_x_marker.ns = "cubelist_grad_dx";
  grad_y_marker.ns = "cubelist_grad_dy";
  grad_direction_marker.ns = "cubelist_grad_direction";
  gradient_marker.header.frame_id = grad_x_marker.header.frame_id = grad_y_marker.header.frame_id =
      grad_direction_marker.header.frame_id = "moving_axis";
  gradient_marker.type = grad_x_marker.type = grad_y_marker.type = grad_direction_marker.type =
      visualization_msgs::Marker::CUBE_LIST;
  gradient_marker.points.resize((nc - 1) * (nl - 1));
  gradient_marker.colors.resize((nc - 1) * (nl - 1));
  grad_x_marker.points.resize((nc - 1) * (nl - 1));
  grad_x_marker.colors.resize((nc - 1) * (nl - 1));
  grad_y_marker.points.resize((nc - 1) * (nl - 1));
  grad_y_marker.colors.resize((nc - 1) * (nl - 1));
  grad_direction_marker.points.resize((nc - 1) * (nl - 1));
  grad_direction_marker.colors.resize((nc - 1) * (nl - 1));
  gradient_marker.scale.x = grad_x_marker.scale.x = grad_y_marker.scale.x = grad_direction_marker.scale.x =
      pace;  // shape.dimensions[0];
  gradient_marker.scale.y = grad_x_marker.scale.y = grad_y_marker.scale.y = grad_direction_marker.scale.y =
      pace;  // shape.dimensions[1];
  gradient_marker.scale.z = grad_x_marker.scale.z = grad_y_marker.scale.z = grad_direction_marker.scale.z = 0.01;

  for (int n_linhas = 0; n_linhas <= 40 - pace; n_linhas = n_linhas + pace)
  {
    col = 0;
    for (int Y = -20; Y <= 20 - pace; Y = Y + pace)
    {
      Cropped_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      X_min = n_linhas;  // X_min = transform.getOrigin().x() + n_linhas;
      X_max = X_min + pace;
      Y_min = Y;
      Y_max = Y + pace;
      Z_min = h_plane - 50;
      Z_max = h_plane + 50;

      pcl_ros::transformPointCloud("moving_axis", ros::Time(0), *Cloud_Reconst, "map", *Transformed_cloud,
                                   NegObstc::listener);
      boxFilter.setMin(Eigen::Vector4f(X_min, Y_min, Z_min, 1.0));
      boxFilter.setMax(Eigen::Vector4f(X_max, Y_max, Z_max, 1.0));
      boxFilter.setInputCloud(Transformed_cloud);
      boxFilter.filter(*Cropped_cloud);
      Cloud_check_sqr = (*Cropped_cloud);
      count_points = Cloud_check_sqr.points.size();

      // ROS_WARN("Number of points in square %d (pos: %d, %d): %lu", i, n_linhas, Y, count_points);

      cubelist_marker.points[i].x = n_linhas + pace / 2;
      cubelist_marker.points[i].y = Y + pace / 2;
      cubelist_marker.points[i].z = 0.01;
      cubelist_marker.colors[i].b = 0;
      cubelist_marker.colors[i].a = 0.5;

      matriz(lin, col) = count_points;

      if (col <= nc - 2 && lin <= nl - 2)
      {
        gradient_marker.points[j].x = grad_x_marker.points[j].x = grad_y_marker.points[j].x =
            grad_direction_marker.points[j].x = n_linhas + pace / 2;
        gradient_marker.points[j].y = grad_x_marker.points[j].y = grad_y_marker.points[j].y =
            grad_direction_marker.points[j].y = Y + pace / 2;
        gradient_marker.points[j].z = grad_x_marker.points[j].z = grad_y_marker.points[j].z =
            grad_direction_marker.points[j].z = 0.1;
        gradient_marker.colors[j].a = grad_x_marker.colors[j].a = grad_y_marker.colors[j].a =
            grad_direction_marker.colors[j].a = 1;
        j++;
      }

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
      col++;
    }
    lin++;
  }

  // calculate gradient matrix
  j = 0;
  for (int l = 0; l < nl - 1; l++)
  {
    for (int c = 0; c < nc - 1; c++)
    {
      grad[l][c].vertical = matriz(l + 1, c) - matriz(l, c);
      grad[l][c].horizontal = matriz(l, c + 1) - matriz(l, c);
      grad[l][c].grad_tot =
          sqrt(pow(static_cast<double>(grad[l][c].vertical), 2) + pow(static_cast<double>(grad[l][c].horizontal), 2));
      grad[l][c].direction =
          atan2(static_cast<double>(grad[l][c].horizontal), static_cast<double>(grad[l][c].vertical));
      // ROS_WARN("Gx=%d, Gy=%d, G=%f, theta=%f", grad[l][c].vertical, grad[l][c].horizontal, grad[l][c].grad_tot,
      //          grad[l][c].direction);

      /*Gx*/
      if (grad[l][c].vertical > 100)
      {
        level_gx = 5;
      }
      else if (grad[l][c].vertical >= 80 && grad[l][c].vertical < 100)
      {
        level_gx = 4;
      }
      else if (grad[l][c].vertical >= 60 && grad[l][c].vertical < 80)
      {
        level_gx = 3;
      }
      else if (grad[l][c].vertical >= 40 && grad[l][c].vertical < 60)
      {
        level_gx = 2;
      }
      else if (grad[l][c].vertical >= 20 && grad[l][c].vertical < 40)
      {
        level_gx = 1;
      }
      else if (grad[l][c].vertical >= 0 && grad[l][c].vertical < 20)
      {
        level_gx = 0;
      }
      else if (grad[l][c].vertical >= -20 && grad[l][c].vertical < 0)
      {
        level_gx = -1;
      }
      else if (grad[l][c].vertical >= -40 && grad[l][c].vertical < -20)
      {
        level_gx = -2;
      }
      else if (grad[l][c].vertical >= -60 && grad[l][c].vertical < -40)
      {
        level_gx = -3;
      }
      else if (grad[l][c].vertical >= -80 && grad[l][c].vertical < -60)
      {
        level_gx = -4;
      }
      else if (grad[l][c].vertical < -80)
      {
        level_gx = -5;
      }
      else
      {
        level_gx = 100;
      }

      /*Gy*/
      if (grad[l][c].horizontal > 100)
      {
        level_gy = 5;
      }
      else if (grad[l][c].horizontal >= 80 && grad[l][c].horizontal < 100)
      {
        level_gy = 4;
      }
      else if (grad[l][c].horizontal >= 60 && grad[l][c].horizontal < 80)
      {
        level_gy = 3;
      }
      else if (grad[l][c].horizontal >= 40 && grad[l][c].horizontal < 60)
      {
        level_gy = 2;
      }
      else if (grad[l][c].horizontal >= 20 && grad[l][c].horizontal < 40)
      {
        level_gy = 1;
      }
      else if (grad[l][c].horizontal >= 0 && grad[l][c].horizontal < 20)
      {
        level_gy = 0;
      }
      else if (grad[l][c].horizontal >= -20 && grad[l][c].horizontal < 0)
      {
        level_gy = -1;
      }
      else if (grad[l][c].horizontal >= -40 && grad[l][c].horizontal < -20)
      {
        level_gy = -2;
      }
      else if (grad[l][c].horizontal >= -60 && grad[l][c].horizontal < -40)
      {
        level_gy = -3;
      }
      else if (grad[l][c].horizontal >= -80 && grad[l][c].horizontal < -60)
      {
        level_gy = -4;
      }
      else if (grad[l][c].horizontal < -80)
      {
        level_gy = -5;
      }
      else
      {
        level_gy = 100;
      }

      /*G*/

      if (grad[l][c].grad_tot > 200)
      {
        level_g = 5;
      }
      else if (grad[l][c].grad_tot >= 180 && grad[l][c].grad_tot < 200)
      {
        level_g = 4;
      }
      else if (grad[l][c].grad_tot >= 160 && grad[l][c].grad_tot < 180)
      {
        level_g = 3;
      }
      else if (grad[l][c].grad_tot >= 140 && grad[l][c].grad_tot < 160)
      {
        level_g = 2;
      }
      else if (grad[l][c].grad_tot >= 120 && grad[l][c].grad_tot < 140)
      {
        level_g = 1;
      }
      else if (grad[l][c].grad_tot >= 100 && grad[l][c].grad_tot < 120)
      {
        level_g = 0;
      }
      else if (grad[l][c].grad_tot >= 80 && grad[l][c].grad_tot < 100)
      {
        level_g = -1;
      }
      else if (grad[l][c].grad_tot >= 60 && grad[l][c].grad_tot < 80)
      {
        level_g = -2;
      }
      else if (grad[l][c].grad_tot >= 40 && grad[l][c].grad_tot < 60)
      {
        level_g = -3;
      }
      else if (grad[l][c].grad_tot >= 20 && grad[l][c].grad_tot < 40)
      {
        level_g = -4;
      }
      else if (grad[l][c].grad_tot < 20)
      {
        level_g = -5;
      }
      else
      {
        level_g = 100;
      }

      /*Gradient direction*/
      if (grad[l][c].direction > M_PI)
      {
        level_gd = 5;
      }
      else if (grad[l][c].direction >= (4 / 5) * M_PI && grad[l][c].direction < M_PI)
      {
        level_gd = 4;
      }
      else if (grad[l][c].direction >= (3 / 5) * M_PI && grad[l][c].direction < (4 / 5) * M_PI)
      {
        level_gd = 3;
      }
      else if (grad[l][c].direction >= (2 / 5) * M_PI && grad[l][c].direction < (3 / 5) * M_PI)
      {
        level_gd = 2;
      }
      else if (grad[l][c].direction >= (1 / 5) * M_PI && grad[l][c].direction < (2 / 5) * M_PI)
      {
        level_gd = 1;
      }
      else if (grad[l][c].direction >= 0 && grad[l][c].direction < (1 / 5) * M_PI)
      {
        level_gd = 0;
      }
      else if (grad[l][c].direction >= -(2 / 5) * M_PI && grad[l][c].direction < -(1 / 5) * M_PI)
      {
        level_gd = -1;
      }
      else if (grad[l][c].direction >= -(3 / 5) * M_PI && grad[l][c].direction < -(2 / 5) * M_PI)
      {
        level_gd = -2;
      }
      else if (grad[l][c].direction >= -(4 / 5) * M_PI && grad[l][c].direction < -(3 / 5) * M_PI)
      {
        level_gd = -3;
      }
      else if (grad[l][c].direction >= -M_PI && grad[l][c].direction < -(4 / 5) * M_PI)
      {
        level_gd = -4;
      }
      else if (grad[l][c].direction < -M_PI)
      {
        level_gd = -5;
      }
      else
      {
        level_gd = 100;
      }

      // ROS_WARN("Levels: Gx=%d, Gy=%d, G=%d, theta=%d", level_gx, level_gy, level_g, level_gd);

      color_grad = colorbar(level_g);
      color_grad_x = colorbar(level_gx);
      color_grad_y = colorbar(level_gy);
      color_grad_d = colorbar(level_gd);

      // ROS_WARN("Colors: R=%f, G=%f, B=%f", color_grad.r, color_grad.g, color_grad_x.b);

      gradient_marker.colors[j].r = color_grad.r;
      gradient_marker.colors[j].g = color_grad.g;
      gradient_marker.colors[j].b = color_grad.b;

      grad_x_marker.colors[j].r = color_grad_x.r;
      grad_x_marker.colors[j].g = color_grad_x.g;
      grad_x_marker.colors[j].b = color_grad_x.b;

      grad_y_marker.colors[j].r = color_grad_y.r;
      grad_y_marker.colors[j].g = color_grad_y.g;
      grad_y_marker.colors[j].b = color_grad_y.b;

      grad_direction_marker.colors[j].r = color_grad_d.r;
      grad_direction_marker.colors[j].g = color_grad_d.g;
      grad_direction_marker.colors[j].b = color_grad_d.b;

      j++;
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
