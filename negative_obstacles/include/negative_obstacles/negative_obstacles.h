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

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"

struct gradient_2d
{
  int vertical;
  int horizontal;
  double grad_tot;
};

struct sobel_grad
{
  double sobel_gx;
  double sobel_gy;
  double sobel;
};

struct prewitt_grad
{
  double gx;
  double gy;
  double prewitt;
};

struct kirsh_grad
{
  double gx;
  double gy;
  double kirsh;
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

/*Gx sobel*/
int Sobel1D(int G, double pace)
{
  int level;
  if (G > 1000 * pace)
    level = 100;
  else if (G >= 800 * pace && G < 1000 * pace)
    level = 80;
  else if (G >= 600 * pace && G < 800 * pace)
    level = 60;
  else if (G >= 500 * pace && G < 600 * pace)
    level = 50;
  else if (G >= 400 * pace && G < 500 * pace)
    level = 40;
  else if (G >= 200 * pace && G < 400 * pace)
    level = 30;
  else if (G >= 100 * pace && G < 200 * pace)
    level = 15;
  else if (G >= 0 && G < 100 * pace)
    level = 0;
  else if (G >= -100 * pace && G < 0)
    level = 0;
  else if (G >= -200 * pace && G < -100 * pace)
    level = 15;
  else if (G >= -400 * pace && G < -200 * pace)
    level = 30;
  else if (G >= -500 * pace && G < -400 * pace)
    level = 40;
  else if (G >= -600 * pace && G < -500 * pace)
    level = 50;
  else if (G >= -800 * pace && G < -600 * pace)
    level = 60;
  else if (G >= -800 * pace && G < -1000 * pace)
    level = 80;
  else if (G < -1000 * pace)
    level = 100;
  else
    level = 0;

  return level;
}

/*Sobel magnitude*/
int SobelMag(int G, double pace)
{
  int level;
  if (G > 4000 * pace)
    level = 100;
  else if (G >= 3000 * pace && G < 4000 * pace)
    level = 90;
  else if (G >= 2000 * pace && G < 3000 * pace)
    level = 80;
  else if (G >= 1000 * pace && G < 2000 * pace)
    level = 70;
  else if (G >= 750 * pace && G < 1000 * pace)
    level = 60;
  else if (G >= 500 * pace && G < 750 * pace)
    level = 50;
  else if (G >= 400 * pace && G < 500 * pace)
    level = 40;
  else if (G >= 200 * pace && G < 400 * pace)
    level = 30;
  else if (G >= 100 * pace && G < 200 * pace)
    level = 20;
  else if (G >= 20 * pace && G < 100 * pace)
    level = 10;
  else if (G < 20 * pace)
    level = 5;
  else
    level = 0;

  return level;
}

/*Gx*/
int Grad1D(int G, double pace)
{
  int level;
  if (G > 100 * pace)
    level = 100;
  else if (G >= 80 * pace && G < 100 * pace)
    level = 50;
  else if (G >= 60 * pace && G < 80 * pace)
    level = 40;
  else if (G >= 40 * pace && G < 60 * pace)
    level = 30;
  else if (G >= 20 * pace && G < 40 * pace)
    level = 15;
  else if (G >= 0 * pace && G < 20 * pace)
    level = 0;
  else if (G >= -20 * pace && G < 0)
    level = 15;
  else if (G >= -40 * pace && G < -20 * pace)
    level = 30;
  else if (G >= -60 * pace && G < -40 * pace)
    level = 40;
  else if (G >= -80 * pace && G < -60 * pace)
    level = 50;
  else if (G < -80 * pace)
    level = 100;
  else
    level = 0;

  return level;
}

/*G*/
int GradMag(int G, double pace)
{
  int level;
  if (G > 200 * pace)
    level = 100;
  else if (G >= 180 * pace && G < 200 * pace)
    level = 90;
  else if (G >= 160 * pace && G < 180 * pace)
    level = 80;
  else if (G >= 140 * pace && G < 160 * pace)
    level = 70;
  else if (G >= 120 * pace && G < 140 * pace)
    level = 60;
  else if (G >= 100 * pace && G < 120 * pace)
    level = 50;
  else if (G >= 80 * pace && G < 100 * pace)
    level = 40;
  else if (G >= 60 * pace && G < 80 * pace)
    level = 30;
  else if (G >= 40 * pace && G < 60 * pace)
    level = 20;
  else if (G >= 20 * pace && G < 40 * pace)
    level = 10;
  else if (G < 20 * pace)
    level = 5;
  else
    level = 0;

  return level;
}

/*Gradient direction*/
int GradDir(int G, double pace)
{
  int level;
  if (G > M_PI)
    level = 100;
  else if (G >= (4 / 5) * M_PI && G < M_PI)
    level = 50;
  else if (G >= (3 / 5) * M_PI && G < (4 / 5) * M_PI)
    level = 40;
  else if (G >= (2 / 5) * M_PI && G < (3 / 5) * M_PI)
    level = 30;
  else if (G >= (1 / 5) * M_PI && G < (2 / 5) * M_PI)
    level = 15;
  else if (G >= 0 && G < (1 / 5) * M_PI)
    level = 0;
  else if (G >= -(2 / 5) * M_PI && G < -(1 / 5) * M_PI)
    level = 15;
  else if (G >= -(3 / 5) * M_PI && G < -(2 / 5) * M_PI)
    level = 30;
  else if (G >= -(4 / 5) * M_PI && G < -(3 / 5) * M_PI)
    level = 40;
  else if (G >= -M_PI && G < -(4 / 5) * M_PI)
    level = 50;
  else if (G < -M_PI)
    level = 100;
  else
    level = 0;

  return level;
}

int Threshold(double G, double lim)
{
  int level;
  if (G > lim)
    level = 100;
  else
    level = 0;

  return level;
}