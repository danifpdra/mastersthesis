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