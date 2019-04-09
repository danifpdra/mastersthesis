#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/convolution.h>
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
#include <stdlib.h>
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

// openCv
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// grid maps
#include <grid_map_core/grid_map_core.hpp>
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

#include "negative_obstacles/negative_obstacles.h"

class NegObstc
{
public:
  NegObstc();
  void LoopFunction();

private:
  ros::NodeHandle nh_;

  /*pcl*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Reconst, Transformed_cloud;

  /*others*/
  double pace;
  float max;
  int N, j, s, writeCount, lin, col, nc, nl, ls, cs;
  Eigen::MatrixXd densityMatrix;
  std::vector<signed char> density_points, grad_points, grad_x_points, grad_y_points, sobel_gx_points, sobel_gy_points,
      sobel_points, prewitt_points, kirsh_points;
  cv::Mat canny_img, detected_edges, laplace_img, temporal_image, blur_img, sobel_img, sobel_grad_x, sobel_grad_y;

  /*publishers and subscribers*/
  ros::Subscriber sub;
  ros::Publisher density_pub, grad_pub, grad_x_pub, grad_y_pub, sobel_gx_pub, sobel_gy_pub, sobel_pub, prewitt_pub,
      kirsh_pub, canny_pub, laplacian_pub;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  ros::Publisher img_pub_canny, img_pub_laplace;
  image_transport::ImageTransport it;
  grid_map::GridMap temporalGridMap, cannyGridMap, laplaceGridMap, sobelGridMap, sobelGxGridMap, sobelGyGridMap;

  /*messages*/
  std_msgs::Header header;
  nav_msgs::MapMetaData info, info_grad, info_sobel;
  nav_msgs::OccupancyGrid densityGrid, gradGrid, gradXGrid, gradYGrid, sobelGxGrid, sobelGyGrid, sobelGrid, prewittGrid,
      kirshGrid, cannyGrid, laplaceGrid;
  sensor_msgs::ImagePtr msg_canny, msg_laplace, msg_sobel, msg_sobelX, msg_sobelY;

  void SpacialSegmentation();
  void EdgeDetection();

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
  : it(nh_)
  , temporalGridMap({ "elevation" })
  , cannyGridMap({ "canny" })
  , laplaceGridMap({ "laplacian" })
  , sobelGridMap({ "sobel" })
  , sobelGxGridMap({ "sobelX" })
  , sobelGyGridMap({ "sobelY" })
{
  /*publishers and subscribers*/
  density_pub = nh_.advertise<nav_msgs::OccupancyGrid>("density_pub", 1, true);

  grad_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_pub", 1, true);
  grad_x_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_x_pub", 1, true);
  grad_y_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grad_y_pub", 1, true);

  sobel_pub = nh_.advertise<nav_msgs::OccupancyGrid>("sobel_pub", 1, true);
  sobel_gx_pub = nh_.advertise<nav_msgs::OccupancyGrid>("sobel_gx_pub", 1, true);
  sobel_gy_pub = nh_.advertise<nav_msgs::OccupancyGrid>("sobel_gy_pub", 1, true);

  prewitt_pub = nh_.advertise<nav_msgs::OccupancyGrid>("prewitt_pub", 1, true);
  kirsh_pub = nh_.advertise<nav_msgs::OccupancyGrid>("kirsh_pub", 1, true);

  img_pub_canny = nh_.advertise<sensor_msgs::Image>("canny_img", 10);
  img_pub_laplace = nh_.advertise<sensor_msgs::Image>("laplace_img", 10);
  canny_pub = nh_.advertise<nav_msgs::OccupancyGrid>("canny_pub", 1, true);
  laplacian_pub = nh_.advertise<nav_msgs::OccupancyGrid>("laplacian_pub", 1, true);

  sub = nh_.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);

  /*initialize pointers*/
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

/**
 * @brief Loop function to calculate negative obstacles
 *
 */
void NegObstc::LoopFunction()
{
  // Cloud_check_size = (*Cloud_Reconst);
  if (Cloud_Reconst->points.size() != 0)
  {
    SpacialSegmentation();

    density_pub.publish(densityGrid);
    grad_pub.publish(gradGrid);
    grad_x_pub.publish(gradXGrid);
    grad_y_pub.publish(gradYGrid);

    prewitt_pub.publish(prewittGrid);
    kirsh_pub.publish(kirshGrid);

    EdgeDetection();
    msg_canny = cv_bridge::CvImage{ header, "mono8", canny_img }.toImageMsg();
    img_pub_canny.publish(msg_canny);

    msg_laplace = cv_bridge::CvImage{ header, "mono8", laplace_img }.toImageMsg();
    img_pub_laplace.publish(msg_laplace);

    grid_map::GridMapRosConverter::initializeFromImage(*msg_canny, pace, cannyGridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*msg_canny, "canny", cannyGridMap, 0, 255, 255);
    grid_map::GridMapRosConverter::toOccupancyGrid(cannyGridMap, "canny", 0, 255, cannyGrid);
    cannyGrid.info = info;

    grid_map::GridMapRosConverter::initializeFromImage(*msg_laplace, pace, laplaceGridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*msg_laplace, "laplacian", laplaceGridMap, 0, 255, 255);
    grid_map::GridMapRosConverter::toOccupancyGrid(laplaceGridMap, "laplacian", 0, 255, laplaceGrid);
    laplaceGrid.info = info;

    canny_pub.publish(cannyGrid);
    laplacian_pub.publish(laplaceGrid);

    msg_sobel = cv_bridge::CvImage{ header, "mono8", sobel_img }.toImageMsg();
    grid_map::GridMapRosConverter::initializeFromImage(*msg_sobel, pace, sobelGridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobel, "sobel", sobelGridMap, 0, 255, 125);
    grid_map::GridMapRosConverter::toOccupancyGrid(sobelGridMap, "sobel", 0, 255, sobelGrid);

    msg_sobelX = cv_bridge::CvImage{ header, "mono8", sobel_grad_x }.toImageMsg();
    grid_map::GridMapRosConverter::initializeFromImage(*msg_sobelX, pace, sobelGxGridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobelX, "sobelX", sobelGxGridMap, 0, 255, 125);
    grid_map::GridMapRosConverter::toOccupancyGrid(sobelGxGridMap, "sobelX", 0, 255, sobelGxGrid);

    msg_sobelY = cv_bridge::CvImage{ header, "mono8", sobel_grad_y }.toImageMsg();
    grid_map::GridMapRosConverter::initializeFromImage(*msg_sobelY, pace, sobelGyGridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobelY, "sobelY", sobelGyGridMap, 0, 255, 125);
    grid_map::GridMapRosConverter::toOccupancyGrid(sobelGyGridMap, "sobelY", 0, 255, sobelGyGrid);

    sobelGrid.info = sobelGxGrid.info = sobelGyGrid.info = info;
    sobel_pub.publish(sobelGrid);
    sobel_gx_pub.publish(sobelGxGrid);
    sobel_gy_pub.publish(sobelGyGrid);
  }
}

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void NegObstc::EdgeDetection()
{
  int kernel_size = 3;

  grid_map::GridMapRosConverter::fromOccupancyGrid(densityGrid, "elevation", temporalGridMap);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(temporalGridMap, "elevation", CV_8UC1, temporal_image);

  /* Create a matrix of the same type and size as src (for canny_img edge detection)*/
  canny_img.create(temporal_image.size(), temporal_image.type());
  /* Reduce noise with a kernel 3x3*/
  cv::blur(temporal_image, blur_img, cv::Size(3, 3));

  /* Canny detector*/
  cv::Canny(blur_img, detected_edges, 110, 255, 5);
  canny_img = cv::Scalar::all(0);
  temporal_image.copyTo(canny_img, detected_edges);
  /* Laplacian detector */
  cv::Laplacian(blur_img, laplace_img, CV_16S, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(laplace_img, laplace_img);
  /* Sobel - Gradient X */
  // Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  cv::Sobel(blur_img, sobel_grad_x, CV_16S, 1, 0, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(sobel_grad_x, sobel_grad_x);
  /* Sobel - Gradient Y*/
  // Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  cv::Sobel(blur_img, sobel_grad_y, CV_16S, 0, 1, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(sobel_grad_y, sobel_grad_y);
  /* Total Gradient (approximate) */
  cv::addWeighted(sobel_grad_x, 0.5, sobel_grad_y, 0.5, 0, sobel_img);
}


void NegObstc::SpacialSegmentation()
{
  pace = 0.2;
  nl = 40 / pace;
  nc = 40 / pace;
  N = nc * nl;
  j = s = lin = col = 0;

  /*clearing grid data*/
  density_points.clear();
  density_points.resize(N);
  densityMatrix.resize(nl, nc);
  densityMatrix.setZero(nl, nc);
  grad_points.clear();
  grad_points.resize((nc - 1) * (nl - 1));
  grad_x_points.clear();
  grad_x_points.resize((nc - 1) * (nl - 1));
  grad_y_points.clear();
  grad_y_points.resize((nc - 1) * (nl - 1));
  prewitt_points.clear();
  prewitt_points.resize((nc - 2) * (nl - 2));
  kirsh_points.clear();
  kirsh_points.resize((nc - 2) * (nl - 2));

  /*initalizing messages for OccupancyGrid construction*/
  info.height = nc;
  info.width = nl;
  info_grad.height = nc - 1;
  info_grad.width = nl - 1;
  info_sobel.height = nc - 2;
  info_sobel.width = nl - 2;
  info.resolution = info_grad.resolution = info_sobel.resolution = pace;
  header.frame_id = "moving_axis";
  info.map_load_time = header.stamp = info_grad.map_load_time = info_sobel.map_load_time = ros::Time(0);
  info.origin.position.x = info_grad.origin.position.x = info_sobel.origin.position.x = 0;
  info.origin.position.y = info_grad.origin.position.y = info_sobel.origin.position.y = -20;
  info.origin.position.z = info_grad.origin.position.z = info_sobel.origin.position.z = 0;
  info.origin.orientation.x = info_grad.origin.orientation.x = info_sobel.origin.orientation.x = 0;
  info.origin.orientation.y = info_grad.origin.orientation.y = info_sobel.origin.orientation.y = 0;
  info.origin.orientation.z = info_grad.origin.orientation.z = info_sobel.origin.orientation.z = 0;

  /*initializing grids with constructed messages*/
  densityGrid.header = gradGrid.header = gradXGrid.header = gradYGrid.header = sobelGrid.header = sobelGxGrid.header =
      sobelGyGrid.header = prewittGrid.header = kirshGrid.header = header;
  densityGrid.info = info;
  gradGrid.info = gradXGrid.info = gradYGrid.info = info_grad;
  sobelGxGrid.info = sobelGyGrid.info = prewittGrid.info = kirshGrid.info = info_sobel;

  /*cubelist marker with gradient colorbar*/
  gradient_2d grad[nc - 1][nl - 1];
  prewitt_grad prewitt_grad[nc - 2][nl - 2];
  kirsh_grad kirsh_grad[nc - 2][nl - 2];

  pcl_ros::transformPointCloud("moving_axis", ros::Time(0), *Cloud_Reconst, "map", *Transformed_cloud,
                               NegObstc::listener);

  /*point density calculation*/
  for (const pcl::PointXYZ &point : *Transformed_cloud)
  {
    if (point.x <= 40 && point.x >= 0 && point.y >= -20 && point.y <= 20)
    {
      lin = (int)floor(point.x / pace);
      col = (int)floor(point.y / pace) + 20 / pace;

      if (lin < nl && col < nc)
        densityMatrix(lin, col) += 1;
      if (lin + col * nl < N)
        density_points[lin + col * nl] += 1;
    }
  }
  max = densityMatrix.maxCoeff();
  densityGrid.data = density_points;

  // calculate gradient matrix
  for (int c = 0; c < nc - 1; c++)
  {
    for (int l = 0; l < nl - 1; l++)
    {
      grad[l][c].vertical = densityMatrix(l + 1, c) - densityMatrix(l, c);
      grad[l][c].horizontal = densityMatrix(l, c + 1) - densityMatrix(l, c);
      grad[l][c].grad_tot =
          abs(static_cast<double>(grad[l][c].vertical)) + abs(static_cast<double>(grad[l][c].horizontal));

      if (c < nc - 2 && l < nl - 2)
      {
        ls = l + 1;
        cs = c + 1;
        prewitt_grad[ls][cs].gx = (-1) * densityMatrix(ls + 1, cs - 1) - densityMatrix(ls, cs - 1) -
                                  densityMatrix(ls - 1, cs - 1) + densityMatrix(ls + 1, cs + 1) +
                                  densityMatrix(ls, cs + 1) + densityMatrix(ls - 1, cs + 1);
        prewitt_grad[ls][cs].gy = (-1) * densityMatrix(ls - 1, cs - 1) - densityMatrix(ls - 1) -
                                  densityMatrix(ls - 1, cs + 1) + densityMatrix(ls + 1, cs - 1) +
                                  densityMatrix(ls + 1, cs) + densityMatrix(ls + 1, cs + 1);

        prewitt_grad[ls][cs].prewitt = abs(prewitt_grad[ls][cs].gx) + abs(prewitt_grad[ls][cs].gy);

        kirsh_grad[ls][cs].gx = 5 * densityMatrix(ls + 1, cs - 1) - 3 * densityMatrix(ls + 1, cs) -
                                3 * densityMatrix(ls + 1, cs + 1) + 5 * densityMatrix(ls, cs - 1) -
                                3 * densityMatrix(ls, cs + 1) + 5 * densityMatrix(ls - 1, cs - 1) -
                                3 * densityMatrix(ls - 1, cs) - 3 * densityMatrix(ls - 1, cs + 1);
        kirsh_grad[ls][cs].gy = (-3) * densityMatrix(ls + 1, cs - 1) - 3 * densityMatrix(ls + 1, cs) -
                                3 * densityMatrix(ls + 1, cs + 1) - 3 * densityMatrix(ls, cs - 1) -
                                3 * densityMatrix(ls, cs + 1) + 5 * densityMatrix(ls - 1, cs - 1) +
                                5 * densityMatrix(ls - 1, cs) + 5 * densityMatrix(ls - 1, cs + 1);

        kirsh_grad[ls][cs].kirsh = abs(kirsh_grad[ls][cs].gx) / 5 + abs(kirsh_grad[ls][cs].gy) / 5;

        prewitt_points[s] = prewitt_grad[ls][cs].prewitt / (3 * max) * 100;
        kirsh_points[s] = kirsh_grad[ls][cs].kirsh;

        s++;
      }

      grad_points[j] = GradMag(grad[l][c].grad_tot, pace);
      grad_x_points[j] = Grad1D(grad[l][c].vertical, pace);
      grad_y_points[j] = Grad1D(grad[l][c].horizontal, pace);

      j++;
    }
  }

  gradGrid.data = grad_points;
  gradXGrid.data = grad_x_points;
  gradYGrid.data = grad_y_points;

  prewittGrid.data = prewitt_points;
  kirshGrid.data = kirsh_points;
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
    reconstruct.LoopFunction();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}