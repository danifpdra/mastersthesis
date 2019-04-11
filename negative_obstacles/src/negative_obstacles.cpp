/*Point cloud*/
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
/*math*/
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
/*msgs*/
#include <novatel_gps_msgs/Inspva.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
/* openCv*/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

/*Grid maps*/
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
  ros::NodeHandle nh;
  /*pcl*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Reconst, Transformed_cloud;

  /*others*/
  double pace;
  float max, carVelocity, norm;
  int N, j, s, writeCount, lin, col, nc, nl, ls, cs, lin_up, col_up, lin_down, col_down;
  Eigen::MatrixXd densityMatrix;
  std::vector<int8_t> density_points, density_points_up, density_points_down, grad_points, grad_x_points, grad_y_points,
      prewitt_points, kirsh_points;
  cv::Mat canny_img, detected_edges, laplace_img, temporal_image, blur_img, sobel_img, sobel_grad_x, sobel_grad_y;

  /*publishers and subscribers*/
  ros::Subscriber sub, velocity_sub;
  ros::Publisher density_pub, density_pub_up, density_pub_down, grad_pub, grad_x_pub, grad_y_pub, sobel_gx_pub,
      sobel_gy_pub, sobel_pub, prewitt_pub, kirsh_pub, canny_pub, laplacian_pub;

  /*ROS*/
  tf::StampedTransform transform;
  tf::TransformListener listener;
  ros::Publisher img_pub_canny, img_pub_laplace, img_pub_density;
  image_transport::ImageTransport it;
  grid_map::GridMap temporalGridMap, cannyGridMap, laplaceGridMap, sobelGridMap, sobelGxGridMap, sobelGyGridMap;

  /*messages*/
  std_msgs::Header header;
  nav_msgs::MapMetaData info, info_grad, info_sobel;
  nav_msgs::OccupancyGrid densityGrid, densityGridUp, densityGridDown, gradGrid, gradXGrid, gradYGrid, sobelGxGrid,
      sobelGyGrid, sobelGrid, prewittGrid, kirshGrid, cannyGrid, laplaceGrid;
  sensor_msgs::ImagePtr msg_canny, msg_laplace, msg_sobel, msg_sobelX, msg_sobelY, msg_density;

  /*functions*/
  void DensityCalculation();
  void EdgeDetection();
  void Publishers();
  void ImageConversion();
  void GradientCalculation();
  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  void GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

/**
 * @brief Construct a new Neg Obstc:: Neg Obstc object
 *
 */
NegObstc::NegObstc()
    : it(nh), temporalGridMap({"density"}), cannyGridMap({"canny"}), laplaceGridMap({"laplacian"}), sobelGridMap({"sobel"}), sobelGxGridMap({"sobelX"}), sobelGyGridMap({"sobelY"})
{
  /*publishers and subscribers*/
  density_pub = nh.advertise<nav_msgs::OccupancyGrid>("density_pub", 1, true);
  density_pub_up = nh.advertise<nav_msgs::OccupancyGrid>("density_pub_up", 1, true);
  density_pub_down = nh.advertise<nav_msgs::OccupancyGrid>("density_pub_down", 1, true);

  grad_pub = nh.advertise<nav_msgs::OccupancyGrid>("grad_pub", 1, true);
  grad_x_pub = nh.advertise<nav_msgs::OccupancyGrid>("grad_x_pub", 1, true);
  grad_y_pub = nh.advertise<nav_msgs::OccupancyGrid>("grad_y_pub", 1, true);

  sobel_pub = nh.advertise<nav_msgs::OccupancyGrid>("sobel_pub", 1, true);
  sobel_gx_pub = nh.advertise<nav_msgs::OccupancyGrid>("sobel_gx_pub", 1, true);
  sobel_gy_pub = nh.advertise<nav_msgs::OccupancyGrid>("sobel_gy_pub", 1, true);

  prewitt_pub = nh.advertise<nav_msgs::OccupancyGrid>("prewitt_pub", 1, true);
  kirsh_pub = nh.advertise<nav_msgs::OccupancyGrid>("kirsh_pub", 1, true);

  img_pub_canny = nh.advertise<sensor_msgs::Image>("canny_img", 10);
  img_pub_laplace = nh.advertise<sensor_msgs::Image>("laplace_img", 10);
  img_pub_density = nh.advertise<sensor_msgs::Image>("density", 10);
  canny_pub = nh.advertise<nav_msgs::OccupancyGrid>("canny_pub", 1, true);
  laplacian_pub = nh.advertise<nav_msgs::OccupancyGrid>("laplacian_pub", 1, true);

  sub = nh.subscribe<sensor_msgs::PointCloud2>("/road_reconstruction", 1, &NegObstc::GetPointCloud, this);
  velocity_sub = nh.subscribe("inspva", 10, &NegObstc::getVelocity, this);

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
  if (Cloud_Reconst->points.size() != 0)
  {
    DensityCalculation();
    GradientCalculation();
    EdgeDetection();
    ImageConversion();
    Publishers();
  }
}

void NegObstc::Publishers()
{
  /*occupancy grids*/
  density_pub.publish(densityGrid);
  // density_pub_up.publish(densityGridUp);
  // density_pub_down.publish(densityGridDown);
  grad_pub.publish(gradGrid);
  grad_x_pub.publish(gradXGrid);
  grad_y_pub.publish(gradYGrid);

  prewitt_pub.publish(prewittGrid);
  kirsh_pub.publish(kirshGrid);

  canny_pub.publish(cannyGrid);
  laplacian_pub.publish(laplaceGrid);

  sobel_pub.publish(sobelGrid);
  sobel_gx_pub.publish(sobelGxGrid);
  sobel_gy_pub.publish(sobelGyGrid);

  /*images*/
  img_pub_density.publish(msg_density);
  img_pub_laplace.publish(msg_laplace);
  img_pub_canny.publish(msg_canny);
}

void NegObstc::getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg)
{
  carVelocity =
      sqrt(std::pow(velMsg->north_velocity, 2) + std::pow(velMsg->east_velocity, 2) + std::pow(velMsg->up_velocity, 2));

  ROS_WARN("Car velocity: %f", carVelocity);
}

void NegObstc::GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg.get(), *Cloud_Reconst);
}

void NegObstc::ImageConversion()
{
  msg_density = cv_bridge::CvImage{header, "mono8", temporal_image}.toImageMsg();

  msg_canny = cv_bridge::CvImage{header, "mono8", canny_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_canny, pace, cannyGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_canny, "canny", cannyGridMap, 0, 255, 255);
  grid_map::GridMapRosConverter::toOccupancyGrid(cannyGridMap, "canny", 150, 255, cannyGrid);
  cannyGrid.info = info;

  msg_laplace = cv_bridge::CvImage{header, "mono8", laplace_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_laplace, pace, laplaceGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_laplace, "laplacian", laplaceGridMap, 0, 255, 255);
  grid_map::GridMapRosConverter::toOccupancyGrid(laplaceGridMap, "laplacian", 175, 255, laplaceGrid);
  laplaceGrid.info = info;

  msg_sobel = cv_bridge::CvImage{header, "mono8", sobel_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_sobel, pace, sobelGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobel, "sobel", sobelGridMap, 0, 255, 125);
  grid_map::GridMapRosConverter::toOccupancyGrid(sobelGridMap, "sobel", 130, 255, sobelGrid);

  msg_sobelX = cv_bridge::CvImage{header, "mono8", sobel_grad_x}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_sobelX, pace, sobelGxGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobelX, "sobelX", sobelGxGridMap, 0, 255, 125);
  grid_map::GridMapRosConverter::toOccupancyGrid(sobelGxGridMap, "sobelX", 150, 255, sobelGxGrid);

  msg_sobelY = cv_bridge::CvImage{header, "mono8", sobel_grad_y}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_sobelY, pace, sobelGyGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobelY, "sobelY", sobelGyGridMap, 0, 255, 125);
  grid_map::GridMapRosConverter::toOccupancyGrid(sobelGyGridMap, "sobelY", 150, 255, sobelGyGrid);
  sobelGrid.info = sobelGxGrid.info = sobelGyGrid.info = info;
}

/**
 * @function EdgeDetection
 * @brief Trackbar EdgeDetection - Detect edges from density grids with different filters
 */
void NegObstc::EdgeDetection()
{
  int x, y, kernel_size = 3;

  grid_map::GridMapRosConverter::fromOccupancyGrid(densityGrid, "density", temporalGridMap);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(temporalGridMap, "density", CV_8UC1, temporal_image);
  /* Create a matrix of the same type and size as src (for canny_img edge detection)*/
  canny_img.create(temporal_image.size(), temporal_image.type());
  /* Reduce noise with a kernel 3x3*/
  // cv::blur(temporal_image, blur_img, cv::Size(3, 3));
  cv::threshold(temporal_image, blur_img, 170, 255, cv::THRESH_OTSU);
  int nz = cv::countNonZero(temporal_image);
  temporal_image.at<uchar>(y, x) = 0;
  if (nz < N / 2)
    cv::threshold(blur_img, blur_img, 150, 255, cv::THRESH_BINARY_INV);
  /* Canny detector*/
  cv::Canny(blur_img, detected_edges, 200, 255, 3);
  canny_img = cv::Scalar::all(0);
  temporal_image.copyTo(canny_img, detected_edges);
  /* Laplacian detector */
  cv::Laplacian(temporal_image, laplace_img, CV_16S, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(laplace_img, laplace_img);
  cv::fastNlMeansDenoising(laplace_img, laplace_img, 3, 7, 21);
  /* Sobel - Gradient X */
  // Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  cv::Sobel(temporal_image, sobel_grad_x, CV_16S, 1, 0, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(sobel_grad_x, sobel_grad_x);
  /* Sobel - Gradient Y*/
  // Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  cv::Sobel(temporal_image, sobel_grad_y, CV_16S, 0, 1, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(sobel_grad_y, sobel_grad_y);
  /* Total Gradient (approximate) */
  cv::addWeighted(sobel_grad_x, 0.5, sobel_grad_y, 0.5, 0, sobel_img);
}

void NegObstc::DensityCalculation()
{
  pace = 0.4; //((float)((int)(pace * 10))) / 10;
  nl = 40 / pace;
  nc = 40 / pace;
  N = nc * nl;
  j = s = lin = col = 0;
  norm = (float)floor((200 * std::pow(pace, 2)) / (carVelocity * 0.5));

  /*clearing grid data*/
  density_points.clear();
  density_points.resize(N);
  density_points_up.clear();
  density_points_up.resize(100 * 100);
  density_points_down.clear();
  density_points_down.resize(80 * 80);
  densityMatrix.resize(nl, nc);
  densityMatrix.setZero(nl, nc);

  pcl_ros::transformPointCloud("moving_axis", ros::Time(0), *Cloud_Reconst, "map", *Transformed_cloud,
                               NegObstc::listener);

  /*point density calculation*/
  for (const pcl::PointXYZ &point : *Transformed_cloud)
  {
    if (point.x <= 40 && point.x >= 0 && point.y >= -20 && point.y <= 20)
    {
      lin = (int)floor(point.x / pace);
      col = (int)floor(point.y / pace) + 20 / pace;

      lin_up = (int)floor(point.x / 0.4);
      col_up = (int)floor(point.y / 0.4) + 20 / 0.4;

      lin_down = (int)floor(point.x / 0.5);
      col_down = (int)floor(point.y / 0.5) + 20 / 0.5;

      if (lin < nl && col < nc)
        densityMatrix(lin, col) += 1;
      if (lin + col * nl < N)
      {
        density_points[lin + col * nl] += 1;
        density_points_up[lin_up + col_up * 100] += 1;
        density_points_down[lin_down + col_down * 80] += 1;
      }
    }
  }
  max = densityMatrix.maxCoeff();
  ROS_WARN("Norm value: %f; Max density: %f; ponto 200: %d", norm, max, density_points[5500]);
  densityGrid.data = density_points;
  densityGridUp.data = density_points_up;
  densityGridDown.data = density_points_down;
}
void NegObstc::GradientCalculation()
{
  /*reset point arrays*/
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
  densityGrid.header = densityGridUp.header = densityGridDown.header = gradGrid.header = gradXGrid.header =
      gradYGrid.header = prewittGrid.header = kirshGrid.header = header;
  densityGrid.info = densityGridUp.info = densityGridDown.info = info;
  gradGrid.info = gradXGrid.info = gradYGrid.info = info_grad;
  sobelGxGrid.info = sobelGyGrid.info = prewittGrid.info = kirshGrid.info = info_sobel;

  densityGridUp.info.height = densityGridUp.info.width = 100;
  densityGridUp.info.origin.position.z = 40;
  densityGridDown.info.height = densityGridDown.info.width = 80;
  densityGridDown.info.resolution = 0.5;
  densityGridUp.info.resolution = 0.4;
  densityGridDown.info.origin.position.z = 20;

  /*cubelist marker with gradient colorbar*/
  gradient_2d grad[nc - 1][nl - 1];
  prewitt_grad prewitt_grad[nc - 2][nl - 2];
  kirsh_grad kirsh_grad[nc - 2][nl - 2];
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

        prewitt_points[s] = Threshold(prewitt_grad[ls][cs].prewitt / (3 * max) * 100, 50);
        kirsh_points[s] = Threshold(kirsh_grad[ls][cs].kirsh, 50);

        s++;
      }

      grad_points[j] = Threshold(grad[l][c].grad_tot, 50);
      grad_x_points[j] = Threshold(abs(grad[l][c].vertical), 50);
      grad_y_points[j] = Threshold(abs(grad[l][c].horizontal), 50);

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
