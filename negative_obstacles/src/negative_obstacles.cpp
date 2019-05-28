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
  float max, carVelocity;
  int N, lin, col, nc, nl;
  Eigen::MatrixXd densityMatrix;
  std::vector<int8_t> density_points;
  cv::Mat canny_img, detected_edges, laplace_img, density_image, blur_img, sobel_img, prewitt_img, gx_img, gy_img, grad_mag_img, kirsh_img;

  /*publishers and subscribers*/
  ros::Subscriber pcl_sub, velocity_sub;
  ros::Publisher density_pub, grad_pub, grad_x_pub, grad_y_pub, sobel_pub, prewitt_pub, kirsh_pub, canny_pub, laplacian_pub;

  /*ROS*/
  tf::StampedTransform transform;
  tf::TransformListener listener;
  ros::Publisher img_pub_canny, img_pub_laplace, img_pub_density;
  image_transport::ImageTransport it;
  grid_map::GridMap temporalGridMap, cannyGridMap, laplaceGridMap, sobelGridMap, prewittGridMap, gxGridMap, gyGridMap, gmagGridMap, kirshGridMap;

  /*messages*/
  std_msgs::Header header;
  nav_msgs::MapMetaData info;
  nav_msgs::OccupancyGrid densityGrid, gradGrid, gradXGrid, gradYGrid, sobelGrid, prewittGrid, kirshGrid, cannyGrid, laplaceGrid;
  sensor_msgs::ImagePtr msg_canny, msg_laplace, msg_sobel, msg_density, msg_prewitt, msg_gx, msg_gy, msg_gmag, msg_kirsh;

  /*functions*/
  void DensityCalculation();
  void EdgeDetection();
  void Publishers();
  void ImageConversion();
  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  void GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

/**********************************************************/
/**********************CONSTRUCTOR*************************/
/**
 * @brief Construct a new Neg Obstc:: Neg Obstc object
 *
 */
NegObstc::NegObstc()
    : it(nh), temporalGridMap({"density"}), cannyGridMap({"canny"}), laplaceGridMap({"laplacian"}), sobelGridMap({"sobel"}), prewittGridMap({"prewitt"}), gxGridMap({"gx"}), gyGridMap({"gy"}), gmagGridMap({"grad_mag"}), kirshGridMap({"kirsh"})
{
  /*publishers and subscribers*/
  density_pub = nh.advertise<nav_msgs::OccupancyGrid>("density_pub", 1, true);

  grad_pub = nh.advertise<nav_msgs::OccupancyGrid>("grad_pub", 1, true);
  grad_x_pub = nh.advertise<nav_msgs::OccupancyGrid>("grad_x_pub", 1, true);
  grad_y_pub = nh.advertise<nav_msgs::OccupancyGrid>("grad_y_pub", 1, true);

  sobel_pub = nh.advertise<nav_msgs::OccupancyGrid>("sobel_pub", 1, true);
  prewitt_pub = nh.advertise<nav_msgs::OccupancyGrid>("prewitt_pub", 1, true);
  kirsh_pub = nh.advertise<nav_msgs::OccupancyGrid>("kirsh_pub", 1, true);

  img_pub_canny = nh.advertise<sensor_msgs::Image>("canny_img", 10);
  img_pub_laplace = nh.advertise<sensor_msgs::Image>("laplace_img", 10);
  img_pub_density = nh.advertise<sensor_msgs::Image>("density", 10);
  canny_pub = nh.advertise<nav_msgs::OccupancyGrid>("canny_pub", 1, true);
  laplacian_pub = nh.advertise<nav_msgs::OccupancyGrid>("laplacian_pub", 1, true);

  pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/Cloud_Acum", 1, &NegObstc::GetPointCloud, this);
  velocity_sub = nh.subscribe("inspva", 10, &NegObstc::getVelocity, this);

  /*initialize pointers*/
  Cloud_Reconst.reset(new pcl::PointCloud<pcl::PointXYZ>);
  Transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

/************************************************************/
/**********************LOOP FUNCTION*************************/
/**
 * @brief Loop function to calculate negative obstacles
 *
 */
void NegObstc::LoopFunction()
{
  if (Cloud_Reconst->points.size() != 0)
  {
    DensityCalculation();
    EdgeDetection();
    ImageConversion();
    Publishers();
  }
}

/*********************************************************/
/**********************PUBLISHERS*************************/
void NegObstc::Publishers()
{
  // std::cout << "here" << std::endl;
  /*occupancy grids*/
  density_pub.publish(densityGrid);

  grad_pub.publish(gradGrid);
  grad_x_pub.publish(gradXGrid);
  grad_y_pub.publish(gradYGrid);

  prewitt_pub.publish(prewittGrid);
  kirsh_pub.publish(kirshGrid);

  canny_pub.publish(cannyGrid);
  laplacian_pub.publish(laplaceGrid);

  sobel_pub.publish(sobelGrid);
  /*images*/
  img_pub_density.publish(msg_density);
  img_pub_laplace.publish(msg_laplace);
  img_pub_canny.publish(msg_canny);
}

/********************************************************/
/**********************CALLBACKS*************************/

void NegObstc::getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg)
{
  carVelocity =
      sqrt(std::pow(velMsg->north_velocity, 2) + std::pow(velMsg->east_velocity, 2) + std::pow(velMsg->up_velocity, 2));

  // ROS_WARN("Car velocity: %f", carVelocity);s
}

void NegObstc::GetPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg.get(), *Cloud_Reconst);
}

/********************************************************/
/**********************FUNCTIONS*************************/
/**
 * @brief Convert all images back to occupancy grids
 * 
 */
void NegObstc::ImageConversion()
{
  msg_density = cv_bridge::CvImage{header, "mono8", density_image}.toImageMsg();

  msg_canny = cv_bridge::CvImage{header, "mono8", canny_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_canny, pace, cannyGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_canny, "canny", cannyGridMap, 0, 255,0);
  grid_map::GridMapRosConverter::toOccupancyGrid(cannyGridMap, "canny", 50, 255, cannyGrid);

  /*try to remove small blobs*/
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1));
  cv::morphologyEx(laplace_img, laplace_img, CV_MOP_OPEN, element);
  msg_laplace = cv_bridge::CvImage{header, "mono8", laplace_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_laplace, pace, laplaceGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_laplace, "laplacian", laplaceGridMap, 0, 255, 0);
  grid_map::GridMapRosConverter::toOccupancyGrid(laplaceGridMap, "laplacian", 120, 255, laplaceGrid);

  msg_sobel = cv_bridge::CvImage{header, "mono8", sobel_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_sobel, pace, sobelGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_sobel, "sobel", sobelGridMap, 0, 255, 0);
  grid_map::GridMapRosConverter::toOccupancyGrid(sobelGridMap, "sobel", 50, 255, sobelGrid);

  msg_prewitt = cv_bridge::CvImage{header, "mono8", prewitt_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_prewitt, pace, prewittGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_prewitt, "prewitt", prewittGridMap, 0, 255, 0);
  grid_map::GridMapRosConverter::toOccupancyGrid(prewittGridMap, "prewitt", 50, 255, prewittGrid);

  msg_kirsh = cv_bridge::CvImage{header, "mono8", kirsh_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_kirsh, pace, kirshGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_kirsh, "kirsh", kirshGridMap, 0, 255, 0);
  grid_map::GridMapRosConverter::toOccupancyGrid(kirshGridMap, "kirsh", 120, 255, kirshGrid);

  msg_gmag = cv_bridge::CvImage{header, "mono8", grad_mag_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_gmag, pace, gmagGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_gmag, "grad_mag", gmagGridMap, 0, 255, 0);
  grid_map::GridMapRosConverter::toOccupancyGrid(gmagGridMap, "grad_mag", 0, 255, gradGrid);

  msg_gx = cv_bridge::CvImage{header, "mono8", gx_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_gx, pace, gxGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_gx, "gx", gxGridMap, 0, 255, 0.5);
  grid_map::GridMapRosConverter::toOccupancyGrid(gxGridMap, "gx", 120, 255, gradXGrid);

  msg_gy = cv_bridge::CvImage{header, "mono8", gy_img}.toImageMsg();
  grid_map::GridMapRosConverter::initializeFromImage(*msg_gy, pace, gyGridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(*msg_gy, "gy", gyGridMap, 0, 255, 0.5);
  grid_map::GridMapRosConverter::toOccupancyGrid(gyGridMap, "gy", 120, 255, gradYGrid);

  /*give proper headers and info to grids*/
  gradGrid.info = gradXGrid.info = gradYGrid.info = sobelGrid.info = prewittGrid.info = kirshGrid.info = laplaceGrid.info = cannyGrid.info = info;
}

/**
 * @function EdgeDetection
 * @brief Trackbar EdgeDetection - Detect edges from density grids with different filters
 */
void NegObstc::EdgeDetection()
{
  int x, y, kernel_size = 3;

  /*convert density grid to image to performance edge detection*/
  grid_map::GridMapRosConverter::fromOccupancyGrid(densityGrid, "density", temporalGridMap);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(temporalGridMap, "density", CV_8UC1, density_image);

  /* Create a matrix of the same type and size as src (for canny_img edge detection)*/
  canny_img.create(density_image.size(), density_image.type());

  /* Reduce noise with a kernel 3x3*/
  bitwise_not(density_image, blur_img);

  /* Canny detector*/
  cv::Canny(blur_img, detected_edges, 160, 255, 3);
  canny_img = cv::Scalar::all(0);
  density_image.copyTo(canny_img, detected_edges);

  /* Laplacian edge detector */
  cv::Laplacian(density_image, laplace_img, CV_16S, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(laplace_img, laplace_img);
  /* Sobel edge detector */
  cv::Mat sobel_grad_x, sobel_grad_y;
  cv::Sobel(density_image, sobel_grad_x, CV_16S, 1, 0, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(sobel_grad_x, sobel_grad_x);
  cv::Sobel(density_image, sobel_grad_y, CV_16S, 0, 1, kernel_size, 1, 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(sobel_grad_y, sobel_grad_y);
  cv::addWeighted(sobel_grad_x, 0.5, sobel_grad_y, 0.5, 0, sobel_img); // Total Gradient (approximate)

  /*Prewitt edge detector*/
  cv::Mat kernel_prewitt_x = (cv::Mat_<int>(3, 3) << -1, 0, 1, -1, 0, 1, -1, 0, 1);
  cv::Mat kernel_prewitt_y = (cv::Mat_<int>(3, 3) << -1, -1, -1, 0, 0, 0, 1, 1, 1);
  cv::Mat prewitt_img_x, prewitt_img_y;
  cv::filter2D(density_image, prewitt_img_x, CV_8UC1, kernel_prewitt_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::filter2D(density_image, prewitt_img_y, CV_8UC1, kernel_prewitt_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(prewitt_img_x, prewitt_img_x);
  cv::convertScaleAbs(prewitt_img_y, prewitt_img_y);
  cv::addWeighted(prewitt_img_x, 0.5, prewitt_img_y, 0.5, 0, prewitt_img);

  /*Gradient edge detector*/
  cv::Mat kernel_gx = (cv::Mat_<int>(3, 3) << 0, 0, 0, -1, 1, 0, 0, 0, 0);
  cv::Mat kernel_gy = (cv::Mat_<int>(3, 3) << 0, -1, 0, 0, 1, 0, 0, 0, 0);
  cv::filter2D(density_image, gx_img, CV_8UC1, kernel_gx, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::filter2D(density_image, gy_img, CV_8UC1, kernel_gy, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(gx_img, gx_img);
  cv::convertScaleAbs(gy_img, gy_img);
  cv::addWeighted(gx_img, 0.5, gy_img, 0.5, 0, grad_mag_img);

  /*Kirsh edge detector*/
  cv::Mat kernel_kirshx = (cv::Mat_<int>(3, 3) << -3, -3, -3, -3, 0, -3, 5, 5, 5);
  cv::Mat kernel_kirshy = (cv::Mat_<int>(3, 3) << 5, -3, -3, 5, 0, -3, 5, -3, -3);
  cv::Mat kirshx_img, kirshy_img;
  cv::filter2D(density_image, kirshx_img, CV_8UC1, kernel_kirshx, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::filter2D(density_image, kirshy_img, CV_8UC1, kernel_kirshy, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(kirshx_img, kirshx_img);
  cv::convertScaleAbs(kirshy_img, kirshy_img);
  cv::addWeighted(kirshx_img, 0.5, kirshy_img, 0.5, 0, kirsh_img);
}

void NegObstc::DensityCalculation()
{
  pace = 0.4;
  nl = 40 / pace;
  nc = 40 / pace;
  N = nc * nl;
  lin = col = 0;

  /*initalizing messages for OccupancyGrid construction*/
  info.height = nc;
  info.width = nl;
  info.resolution = pace;
  header.frame_id = "moving_axis";
  info.map_load_time = header.stamp = ros::Time(0);
  info.origin.position.x = 0;
  info.origin.position.y = -20;
  info.origin.position.z = 0;
  info.origin.orientation.x = 0;
  info.origin.orientation.y = 0;
  info.origin.orientation.z = 0;

  /*clearing grid data*/
  density_points.clear();
  density_points.resize(N);
  densityMatrix.resize(nl, nc);
  densityMatrix.setZero(nl, nc);

  std::fill(density_points.begin(), density_points.end(), 0);

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
      {
        density_points[lin + col * nl] += 1;
      }
    }
  }
  max = densityMatrix.maxCoeff();
  densityMatrix = densityMatrix * (100 / max);
  // // ROS_WARN("Norm value: %f; Max density: %f; ponto 5500: %d", norm, max, density_points[5500]);
  for (int it = 0; it < N; it++)
  {
    density_points[it] = density_points[it] / max * 100;
  }
  densityGrid.data = density_points;
  densityGrid.info = info;
  densityGrid.header = header;
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
