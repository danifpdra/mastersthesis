#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <boost/foreach.hpp>
/*Point cloud*/
#include <ros/ros.h>

/*math*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
/*msgs*/
#include <nav_msgs/Odometry.h>
#include <novatel_gps_msgs/Inspva.h>
#include "gps_common/GPSFix.h"
#include "gps_common/conversions.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
/* openCv*/
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <unistd.h>  //Sleep

using namespace std;

class QuantEval
{
public:
  QuantEval();
  void LoopFunction();
  void CloseHandle();
  void StartHandle();
  std::vector<string> ReadKml(string path);
  void ConvertToCoordinates();

private:
  // declaration of global variables
  ros::NodeHandle nh;

  double car_lat, car_lon, dx_r, dx_l, dy_r, dy_l, pace;
  int lin, col, nl, nc, N;
  double yaw, yaw_1;
  int n_vezes = 0;
  int gt_coincident;

  std::string str_i, str_f;
  std::ofstream handle, handle_csv;
  ros::Subscriber velocity_sub, grid_sub, laplacian_sub;
  ros::Publisher gps_pub, gt_pub, cleangrid_pub;

  // to read kml
  std::vector<string> coordinates_right, coordinates_left;
  std::vector<double> lat_right, lat_left, lon_right, lon_left, gt_dx_meters, gt_dy_meters;
  std::vector<int8_t> gt_points, cleaned_edges;
  std::string zone;

  std_msgs::Header header;
  nav_msgs::MapMetaData info;
  nav_msgs::OccupancyGrid GTGrid, EdgeGrid;
  gps_common::GPSFix gps_msg;
  geometry_msgs::Point Coord;

  Eigen::Matrix2d rotationMatrix;
  Eigen::Vector2d crd_r, crd_l, crd_r_correct, crd_l_correct;
  Eigen::Vector2d carUTM;
  Eigen::MatrixXd matToClean;

  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  void getDensityGrid(const nav_msgs::OccupancyGrid &msgGrid);
  void getEdgeGrid(const nav_msgs::OccupancyGrid &msgGrid);
  std::string FormatPlacemark(double lat1, double lon1);
  void DistanceToCar();
  double interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate);
  void StatisticMeasures();
  void CleanLimits();
};

/**********************************************************/
/**********************CONSTRUCTOR*************************/
QuantEval::QuantEval() : str_i({ "<coordinates>" }), str_f({ "</coordinates>" })
{
  velocity_sub = nh.subscribe("inspva", 10, &QuantEval::getVelocity, this);
  grid_sub = nh.subscribe("density_pub", 10, &QuantEval::getDensityGrid, this);
  laplacian_sub = nh.subscribe("laplacian_pub", 10, &QuantEval::getEdgeGrid, this);

  gps_pub = nh.advertise<gps_common::GPSFix>("gps_pub", 1, true);
  gt_pub = nh.advertise<nav_msgs::OccupancyGrid>("gt_pub", 1, true);
  cleangrid_pub = nh.advertise<nav_msgs::OccupancyGrid>("cleangrid_pub", 1, true);
}

/********************************************************/
/**********************CALLBACKS*************************/

void QuantEval::getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg)
{
  /*get car coordinates*/
  car_lat = gps_msg.latitude = velMsg->latitude;
  car_lon = gps_msg.longitude = velMsg->longitude;
  /*get car z orientation - relative to the world frame*/
  yaw = (velMsg->azimuth) * swri_math_util::_deg_2_rad;
  yaw_1 = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);
  /*transform coordinates to UTM and create rotation matrix*/
  gps_common::LLtoUTM(car_lat, car_lon, carUTM(0), carUTM(1), zone);
  rotationMatrix << cos(yaw_1), sin(yaw_1), -sin(yaw_1), cos(yaw_1);
}

void QuantEval::getDensityGrid(const nav_msgs::OccupancyGrid &msgGrid)
{
  /*get grid information to create new grids*/
  info = msgGrid.info;
  header = msgGrid.header;
  nc = msgGrid.info.height;
  nl = msgGrid.info.width;
  pace = msgGrid.info.resolution;
}

void QuantEval::getEdgeGrid(const nav_msgs::OccupancyGrid &msgGrid)
{
  /*get data from grid to study and place it in a matrix*/
  std::vector<int8_t> points_to_clean;
  nc = msgGrid.info.height;
  nl = msgGrid.info.width;
  matToClean.resize(nc, nl);
  points_to_clean.resize(N);
  points_to_clean = msgGrid.data;
  for (int idx = 0; idx < nc * nl; idx++)
  {
    int c = (int)(idx / nl);
    int l = idx - (c * nl);
    matToClean(l, c) = points_to_clean[idx];
  }
}
/************************************************************/

/************************************************************/
/**********************LOOP FUNCTION*************************/
void QuantEval::LoopFunction()
{
  if (car_lon < -7 && car_lon > -9 && car_lat > 39 && car_lat < 41 && pace > 0.1)
  {
    gps_pub.publish(gps_msg);
    // std::cout << "1" << std::endl;
    handle << FormatPlacemark(car_lat, car_lon);
    // std::cout << "2" << std::endl;
    DistanceToCar();
    // std::cout << "3" << std::endl;
    gt_pub.publish(GTGrid);
    // std::cout << "4" << std::endl;

    CleanLimits();
    // std::cout << "5" << std::endl;
    cleangrid_pub.publish(EdgeGrid);
    // std::cout << "6" << std::endl;

    if (gt_coincident != 0)
      StatisticMeasures();
    // std::cout << "7" << std::endl;
  }
}
/************************************************************/


std::vector<string> QuantEval::ReadKml(string path)
{
  std::vector<string> coordinates;
  std::ifstream handle_kml;
  std::string file_content;
  std::size_t found_i, found_f;
  std::stringstream strStream;

/*open kml files with groundtruth limits*/
  handle_kml.open(path);

  if (!handle_kml.is_open())
    std::cout << "could not open file" << std::endl;
  else
  {
    std::cout << "opened files" << std::endl;
    strStream << handle_kml.rdbuf();  // read the file
    file_content = strStream.str();
  }

  found_i = file_content.find(str_i); /*find beginning of coordinates*/
  found_f = file_content.find(str_f); /*find ending of coordinates*/

  handle_kml.close(); /*close handle*/

  /*cut string to contemplate only the coordinates*/
  std::string str = file_content.substr(found_i + 18, found_f - found_i - 19);

  /*separate in coordinates points*/
  std::stringstream ss(str);
  while (ss.good())
  {
    string substr;
    std::getline(ss, substr, ' ');
    coordinates.push_back(substr);
  }

  return coordinates;
}

void QuantEval::ConvertToCoordinates()
{
  coordinates_right = ReadKml("/home/daniela/catkin_ws/src/road_detection/Kml_files/RightPathAhead.kml");
  /*separate in latitude and longitude*/
  int n_coord = 1;
  for (auto const &point : coordinates_right)
  {
    int count = 1;
    std::stringstream sss(point);
    while (sss.good())
    {
      string string;
      std::getline(sss, string, ',');
      if (count == 1 && !string.empty() && n_coord < coordinates_right.size())
        lon_right.push_back(stof(string));
      if (count == 2 && !string.empty() && n_coord < coordinates_right.size())
        lat_right.push_back(stof(string));
      count++;
    }
    n_coord++;
  }

  coordinates_left = ReadKml("/home/daniela/catkin_ws/src/road_detection/Kml_files/LeftPathAhead.kml");
  n_coord = 1;
  for (auto const &point : coordinates_left)
  {
    int count = 1;
    std::stringstream sss(point);
    while (sss.good())
    {
      string string;
      std::getline(sss, string, ',');
      if (count == 1 && !string.empty() && n_coord < coordinates_left.size())
        lon_left.push_back(stof(string));
      if (count == 2 && !string.empty() && n_coord < coordinates_left.size())
        lat_left.push_back(stof(string));
      count++;
    }
    n_coord++;
  }
}

void QuantEval::DistanceToCar()
{
  N = nc * nl;

  gt_points.clear();
  gt_points.resize(N);

  gt_dx_meters.clear();
  gt_dy_meters.clear();

  GTGrid.info = info;
  GTGrid.header = header;
  // limite direito da estrada
  for (int i = 0; i < coordinates_right.size() - 1; i++)
  {
    gps_common::LLtoUTM(lat_right[i], lon_right[i], dx_r, dy_r, zone);

    crd_r << (-carUTM(0) + dx_r), (-carUTM(1) + dy_r);
    crd_r_correct = rotationMatrix * crd_r;
    gt_dx_meters.push_back(crd_r_correct(0) + 2.925);
    gt_dy_meters.push_back(-crd_r_correct(1));
  }
  // limite esquerdo da estrada
  for (int i = 0; i < coordinates_left.size() - 1; i++)
  {
    gps_common::LLtoUTM(lat_left[i], lon_left[i], dx_l, dy_l, zone);

    crd_l << (-carUTM(0) + dx_l), (-carUTM(1) + dy_l);
    crd_l_correct = rotationMatrix * crd_l;
    gt_dx_meters.push_back(crd_l_correct(0) + 2.925);
    gt_dy_meters.push_back(-crd_l_correct(1));
  }
  // interpolate
  for (int i = 0; i < coordinates_left.size() + coordinates_right.size(); i++)
  {
    // std::cout << "Dx: " << gt_dx_meters[i] << " and dy is: " << gt_dy_meters[i] << std::endl;
    for (int n = 1; n < (int)floor((gt_dx_meters[i + 1] - gt_dx_meters[i]) / pace); n++)
    {
      std::vector<double> xData = { gt_dx_meters[i], gt_dx_meters[i + 1] };
      std::vector<double> yData = { gt_dy_meters[i], gt_dy_meters[i + 1] };
      double x = gt_dx_meters[i] + pace * n;
      double y = interpolate(xData, yData, x, true);
      gt_dx_meters.push_back(x);
      gt_dy_meters.push_back(y);
    }
  }
  gt_coincident = 0;
  for (int i = 0; i < gt_dx_meters.size(); i++)
  {
    if (gt_dx_meters[i] <= (double)40 && gt_dx_meters[i] >= (double)0 && gt_dy_meters[i] >= (double)-20 &&
        gt_dy_meters[i] <= (double)20)
    {
      lin = (int)floor(gt_dx_meters[i] / pace);
      col = (int)floor(gt_dy_meters[i] / pace) + 20 / pace;

      if (lin + col * nl < N && lin < nl && col < nc)
      {
        gt_points[lin + col * nl] = 100;
      }
      gt_coincident++;
    }
  }
  GTGrid.data.clear();
  GTGrid.data = gt_points;
}

/**
 * @brief Function to only consider the first right and left limit to the car
 *
 */
void QuantEval::CleanLimits()
{
  cleaned_edges.clear();
  cleaned_edges.resize(N);

  for (int i = 0; i < nl; i++)
  {
    int stop = 0;
    int idx = nc / 2;
    while (stop == 0 && idx < nc && i + idx * nl < N)
    {
      if (matToClean(i, idx) != 0)
      {
        cleaned_edges[i + idx * nl] = 100;
        stop = 1;
      }
      idx++;
    }
    stop = 0;
    idx = nc / 2;
    while (stop == 0 && idx > 0 && i + idx * nl < N)
    {
      if (matToClean(i, idx) != 0)
      {
        cleaned_edges[i + idx * nl] = 100;
        stop = 1;
      }
      idx--;
    }
  }

  EdgeGrid.info = info;
  EdgeGrid.header = header;
  EdgeGrid.data = cleaned_edges;
}

void QuantEval::StatisticMeasures()
{
  int TP, FP, TN, FN;
  double PPV, TNR, NPV, TPR;
  TP = FP = TN = FN = PPV = TNR = NPV = TPR = 0;

  if (handle_csv.is_open())
  {
    for (int i = 0; i < gt_points.size() - 1; i++)
    {
      std::cout << "GT: " << (int)gt_points[i] << " ; limits: " << (int)(cleaned_edges[i]) << std::endl;
      if ((int)gt_points[i] == (int)cleaned_edges[i] && (int)gt_points[i] == 0)  // true negative
      {
        TN++;
      }
      else if (((int)gt_points[i] == (int)cleaned_edges[i] || (int)gt_points[i] == (int)cleaned_edges[i - 1] ||
                (int)gt_points[i] == (int)cleaned_edges[i + 1]) &&
               (int)gt_points[i] == 100)  // true positive
      {
        TP++;
      }
      else if ((int)gt_points[i] != (int)cleaned_edges[i] && (int)cleaned_edges[i] == 100)  // false positive
      {
        FP++;
      }
      else if ((int)gt_points[i] != (int)cleaned_edges[i] && (int)cleaned_edges[i] == 0)  // false negative
      {
        FN++;
      }
      else
      {
        ROS_WARN("Opsssssssssssss");
      }
    }

    std::cout << "TN: " << TN << " ; TP: " << TP << " ; FP: " << FP << " ; FN: " << FN << std::endl;

    switch (TN)
    {
      case 0:
        TNR = 0;
        NPV = 0;
        break;
      default:
        TNR = static_cast<double>(TN) / static_cast<double>(FP + TN);  // sensitivity
        NPV = static_cast<double>(TN) / static_cast<double>(TN + FN);
    }

    switch (TP)
    {
      case 0:
        PPV = 0;
        TPR = 0;
        break;
      default:
        PPV = static_cast<double>(TP) / static_cast<double>(TP + FP);
        TPR = static_cast<double>(TP) / static_cast<double>(TP + FN);
    }

    handle_csv << PPV << "," << TNR << "," << NPV << "," << TPR << "\n";
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
  ros::init(argc, argv, "QuantEval");
  QuantEval reconstruct;
  reconstruct.StartHandle();
  reconstruct.ConvertToCoordinates();

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.LoopFunction();
    ros::spinOnce();
    rate.sleep();
  }

  reconstruct.CloseHandle();

  return 0;
}

void QuantEval::StartHandle()
{
  handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  handle_csv.exceptions(std::ofstream::failbit | std::ofstream::badbit);

  char filename[150];
  char filename_csv[150];

  /*get time*/
  time_t theTime = time(NULL);
  struct tm *aTime = localtime(&theTime);
  int hour = aTime->tm_hour;
  int min = aTime->tm_min;

  /*KML file with car path*/
  sprintf(filename, "/home/daniela/catkin_ws/src/road_detection/eval_api/Results/Path_%dH_%dM.kml", hour, min);
  // Open the KML file for writing:
  handle.open(filename);
  // Write to the KML file:
  handle << "<?xml version='1.0' encoding='utf-8'?>\n";
  handle << "<kml xmlns='http://www.opengis.net/kml/2.2'>\n";
  handle << "<Placemark>\n";
  handle << "<description>Path to evaluate road limits' precision</description>\n";
  handle << "<styleUrl>#pathstyle</styleUrl>\n";
  handle << "<LineString>\n";
  handle << "<tessellate>1</tessellate>\n";
  handle << "<coordinates>";

  /*CSV file with statistical measures*/
  sprintf(filename_csv, "/home/daniela/catkin_ws/src/road_detection/eval_api/Results/CSV/Measures%dH_%dM.csv", hour,
          min);
  // Open the KML file for writing:
  handle_csv.open(filename_csv);
  handle_csv << "PPV,TNR,NPV,TPR\n";
}

std::string QuantEval::FormatPlacemark(double lat1, double lon1)
{
  std::ostringstream ss;
  if (car_lon < -7 && car_lon > -9 && car_lat > 39 && car_lat < 41)
    handle << " " << std::setprecision(20) << lon1 << "," << lat1 << ",0";
  return ss.str();
}

void QuantEval::CloseHandle()
{
  handle << "</coordinates>\n";
  handle << "</LineString>\n";
  handle << "</Placemark>\n";
  handle << "</kml>\n";
  handle.close();
  handle_csv.close();
}

double QuantEval::interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate)
{
  int size = xData.size();

  int i = 0;                 // find left end of interval for interpolation
  if (x >= xData[size - 2])  // special case: beyond right end
  {
    i = size - 2;
  }
  else
  {
    while (x > xData[i + 1])
      i++;
  }
  double xL = xData[i], yL = yData[i], xR = xData[i + 1],
         yR = yData[i + 1];  // points on either side (unless beyond ends)
  if (!extrapolate)          // if beyond ends of array and not extrapolating
  {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  double dydx = (yR - yL) / (xR - xL);  // gradient

  return yL + dydx * (x - xL);  // linear interpolation
}