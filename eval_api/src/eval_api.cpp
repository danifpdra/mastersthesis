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
#include <novatel_gps_msgs/Inspva.h>
#include "gps_common/GPSFix.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
/* openCv*/
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
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
  void ReadKml();

private:
  ros::NodeHandle nh;

  double car_lat, car_lon, dx_r, dx_l, dy_r, dy_l, pace, dx_rot_r, dx_rot_l, dy_rot_r, dy_rot_l;
  int lin, col, nl, nc, N;
  double yaw, yaw_1;

  std::ofstream handle;
  std::ifstream handle_kml_right, handle_kml_left;
  ros::Subscriber velocity_sub, grid_sub;
  ros::Publisher gps_pub, gt_pub;

  // to read kml
  std::string file_content_right, file_content_left;
  std::string str_i, str_f;
  std::size_t found_i_r, found_f_r, found_i_l, found_f_l;
  std::stringstream strStream_r, strStream_l;
  std::vector<string> coordinates_right, coordinates_left;
  std::vector<double> lat_right, lat_left, lon_right, lon_left, lat_dx_meters, lon_dy_meters;
  std::vector<int8_t> gt_points;

  std_msgs::Header header;
  nav_msgs::MapMetaData info;
  nav_msgs::OccupancyGrid GTGrid;
  gps_common::GPSFix gps_msg;

  Eigen::Matrix2d rotationMatrix;
  Eigen::Vector2d crd_r, crd_l, crd_rot_r, crd_rot_l;

  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  void getGrid(const nav_msgs::OccupancyGrid &msgGrid);
  std::string FormatPlacemark(double lat1, double lon1);
  void DistanceToCar();
  double DistFrom(double lat1, double lon1);
  double ToRadians(double degrees);
  double interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate);
};

QuantEval::QuantEval() : str_i({ "<coordinates>" }), str_f({ "</coordinates>" })
{
  velocity_sub = nh.subscribe("inspva", 10, &QuantEval::getVelocity, this);
  grid_sub = nh.subscribe("density_pub", 10, &QuantEval::getGrid, this);

  gps_pub = nh.advertise<gps_common::GPSFix>("gps_pub", 1, true);
  gt_pub = nh.advertise<nav_msgs::OccupancyGrid>("gt_pub", 1, true);
}

void QuantEval::StartHandle()
{
  handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);

  char filename[100];

  time_t theTime = time(NULL);
  struct tm *aTime = localtime(&theTime);

  int hour = aTime->tm_hour;
  int min = aTime->tm_min;

  sprintf(filename, "/home/daniela/catkin_ws/src/mastersthesis/eval_api/Results/Path_%dH_%dM.kml", hour, min);
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
}

void QuantEval::getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg)
{
  car_lat = gps_msg.latitude = velMsg->latitude;
  car_lon = gps_msg.longitude = velMsg->longitude;

  // yaw = (90.0 - velMsg->azimuth) * swri_math_util::_deg_2_rad;
  yaw = (0 - velMsg->azimuth) * swri_math_util::_deg_2_rad;

  yaw_1 = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);
  std::cout << yaw_1 << std::endl;
  rotationMatrix << cos(yaw_1), sin(yaw_1), -sin(yaw_1), cos(yaw_1);
}

void QuantEval::getGrid(const nav_msgs::OccupancyGrid &msgGrid)
{
  info = msgGrid.info;
  header = msgGrid.header;

  nc = msgGrid.info.height;
  nl = msgGrid.info.width;
  pace = msgGrid.info.resolution;
}

void QuantEval::LoopFunction()
{
  // std::cout << "latitude: " << std::setprecision(20) << lat << "; longitude: " << lon << std::endl;
  if (car_lon < -7 && car_lon > -9 && car_lat > 39 && car_lat < 41 && pace > 0.1)
  {
    gps_pub.publish(gps_msg);
    handle << FormatPlacemark(car_lat, car_lon);
    gt_points.clear();
    gt_points.resize(N);
    DistanceToCar();
    gt_pub.publish(GTGrid);
  }
}

void QuantEval::ReadKml()
{
  handle_kml_right.open("/home/daniela/catkin_ws/src/mastersthesis/Kml_files/RightPath.kml");
  handle_kml_left.open("/home/daniela/catkin_ws/src/mastersthesis/Kml_files/LeftPath.kml");

  if (!handle_kml_right.is_open() || !handle_kml_left.is_open())
    std::cout << "could not open file" << std::endl;
  else
  {
    std::cout << "opened files" << std::endl;
    strStream_r << handle_kml_right.rdbuf();  // read the file
    file_content_right = strStream_r.str();

    strStream_l << handle_kml_left.rdbuf();  // read the file
    file_content_left = strStream_l.str();
  }

  found_i_r = file_content_right.find(str_i); /*find beginning of coordinates*/
  found_f_r = file_content_right.find(str_f); /*find ending of coordinates*/

  found_i_l = file_content_left.find(str_i); /*find beginning of coordinates*/
  found_f_l = file_content_left.find(str_f); /*find ending of coordinates*/

  handle_kml_right.close(); /*close handle*/
  handle_kml_left.close();
  /*cut string to contemplate only the coordinates*/
  std::string strr = file_content_right.substr(found_i_r + 18, found_f_r - found_i_r - 19);
  std::string strl = file_content_left.substr(found_i_l + 18, found_f_l - found_i_l - 19);
  /*separate in coordinates points*/
  std::stringstream ss_r(strr);
  while (ss_r.good())
  {
    string substr;
    std::getline(ss_r, substr, ' ');
    coordinates_right.push_back(substr);
  }
  std::stringstream ss_l(strl);
  while (ss_l.good())
  {
    string substr;
    std::getline(ss_l, substr, ' ');
    coordinates_left.push_back(substr);
  }

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
  lat_dx_meters.clear();
  lon_dy_meters.clear();
  // gt_points.assign(gt_points.size(), 0);

  GTGrid.info = info;
  GTGrid.header = header;
  // limite direito da estrada
  for (int i = 0; i < coordinates_right.size() - 1; i++)
  {
    dx_r = DistFrom(car_lat, lon_right[i]);  // distance between moving_axis and ground has to be subtracted
    dy_r = DistFrom(lat_right[i], car_lon);
    crd_r << dx_r, dy_r;
    crd_rot_r = rotationMatrix * crd_r;
    dx_rot_r = crd_rot_r(0) - 2.925;
    dy_rot_r = crd_rot_r(1);
    lat_dx_meters.push_back(dx_rot_r);
    lon_dy_meters.push_back(dy_rot_r);
  }

  for (int i = 0; i < coordinates_left.size() - 1; i++)
  {
    dx_l = DistFrom(car_lat, lon_left[i]);  // distance between moving_axis and ground has to be subtracted
    dy_l = -1 * DistFrom(lat_left[i], car_lon);
    crd_l << dx_l, dy_l;
    crd_rot_l = rotationMatrix * crd_l;
    dx_rot_l = crd_rot_l(0) - 2.925;
    dy_rot_l = crd_rot_l(1);
    lat_dx_meters.push_back(dx_rot_l);
    lon_dy_meters.push_back(dy_rot_l);
  }
  // interpolate
  for (int i = 0; i < coordinates_left.size() + coordinates_right.size(); i++)
  {
    if (std::abs(lon_dy_meters[i + 1] - lon_dy_meters[i]) <
        0.1 * std::max(std::abs(lon_dy_meters[i + 1]), std::abs(lon_dy_meters[i])))
    {
      ROS_WARN("true and the nr of points is: %d", (int)floor((lat_dx_meters[i + 1] - lat_dx_meters[i]) / pace));
      for (int n = 1; n < (int)floor((lat_dx_meters[i + 1] - lat_dx_meters[i]) / pace); n++)
      {
        if (lon_dy_meters[i] < 0 && lon_dy_meters[i + 1] < 0)
        {
          std::vector<double> xData = { lat_dx_meters[i], lat_dx_meters[i + 1] };
          std::vector<double> yData = { lon_dy_meters[i], lon_dy_meters[i + 1] };
          double x = lat_dx_meters[i] + pace * n;
          double y = interpolate(xData, yData, x, false);
          lat_dx_meters.push_back(x);
          lon_dy_meters.push_back(y);
        }
        else if (lon_dy_meters[i] > 0 && lon_dy_meters[i + 1] > 0)
        {
          std::vector<double> xData = { lat_dx_meters[i], lat_dx_meters[i + 1] };
          std::vector<double> yData = { lon_dy_meters[i], lon_dy_meters[i + 1] };
          int x = lat_dx_meters[i] + pace * n;
          int y = interpolate(xData, yData, x, false);
          lat_dx_meters.push_back(x);
          lon_dy_meters.push_back(y);
        }
      }
    }
  }

  for (int i = 0; i < lat_dx_meters.size(); i++)
  {
    // std::cout << "inside cycle" << std::endl;
    if (lat_dx_meters[i] <= (double)40 && lat_dx_meters[i] >= (double)0 && lon_dy_meters[i] >= (double)-20 &&
        lon_dy_meters[i] <= (double)20)
    {
      // std::cout << "inside cycle for" << std::endl;
      lin = (int)floor(lat_dx_meters[i] / pace);
      col = (int)floor(lon_dy_meters[i] / pace) + 20 / pace;

      std::cout << "size of x: " << lat_dx_meters.size() << "; size of y: " << lon_dy_meters.size()
                << ". The size of the cycle should be: " << coordinates_left.size() + coordinates_right.size()
                << std::endl;

      if (lin + col * nl < N && lin < nl && col < nc)
      {
        std::cout << "the index of the vector is " << lin + col * nl << std::endl;
        std::cout << "lines: " << lin << "; col: " << col << std::endl;

        gt_points[lin + col * nl] = 100;

        std::cout << "acessed points in occupancy grid " << std::endl;
      }
    }
  }
  GTGrid.data.clear();
  GTGrid.data = gt_points;
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
  reconstruct.ReadKml();

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
}

double QuantEval::DistFrom(double lat1, double lon1)
{
  double earthRadius = 6371000;  // meters
  double dLat = ToRadians(car_lat - lat1);
  double dLon = ToRadians(car_lon - lon1);
  double a =
      sin(dLat / 2) * sin(dLat / 2) + cos(ToRadians(lat1)) * cos(ToRadians(car_lat)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double dist = abs(earthRadius * c);

  return dist;
}

double QuantEval::ToRadians(double degrees)
{
  double radians = degrees * M_PI / 180;
  return radians;
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