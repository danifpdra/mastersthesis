
#include <tf/transform_listener.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
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
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <cairo/cairo.h>
// #include <gtkmm.h>
#include <gdk/gdk.h>
#include <gtk/gtk.h>

#include <gdk/gdkkeysyms.h>
#include <glib.h>
// #include "osmgpsmap-1.0/osm-gps-map.h"

#include <unistd.h>                      //Sleep
#include <eval_api/GoogleEarthPath.hpp>  //This class

// #include "kml/base/file.h"
// #include "kml/base/string_util.h"
// #include "kml/base/zip_file.h"
// #include "kml/dom.h"

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

  float car_lat, car_lon, dx, dy, pace;
  int lin, col, nl, nc, N;

  std::ofstream handle;
  std::ifstream handle_kml_right, handle_kml_left;
  ros::Subscriber velocity_sub, grid_sub;
  ros::Publisher gps_pub, gt_pub;
  gps_common::GPSFix gps_msg;
  // to read kml
  std::string file_content_right, file_content_left;
  std::string str_i, str_f;
  std::size_t found_i_r, found_f_r, found_i_l, found_f_l;
  std::stringstream strStream_r, strStream_l;
  std::vector<string> coordinates_right, coordinates_left;
  std::vector<float> lat_right, lat_left, lon_right, lon_left, lat_dx_meters, lon_dy_meters;
  std::vector<int8_t> gt_points;

  std_msgs::Header header;
  nav_msgs::MapMetaData info;
  nav_msgs::OccupancyGrid GTGrid;

  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  void getGrid(const nav_msgs::OccupancyGrid &msgGrid);
  std::string FormatPlacemark(float lat1, float lon1);
  void DistanceToCar();
  float DistFrom(float lat1, float lon1);
  float ToRadians(float degrees);
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
  car_lat = velMsg->latitude;
  car_lon = velMsg->longitude;
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
  gps_msg.latitude = car_lat;
  gps_msg.longitude = car_lon;

  // std::cout << "latitude: " << std::setprecision(20) << lat << "; longitude: " << lon << std::endl;
  if (car_lon < -7 && car_lon > -9 && car_lat > 39 && car_lat < 41)
    gps_pub.publish(gps_msg);
  handle << FormatPlacemark(car_lat, car_lon);

  DistanceToCar();
  gt_pub.publish(GTGrid);
}

void QuantEval::ReadKml()
{
  handle_kml_right.open("/home/daniela/RightPath.kml");
  handle_kml_left.open("/home/daniela/LeftPath.kml");

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

  // std::cout << found_i << " " << found_f << std::endl;
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
  int ncoord = 1;
  for (auto const &point : coordinates_right)
  {
    // std::cout << point << std::endl;
    int count = 1;
    std::stringstream sss(point);
    while (sss.good())
    {
      string string;
      std::getline(sss, string, ',');
      // std::cout << string << " and " << count << std::endl;
      if (count == 1 && !string.empty() && ncoord < coordinates_right.size())
        lon_right.push_back(stof(string));
      if (count == 2 && !string.empty() && ncoord < coordinates_right.size())
        lat_right.push_back(stof(string));
      count++;
    }
    ncoord++;
  }
  ncoord = 1;
  for (auto const &point : coordinates_left)
  {
    int count = 1;
    std::stringstream sss(point);
    while (sss.good())
    {
      string string;
      std::getline(sss, string, ',');
      if (count == 1 && !string.empty() && ncoord < coordinates_left.size())
        lon_left.push_back(stof(string));
      if (count == 2 && !string.empty() && ncoord < coordinates_left.size())
        lat_left.push_back(stof(string));
      count++;
    }
    ncoord++;
  }
}

void QuantEval::DistanceToCar()
{
  N = nc * nl;
  gt_points.clear();
  gt_points.resize(N);
  gt_points.assign(gt_points.size(), 0);

  GTGrid.info = info;
  GTGrid.header = header;
  // limite direito da estrada
  for (int i = 0; i < coordinates_right.size(); i++)
  {
    dx = DistFrom(car_lat, lon_right[i]) - 2.925;  // distance between moving_axis and ground has to be subtracted
    dy = DistFrom(lat_right[i], car_lon);
    lat_dx_meters.push_back(dx);
    lon_dy_meters.push_back(dy);
  }

  for (int i = 0; i < coordinates_left.size(); i++)
  {
    dx = DistFrom(car_lat, lon_left[i]) - 2.925;  // distance between moving_axis and ground has to be subtracted
    dy = -1 * DistFrom(lat_left[i], car_lon);
    lat_dx_meters.push_back(dx);
    lon_dy_meters.push_back(dy);

    std::cout << "dx: " << dx << "; dy: " << dy << std::endl;
  }

  for (int i = 0; i < lat_dx_meters.size(); i++)
  {
    if (lat_dx_meters[i] <= 40 && lat_dx_meters[i] >= 0 && lon_dy_meters[i] >= -20 && lon_dy_meters[i] <= 20)
    {
      lin = (int)floor(lat_dx_meters[i] / pace);
      col = (int)floor(lon_dy_meters[i] / pace) + 20 / pace;

      gt_points[lin + col * nl] = 100;
    }
  }

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

std::string QuantEval::FormatPlacemark(float lat1, float lon1)
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

float QuantEval::DistFrom(float lat1, float lon1)
{
  float earthRadius = 6371000;  // meters
  float dLat = ToRadians(car_lat - lat1);
  float dLon = ToRadians(car_lon - lon1);
  float a =
      sin(dLat / 2) * sin(dLat / 2) + cos(ToRadians(lat1)) * cos(ToRadians(car_lat)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float dist = earthRadius * c;

  return dist;
}

float QuantEval::ToRadians(float degrees)
{
  float radians = degrees * M_PI / 180;
  return radians;
}
