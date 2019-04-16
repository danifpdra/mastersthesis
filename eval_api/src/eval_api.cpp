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
#include "osmgpsmap-1.0/osm-gps-map.h"

#include <unistd.h>                      //Sleep
#include <eval_api/GoogleEarthPath.hpp>  //This class

using namespace std;

class QuantEval
{
public:
  QuantEval();
  void LoopFunction();
  void GtkLaunch();
  void CloseHandle();
  void StartHandle();

private:
  ros::NodeHandle nh;

  // void FinishCallback(GtkWidget *windows, gpointer data);
  // void NextCallback(GtkWidget *windows, gpointer data);
  // void BackCallback(GtkWidget *windows, gpointer data);

  static void FinishCallback(GtkWidget *windows, gpointer *data)
  {
    gtk_main_quit();
    // main::quit();
    // cv::imwrite("~/catkin_ws/src/mastersthesis/eval_api/Results/image_XXX.jpg", gray_image);
  }

  static void NextCallback(GtkWidget *windows, gpointer *data)
  {
  }

  static void BackCallback(GtkWidget *windows, gpointer *data)
  {
  }

  float lat, lon;
  GtkWidget *widget;
  OsmGpsMap *map;
  GtkBuilder *builderG;
  char *gladeFile = (char *)"/home/daniela/catkin_ws/src/mastersthesis/eval_api/src/eval_api.glade";
  int ret;
  ros::Subscriber velocity_sub;
  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  std::string FormatPlacemark(float lat1, float lon1);

  /***********************************************/
  // write kml file
  std::ofstream handle;
  /*********************************************/
};

void QuantEval::StartHandle()
{
  handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);

  char filename[100];
  int writeCount = std::rand();
  sprintf(filename, "/home/daniela/catkin_ws/src/mastersthesis/eval_api/Results/Sample_%d.kml", writeCount);
  // Open the KML file for writing:
  handle.open(filename);
  // Write to the KML file:
  handle << "<?xml version='1.0' encoding='utf-8'?>\n";
  handle << "<kml xmlns='http://www.opengis.net/kml/2.2'>\n";
  handle << "<Placemark>\n";
  handle << "<description>This is the path between the 2 points</description>\n";
  handle << "<styleUrl>#pathstyle</styleUrl>\n";
  handle << "<LineString>\n";
  handle << "<tessellate>1</tessellate>\n";
  handle << "<coordinates>";
}

void QuantEval::getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg)
{
  lat = velMsg->latitude;
  lon = velMsg->longitude;
}

QuantEval::QuantEval()
{
  velocity_sub = nh.subscribe("inspva", 10, &QuantEval::getVelocity, this);
}

void QuantEval::LoopFunction()
{
  std::cout << "latitude: " << std::setprecision(20) << lat << "; longitude: " << lon << std::endl;
  handle << FormatPlacemark(lat, lon);
}

void QuantEval::GtkLaunch()
{
  builderG = gtk_builder_new();
  ret = gtk_builder_add_from_file(builderG, gladeFile, NULL);

  if (!ret)
  {
    std::cout << gladeFile << " file was not found. Aborting!" << std::endl;
  }

  std::cout << "here" << std::endl;

  gtk_builder_connect_signals(builderG, NULL);

  GtkWidget *win = GTK_WIDGET(gtk_builder_get_object(builderG, "window1"));

  GtkButton *back_button = GTK_BUTTON(gtk_builder_get_object(builderG, "back_button"));
  GtkButton *next_button = GTK_BUTTON(gtk_builder_get_object(builderG, "next_button"));
  GtkButton *ok_button = GTK_BUTTON(gtk_builder_get_object(builderG, "ok_button"));

  GtkSpinButton *distance_spin = GTK_SPIN_BUTTON(gtk_builder_get_object(builderG, "distance_spin"));
  GtkSpinButton *resolution_spin = GTK_SPIN_BUTTON(gtk_builder_get_object(builderG, "resolution_spin"));

  GtkComboBoxText *filter_combo = GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builderG, "filter_combo"));

  GtkDrawingArea *drawingarea = GTK_DRAWING_AREA(gtk_builder_get_object(builderG, "map_image"));
  GtkImage *filter_image = GTK_IMAGE(gtk_builder_get_object(builderG, "filter_image"));
  GtkImage *camera_image = GTK_IMAGE(gtk_builder_get_object(builderG, "camera_image"));

  g_signal_connect(G_OBJECT(back_button), "clicked", G_CALLBACK(BackCallback), this);
  g_signal_connect(G_OBJECT(next_button), "clicked", G_CALLBACK(NextCallback), this);
  g_signal_connect(G_OBJECT(ok_button), "clicked", G_CALLBACK(FinishCallback), this);
}

void QuantEval::CloseHandle()
{
  handle << "</coordinates>\n";
  handle << "</LineString>\n";
  handle << "</Placemark>\n";
  handle << "</kml>\n";
  handle.close();
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
  srand(time(NULL));
  ros::init(argc, argv, "QuantEval");
  gtk_init(&argc, &argv);
  QuantEval reconstruct;
  reconstruct.StartHandle();
  // reconstruct.GtkLaunch();
  // gtk_main();

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
  if (lon < -7 && lon > -9 && lat > 39 && lat < 41)
    handle << " " << std::setprecision(20) << lon1 << "," << lat1 << ",0";
  return ss.str();
}
