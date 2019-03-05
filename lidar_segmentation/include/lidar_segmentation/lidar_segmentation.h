/**
\file  lidar_segmentation.h 
\brief Global include file
\author Daniel Coimbra
*/

#ifndef _COIMBRA_LIDAR_SEGMENTATION_H_
#define _COIMBRA_LIDAR_SEGMENTATION_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <rosbag/bag.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <iostream>
#include <stdlib.h>
#include "colormap/colormap.h"
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <ros/package.h>
#include "clustering.h"
#include "groundtruth.h"


# define SIMPLE_SEG 1;      
# define PREM_SEG 2;
# define DIET_SEG 3;
# define ABD_SEG 4;
# define NN_SEG 5;
# define SANTOS_C_SEG 6;
# define SANTOS_B_SEG 7;

using namespace std;


/**
 * \class Point
 * Class with a corresponding x y theta range label and iteration of a laser point read from the txt file 
 * \date April 2013
 * \author Jorge Almeida 
 * 
 */

class Point
{
	public:
		
		Point()
		{
			x=0;
			y=0;
			theta=0;
			range=0;
			label=0;
			iteration=0;
		}
		
		double x;             /**< x coordinate */
		double y;			   /**< y coordinate */		
		double theta;		   /**< point's angle value */
		double range;		   /**< point's range value */	
		int label;			   /**< point's position in the incoming Laser Scan */
		int cluster_id;		   /**< id of the GT cluster which point is assigned to */	
		int iteration;        /**<  iteration which the point belongs */ 
};

//Shared pointer to the Point class
typedef boost::shared_ptr<Point> PointPtr;
typedef boost::shared_ptr< ::sensor_msgs::LaserScan> LaserScanPtr;


/**
@brief Comparison function 
@param p1 first PointPtr input  
@param p2 second PointPtr input
@return the input with belonging to the lowest iteration and lowest label, else return false
*/

bool comparePoints(PointPtr p1,PointPtr p2);

/**
@brief Convert the all Point class from a scan to a Cluster class 
@param scan_msg  incoming LaserScan 
@param points incoming Laser Points
@param clusters_GT output vector of clusters, these clusters use the Cluster class
@return Number of clusters on the groundtruth
*/

int convertPointsToCluster( const sensor_msgs::LaserScan::ConstPtr& scan_msg , vector<PointPtr>& points, vector<ClusterPtr>& clusters_GT);

/**
@brief Creates a Point class from the file with the Ground-truth points
@param points incoming Laser Points
@param data_gts output x, y and labels from one iteration, it uses the C_DataFromFile class   
@return return 0
*/

int createPointsFromFile( vector<PointPtr>& points , C_DataFromFilePtr data_gt );


/**
@brief Function that outputs a set of points with range between min_range and max_range values 
@param points_in incoming Laser Points
@param points_out points with range between the minimum and maximum value
@param min_range minimum range for a point to be considered valid
@param max_range maximum range for a point to be considered valid
@return void 
*/

void filterPoints(vector<PointPtr>& points_in , vector<PointPtr>& points_out, double min_range, double max_range);


/**
@brief Removes points belonging to clusters with less than min_points
@param laser_points incoming Laser Points
@param clusters clusters input vector of clusters, these clusters use the Cluster class
@param min_points clusters with points less than this value are removed
@return void 
*/
void removeInvalidPoints(vector<PointPtr>& laser_points, vector<ClusterPtr> clusters, uint min_points );


/**
@brief Compares the point id to a cluster_id, if true that point is erased
@param p point to be compared
@param cluster_id id of the invalid cluster
@return 1 if point id = cluster id
*/

bool correctClusterId (PointPtr p, int cluster_id);


/**
@brief Wirtes properties of the Ground-truth segments into a text file
@param iteration iteration of the Laser Scan
@param points incoming Laser Points
@param id_result if id_result == 1 writes segments' boundaries; else writes segments' centers
@return return 0
*/

int writeResults_GT(uint iteration , vector<PointPtr>& points , int id_result);


/**
@brief Wirtes properties of the Algorithms segments into a text file
@param points incoming Laser Points
@param algorithm_id case 1 - Simple Segmentation; 
					case 2 - Multivariable Segmentation; 
					case 3 - Dietmayer Segmentation;		
					case 4 - Adaptive Breakpoint Detector;
					case 5 - Spatital Nearest Neigbour;
					case 6 - Santos Approach C0 variation;
					case 7 - Santos Approach Beta variation;
@param initial_value Initial value of the threshold parameter;
@param increment Increment of the threshold parameter;
@param number_of_iterations Number of threshold variations
@param iteration iteration of the Lasr Scan
@param id_result if id_result == 1 writes segments' boundaries; else writes segments' centers
@return return 0
*/

int writeResults_paths(vector<PointPtr>& points, int algorithm_id, double initial_value, double increment, int number_of_iterations , uint iteration, int id_result);


#endif
