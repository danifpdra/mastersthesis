/**
\file  clustering.h 
\brief Clustering related functions header.
\author Daniel Coimbra
*/

#ifndef _COIMBRA_CLUSTERING_H_
#define _COIMBRA_CLUSTERING_H_

#include <vector>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

class Point;
typedef boost::shared_ptr<Point> PointPtr;


/**
 * \class Cluster
 * Class to accumulate clusters 
 * \date April 2013
 * \author Daniel Coimbra
 * 
 */

class Cluster
{
public:
	int id;						        /**< cluster id */
	
	PointPtr centroid;     				/**< cluster centroid [xc yc] */
	
	PointPtr central_point; 	        /**< cluster central point [xm ym] */
	
	vector<double> ranges;             /**< scan values distances [m] */   
	
	vector<PointPtr> support_points; 	/**< Vector of the support points of the cluster */
};


//Shared pointer to the Cluster class
typedef boost::shared_ptr<Cluster> ClusterPtr;

/* Clustering functions */

/**
@brief Performs a Simple Segmentation operation
@param points incoming Laser Points
@param threshold distance value used to break clusters
@param clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the simple segmentation algorithm
*/

int simpleClustering(vector<PointPtr>& points, double threshold, vector<ClusterPtr>& clusters);

/**
@brief Performs Segmentation operation with the Dietmayer Segmentation Algorithm 
@param points incoming Laser Points
@param C0 parameter used for noise reduction 
@param clusters_Dietmayer output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Dietmayer Segmentation Algorithm
*/

int dietmayerClustering( vector<PointPtr>& points, double C0 ,vector<ClusterPtr>& clusters_Dietmayer);

/**
@brief Performs Segmentation operation with the Multivariable Segmentation Algorithm
@param points incoming Laser Points
@param threshold_prem cosine distance value used to break clusters
@param clusters_Premebida clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Multivariable Segmentation Algorithm
*/

int premebidaClustering( vector<PointPtr>& points, double threshold_prem , vector<ClusterPtr>& clusters_Premebida);

/**
@brief Performs Segmentation operation with the Adaptative Breakpoint Detector
@param points incoming Laser Points
@param lamda auxiliary parameter
@param clusters_ABD clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Adaptative Breakpoint Detector
*/

int abdClustering( vector<PointPtr>& points , double lambda ,vector<ClusterPtr>& clusters_ABD);

/**
@brief Performs Segmentation operation with the Spacial Nerarest Neighbor Algorithm 
@param points incoming Laser Points
@param threshold distance value used to break clusters
@param clusters_nn clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Spacial Nerarest Neighbor Algorithm
*/

int nnClustering( vector<PointPtr>& points, double threshold , vector<ClusterPtr>& clusters_nn);

/**
@brief Performs Segmentation operation with the Santos Approach from the Dietmayer Segmentation Algorithm 
@param points incoming Laser Points
@param C0 parameter used for noise reduction
@param beta parameter aiming to reduce the dependence of the segmentation with respect to the distance between the LRF and the objects
@param clusters_Santos clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Santos Approach from the Dietmayer Segmentation Algorithm
*/

int santosClustering( vector<PointPtr>& points, double C0, double beta, vector<ClusterPtr>& clusters_Santos);

/**
@brief A auxiliary function of the premebidaClustering function
 *Calculates the a set of atributes of a pair of laser points 
@param range1 range value of the the first point 
@param range2 range value of the the second point
@param range1cart cartesian value of the first point
@param range2cart cartesian value of the second point
@return Vector of atributes
*/

vector<double>  rangeFeatures( double range1, double range2, PointPtr range1cart, PointPtr range2cart);

/**
@brief Calculates the cosine distance between 2 vectors   
@param vect1 input vector with the range atributes from the fist pair
@param vect2 input vector with the range atributes from the second pair
@return Cosine distance 
*/

double cosineDistance(vector<double>& vect1, vector<double>& vect2);

/**
@brief Converts degree values to radian values  
@param angle angle value in degrees
@return angle value in radians
*/

double deg2rad(double angle);


/**
@brief Calculates the cluster's centroid   
@param support_points cluster's support points 
@return The coordinates of the cluster's centroid 
*/

PointPtr calculateClusterCentroid( vector<PointPtr> support_points );

/**
@brief Calculates the cluster's central point   
@param support_points cluster's support points
@return The coordinates of the cluster's central point
*/

PointPtr calculateClusterMedian(vector<PointPtr> support_points);

#endif