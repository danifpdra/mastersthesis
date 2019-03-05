/**
\file  groundtruth.h 
\brief Groundtruth related functions header
\author Daniel Coimbra
*/

#ifndef _COIMBRA_GROUNDTRUTH_H_
#define _COIMBRA_GROUNDTRUTH_H_

#include <vector>


/**
 * \class C_DataFromFile
 * Class to accumulate the data from the .txt file 
 * \date April 2013
 * \author Daniel Coimbra
 * 
 */

class C_DataFromFile
{

public:	
	
	int iteration; /**< Number of the iteration */
	
	vector<double> x_valuesf; /**< Vector with all the the xx values of one iteration */
	
	vector<double> y_valuesf; /**< Vector with all the the yy values of one iteration */
	
	vector<int> labels;			/**< Vector with all the the labels of one iteration */
};

//Shared pointer to the C_DataFromFile class
typedef boost::shared_ptr<C_DataFromFile> C_DataFromFilePtr;


/**
@brief Reads from a file the x, y and labels values from all the laser points from one iteration
@param data_gts output x, y and labels from one iteration, it uses the C_DataFromFile class 
@param values_per_scan Number of values per scan
@return The x, y and labels values from all the laser points from one iteration
*/

int readDataFile( vector<C_DataFromFilePtr>&  data_gts , int values_per_scan);


/**
@brief Performs Segmentation operation with the Adaptative Breakpoint Detector
@param points incoming Laser Points
@param clusters_GT clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Adaptative Breakpoint Detector
*/

int convertPointsToCluster(vector<PointPtr>& points, vector<ClusterPtr>& clusters_GT);

#endif
