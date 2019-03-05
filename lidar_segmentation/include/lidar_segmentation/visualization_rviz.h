/**
\file  visualization_rviz.h 
\brief Visualization on rviz related functions header 
\author Daniel Coimbra
*/

#ifndef _COIMBRA_VISUALIZATION_RVIZ_H_
#define _COIMBRA_VISUALIZATION_RVIZ_H_

#include <vector>

/**
 * \class Markers
 * Class to handle the visualization markers 
 * \author Jorge Almeida in mtt
 * 
 */

class Markers
{
	public:		
		void update(visualization_msgs::Marker& marker)
		{
			for(uint i=0;i<markers.size();++i)
				if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
				{
					markers.erase(markers.begin()+i);
					break;
				}
				
			markers.push_back(marker);
		}
		
		void decrement(void)
		{
			for(uint i=0;i<markers.size();++i)
			{
				switch(markers[i].action)
				{
					case visualization_msgs::Marker::ADD:
						markers[i].action = visualization_msgs::Marker::DELETE;
						break;
					case visualization_msgs::Marker::DELETE:
						markers[i].action = -1;
						break;
				}
			}
		}
		
		void clean(void)
		{
			vector<visualization_msgs::Marker> new_markers;

			for(uint i=0;i<markers.size();++i)
				if(markers[i].action!=-1)
					new_markers.push_back(markers[i]);
				
			markers=new_markers;
		}

		vector<visualization_msgs::Marker> getOutgoingMarkers(void)
		{
			vector<visualization_msgs::Marker> m_out(markers);
			return markers;
		}
		
	private:
		
		vector<visualization_msgs::Marker> markers;
};


/**
@brief Create marker form clusters
 * This function creates a set of markers for the current clusters from the sereral segmentation algorithms
 *this uses the Markers class to keep a up to date list of the current markers. 
@param clusters Cluster class clusters from the simpleClustering function
@param clusters_Premebida Cluster class clusters from the premebidaClustering function
@param clusters_Dietmayer Cluster class clusters from the dietmayerClustering function
@param clusters_Santos Cluster class clusters from the santosClustering function
@param clusters_ABD Cluster class clusters from the abdClustering function
@param clusters_nn Cluster class clusters from the nnClustering function
@param clusters_GT Cluster class clusters from the convertPointsToCluster function
@return markers to be published
@author Jorge Almeida inspired - createTargetMarkers in mtt
*/

vector<visualization_msgs::Marker> createTargetMarkers( vector<ClusterPtr>& clusters , vector<ClusterPtr>& clusters_Premebida ,  vector<ClusterPtr>& clusters_Dietmayer, vector<ClusterPtr>& clusters_Santos, vector<ClusterPtr>& clusters_ABD ,vector<ClusterPtr>& clusters_nn , vector<ClusterPtr>&  clusters_GT  );

#endif


