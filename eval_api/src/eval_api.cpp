#include <tf/transform_listener.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
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

class QuantEval
{
public:
  QuantEval();
  void LoopFunction();

private:
  void bagManipule();
};

bag.close();

QuantEval::QuantEval()
{
}

void bagManipule()
{
  rosbag::Bag bag;
  bag.open("~/catkin_ws/src/mastersthesis/bags/voltaUA.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH (rosbag::MessageInstance const m, view)
  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
      std::cout << s->data << std::endl;

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
      std::cout << i->data << std::endl;
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
