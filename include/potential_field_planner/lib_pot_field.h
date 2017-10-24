/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some functions useful to accomplish the "path Planning" final project based on Baxter.
         Amazon Robotics Challenge - Universidad Jaume I (Spain) Competitor
*/

#ifndef POTENTIAL_FIELD_PLANNER_LIB_POT_FIELD_H
#define POTENTIAL_FIELD_PLANNER_LIB_POT_FIELD_H

// ROS
#include <ros/ros.h>

// C++
#include <stdio.h>
#include <fstream>
#include <std_msgs/Bool.h>
#include <boost/filesystem.hpp>

// Sansebastiano's pkg
#include <side_pkg/side_func.h>

//side_func.h's libs
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/math/constants/constants.hpp>
////#include <geometry_msgs/PointStamped.h>
#include <tf_conversions/tf_kdl.h>

//////////////////////////////////////////////////////////////////////////////////////
// VALUES MODIFIABLE BY THE USER \\

#define exit_time					5.5
#define std_sleep_time				0.02

//////////////////////////////////////////////////////////////////////////////////////


namespace pot_field_functions
{
  class CellMap
  {
  	  public:
	  	  CellMap(double depth_x, double width_y, double height_z, double cell_size, geometry_msgs::Vector3 dist_centres);
	  	  ~CellMap()
	  	  {};

  	  private:
	  	std::vector< std::vector< std::vector< double > > > map_values_;
  };


// End namespace "pot_field_functions"
}


#endif /* POTENTIAL_FIELD_PLANNER_LIB_POT_FIELD_H */


