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
	  	  //constructor
	  	  //brief: Function to generate the map divided in cells so that X-axis is the front axis, Y-axis is the left_right axis, Z-axis is the vertical axis
	   	  //       Having a map with odd values for every dimension is preferred due to further calculations, so this map has odd sizes
	  	  CellMap(double cell_size, geometry_msgs::Vector3 map_size, geometry_msgs::Vector3 map_center_loc);
	  	  ~CellMap(){};

	  	  //Get functions
	  	  //brief: Function to get single cell size
	  	  double GetCellSize(void);
	  	  //brief: Function to get map size in length unit (ex. [m])
	  	  geometry_msgs::Vector3 GetMapSize(void);
	  	  //brief: Function to get the number of cell for every dimension
	  	  geometry_msgs::Vector3 GetMapSize4Cell(void);
	  	  //brief: Function to get map center location respect to the absolute reference
	  	  geometry_msgs::Vector3 GetMapcenterLocation(void);
	  	  //brief: Function to get the cell map
	  	  std::vector<std::vector<std::vector<double>>> GetDoubleCellMap(void);

	  	  //Other functions
	  	  //brief: Function to saturate the values of a mapCell matrix
	  	  void Saturator(double saturation_val);
	  	  //brief: Function to normalize the values of a mapCell matrix on [0;normal_value]
	  	  void Normalization(double normal_value = 100.0);

	  	  //brief: Function to convert a space point into a matrix point data
	  	  std::vector<int> PointXYZtoIJK(geometry_msgs::Vector3 XYZ_location);



  	  private:
	  	  double cell_size_;
	  	  geometry_msgs::Vector3 map_size_;
	  	  geometry_msgs::Vector3 map_center_location_;
	  	  std::vector<std::vector<std::vector<double>>> cell_map_;

	  	  //Set functions (input validation included)
	  	  void SetCellSize(double cell_size);
	  	  void SetMapSize(geometry_msgs::Vector3 map_size);
	  	  void SetMapcenterLocation(geometry_msgs::Vector3 map_center_loc);
	  	  bool SetDoubleCellMap(std::vector<std::vector<std::vector<double>>> double_cell_map);
  };


// End namespace "pot_field_functions"
}


#endif /* POTENTIAL_FIELD_PLANNER_LIB_POT_FIELD_H */


