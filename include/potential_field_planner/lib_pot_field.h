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

#define exit_time					5.5		//[s]
#define std_sleep_time				0.01	//[s]
#define std_wave_step				0.05	//arbitrary value to define how to progress while the class is generating the map

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
	  	  std::vector<int> GetMapSize4Cell(void);
	  	  //brief: Function to get map center location respect to the absolute reference
	  	  geometry_msgs::Vector3 GetMapcenterLocation(void);
	  	  //brief: Function to get the boolean cell map [occupancy map]
	  	  std::vector<std::vector<std::vector<bool>>> GetBoolCellMap(void);
	 	  //brief: Function to get the double cell map general [potential field]
	  	  std::vector<std::vector<std::vector<double>>> GetDoubleCellMapRep(void);
	 	  //brief: Function to get the double cell map general [potential field]
	  	  std::vector<std::vector<std::vector<double>>> GetDoubleCellMapAtt(void);
	 	  //brief: Function to get the double cell map general [potential field]
	  	  std::vector<std::vector<std::vector<double>>> GetDoubleCellMapGen(void);

	  	  //Other functions
	  	  //brief: Function to saturate the values of the general CellMap
	  	  void SaturatorGenMap(double saturation_val);
	  	  //brief: Function to normalize the values of the general CellMap on [0;normal_value]
	  	  void NormalizationGenMap(double normal_value = 100.0);

	  	  //brief: Function to convert a space point into a matrix point data
	  	  std::vector<int> PointXYZtoIJK(geometry_msgs::Vector3 XYZ_location);
	  	  //brief: Function to convert a matrix point into a space point data
	  	  geometry_msgs::Vector3 PointIJKtoXYZ(std::vector<int> IJK_location);

	  	  //brief: Function to find every cell[IJK] having the value EQUAL (=) to boolean_val [occupancy map]
	  	  std::vector<std::vector<int>> GetCellsIJK_BoolOccupancy(bool boolean_val = 1);
	  	  //brief: Function to find every point[XYZ] having the value EQUAL (=) to boolean_val [occupancy map]
	  	  std::vector<geometry_msgs::Vector3> GetCellsXYZ_BoolOccupancy(bool boolean_val = 1);
	  	  //brief: Function to find every cell[IJK] having the value EQUAL (=), DIFFERENT (!), strictly MINOR(<), or strictly MAJOR(>) than a comparison value [potential field]
	  	  std::vector<std::vector<int>> GetCellsIJK_DoubleComparison(char comparison_symbol, double comparison_val);
	  	  //brief: Function to find every point[XYZ] having the value EQUAL (=), DIFFERENT (!), strictly MINOR(<), or strictly MAJOR(>) than a comparison value [potential field]
	  	  std::vector<geometry_msgs::Vector3> GetCellsXYZ_DoubleComparison(char comparison_symbol, double comparison_val);

	  	  //brief: Function to insert objects in the boolean map [occupancy map]
	  	  bool ObjectInsertion(std::string obj_kind, geometry_msgs::Vector3 obj_position, geometry_msgs::Vector3 obj_orientation, std::vector<double> obj_valume_sizes);

	  	  //brief: Function to generate the repulsive field map; repulsive_radius is how far do the user want to make feel the repulsive field
	  	  bool RepulsiveMap_generator(double repulsive_radius, double infinite_val);
	  	  //brief: Function to generate the attractive field map [based on the classical approach]
	  	  bool AttractiveMapCl_generator(geometry_msgs::Vector3 goal_position, double wave_step = std_wave_step);
	  	  //brief: Function to generate the attractive field map [based on the wave-front approach]
	  	  bool AttractiveMapWF_generator(geometry_msgs::Vector3 goal_position, double wave_step = std_wave_step);
	  	  //brief: Function to generate the global potential field map; att_wave_app = true means using wave-front approach | = false means classic approach
	  	  bool PotentialMap_generator(double rep_radius, geometry_msgs::Vector3 goal_position, double wave_step = std_wave_step, bool att_wave_app = true);

	  	  //brief: Function to extract the path point in IJK matrix coordinates
	  	  std::vector<std::vector<int>> pathExtractor_IJK(geometry_msgs::Vector3 initial_position);
	  	  //brief: Function to extract the path point in XYZ matrix coordinates
	  	  std::vector<geometry_msgs::Vector3> pathExtractor_XYZ(geometry_msgs::Vector3 initial_position);

  	  private:
	  	  double cell_size_;
	  	  geometry_msgs::Vector3 map_size_;
	  	  geometry_msgs::Vector3 map_center_location_;
	  	  std::vector<std::vector<std::vector<bool>>> bool_cell_map_;
	  	  std::vector<std::vector<std::vector<double>>> double_cell_map_rep_;  //repulsive map
	  	  std::vector<std::vector<std::vector<double>>> double_cell_map_att_;  //attractive map
	  	  std::vector<std::vector<std::vector<double>>> double_cell_map_gen_;  //general map (attractive + repulsive)

	  	  //Set functions (input validation included)
	  	  void SetCellSize(double cell_size);
	  	  void SetMapSize(geometry_msgs::Vector3 map_size);
	  	  void SetMapcenterLocation(geometry_msgs::Vector3 map_center_loc);
	  	  bool SetBoolCellMap(std::vector<std::vector<std::vector<bool>>> bool_cell_map);
	  	  bool SetDoubleCellMapRep(std::vector<std::vector<std::vector<double>>> double_cell_map);
	  	  bool SetDoubleCellMapAtt(std::vector<std::vector<std::vector<double>>> double_cell_map);
	  	  bool SetDoubleCellMapGen(std::vector<std::vector<std::vector<double>>> double_cell_map);
  };


// End namespace "pot_field_functions"
}


#endif /* POTENTIAL_FIELD_PLANNER_LIB_POT_FIELD_H */


