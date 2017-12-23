/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some functions useful to accomplish the "path Planning" final project based on Baxter.
         Amazon Robotics Challenge - Universidad Jaume I (Spain) [Competitor]
*/

// this pkg
#include <potential_field_planner/lib_pot_field.h>

namespace bsc = basic_side_classes;
namespace bsf = basic_side_functions;
namespace gsf = geometry_side_functions;

namespace pff = pot_field_functions;

  ////Public section////
  //constructor
  pff::CellMap::CellMap(double cell_size, geometry_msgs::Vector3 map_size, geometry_msgs::Vector3 map_center_loc)
  {
	  int temp_dim[3];
	  double temp_double[3];
	  std::vector<double> temp_vector;
	  std::vector<std::vector<std::vector<double>>> map;

	  //input validation and variable setting
	  SetCellSize(cell_size);
	  SetMapSize(map_size);

	  temp_double[0] = map_size_.x/cell_size_;
	  temp_double[1] = map_size_.y/cell_size_;
	  temp_double[2] = map_size_.z/cell_size_;

	  //to guarantee odd sizes and the finest approximation to the edges of the map
	  for(int i = 0; i < 3; i++)
	  {
		  if((int)temp_double[i] % 2 != 0)
			  temp_dim[i] = (int)temp_double[i];
		  else
			  temp_dim[i] = (int)temp_double[i] +1;

	  }
	  //map_size changes if cell are not odd in every row and column
	  map_size.x = temp_dim[0]*cell_size_;
	  map_size.y = temp_dim[1]*cell_size_;
	  map_size.z = temp_dim[2]*cell_size_;
	  //variable setting
	  SetMapSize(map_size);

	  //map generation
	  map.resize(temp_dim[0]);
	  for(int i = 0; i < temp_dim[0]; i++)
	  {
		  map[i].resize(temp_dim[1]);
		  for(int j = 0; j < temp_dim[1]; j++)
		  {
			  map[i][j].resize(temp_dim[2]);
		  }
	  }

	  //variable setting
	  SetMapcenterLocation(map_center_loc);
	  SetDoubleCellMap(map);
  }

  //Get functions
  double pff::CellMap::GetCellSize()
  {
	  return cell_size_;
  }
  geometry_msgs::Vector3 pff::CellMap::GetMapSize()
  {
	  return map_size_;
  }
  geometry_msgs::Vector3 pff::CellMap::GetMapSize4Cell()
  {
	  geometry_msgs::Vector3 map_size4cell;
	  map_size4cell.x = map_size_.x/cell_size_;
	  map_size4cell.y = map_size_.y/cell_size_;
	  map_size4cell.z = map_size_.z/cell_size_;

	  return map_size4cell;
  }
  geometry_msgs::Vector3 pff::CellMap::GetMapcenterLocation()
  {
	  return map_center_location_;
  }
  std::vector<std::vector<std::vector<double>>> pff::CellMap::GetDoubleCellMap()
  {
	  return cell_map_;
  }

  //Other functions
  void pff::CellMap::Saturator(double saturation_val)
  {
	  for(int i = 0; i < cell_map_.size(); i++)
	  {
		  for(int j = 0; j < cell_map_[0].size(); j++)
		  {
			  for(int z = 0; z < cell_map_[0][0].size(); z++)
			  {
				  if(cell_map_[i][j][z] > saturation_val)
					  cell_map_[i][j][z] = saturation_val;
			  }
		  }
	  }
  }
  void pff::CellMap::Normalization(double normal_value)
  {

	  double temp_double;

 	  //research of the higher value in the map
 	  temp_double = 0.0;
 	  for(int i = 0; i < cell_map_.size(); i++)
 	  {
 		  for(int j = 0; j < cell_map_[0].size(); j++)
 		  {
 			  for(int z = 0; z < cell_map_[0][0].size(); z++)
 			  {
 				  if(cell_map_[i][j][z] > temp_double)
 				  {
 					  temp_double = cell_map_[i][j][z];
 				  }
 			  }
 		  }
 	  }

 	  //normalizator
 	  temp_double = normal_value/temp_double;

	  //normalization
 	  for(int i = 0; i < cell_map_.size(); i++)
 	  {
 		  for(int j = 0; j < cell_map_[0].size(); j++)
 		  {
 			  for(int z = 0; z < cell_map_[0][0].size(); z++)
 			  {
 				 cell_map_[i][j][z] *= temp_double;
 			  }
 		  }
 	  }

  }

  std::vector<int> pff::CellMap::PointXYZtoIJK(geometry_msgs::Vector3 XYZ_location)
  {
	  std::vector<int> IJK2return; IJK2return.resize(3);
	  std::vector<std::vector<double>> trans_abs_point_matrix, trans_abs_center_matrix, trans_center_point_matrix;

	  //main program
	  //generation of transfer matrices
	  trans_abs_point_matrix = gsf::translation_matrix(XYZ_location);
	  trans_abs_center_matrix = gsf::translation_matrix(map_center_location_);

	  //change of referred frame
	  bsf::matrix2DINVERT(trans_abs_center_matrix);
	  bsf::matrix2DPROD(trans_abs_point_matrix,trans_abs_center_matrix,trans_center_point_matrix);
	  bsf::matrix2DPRINT(trans_center_point_matrix);

	  //extraction of XYZ respect to the center of the map, from transfer matrix
	  XYZ_location = gsf::transMatrix2XYZ(trans_center_point_matrix);
	  std::cout << "passed!" << std::endl;

	  XYZ_location.x /= cell_size_;
	  XYZ_location.y /= cell_size_;
	  XYZ_location.z /= cell_size_;

	  //actual conversion to IJK
	  IJK2return[0] = bsf::round_f(XYZ_location.x);
	  IJK2return[1] = bsf::round_f(XYZ_location.y);
	  IJK2return[2] = bsf::round_f(XYZ_location.z);

	  return IJK2return;
  }


  ////Private section////
  //Set functions
  void pff::CellMap::SetCellSize(double cell_size)
  {
	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  cell_size = 0.05;
		  std::cout << "its value has been modified to " << cell_size << " (default value) to make the program work" << std::endl;
	  }
	  cell_size_ = cell_size;
  }
  void pff::CellMap::SetMapSize(geometry_msgs::Vector3 map_size)
  {
	  if(map_size.x < cell_size_)
	  {
		  std::cout << "Error: the map size along X-axis must be larger than cell size!" << std::endl;
		  map_size.x = cell_size_;
		  std::cout << "Since the cell size is " << cell_size_ << ", that map size has been modified to cell size to make the program work" << std::endl;
	  }
	  if(map_size.y < cell_size_)
	  {
		  std::cout << "Error: the map size along Y-axis must be larger than cell size!" << std::endl;
		  map_size.y = cell_size_;
		  std::cout << "Since the cell size is " << cell_size_ << ", that map size has been modified to cell size to make the program work" << std::endl;
	  }
	  if(map_size.z < cell_size_)
	  {
		  std::cout << "Error: the map size along Z-axis must be larger than cell size!" << std::endl;
		  map_size.z = cell_size_;
		  std::cout << "Since the cell size is " << cell_size_ << ", that map size has been modified to cell size to make the program work" << std::endl;
	  }
	  map_size_ = map_size;
  }
  void pff::CellMap::SetMapcenterLocation(geometry_msgs::Vector3 map_center_loc)
  {
	  map_center_location_ = map_center_loc;
  }
  bool pff::CellMap::SetDoubleCellMap(std::vector<std::vector<std::vector<double>>> double_cell_map)
  {
	  int temp_int;

	  //function check
	  //matrix size check
	  temp_int = double_cell_map[0].size();
	  for(int i = 0; i < double_cell_map.size(); i++)
	  {
		  if(double_cell_map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!" << std::endl;
			  goto return_0;
		  }
	  }
	  temp_int = double_cell_map[0][0].size();
	  for(int i = 0; i < double_cell_map.size(); i++)
	  {
		  for(int j = 0; j < double_cell_map[0].size(); j++)
		  {
			  if(double_cell_map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!" << std::endl;
				  goto return_0;
			  }
		  }
	  }


	  cell_map_ = double_cell_map;
	  return true;

	  return_0:
	  return false;
  }




// End namespace "pot_field_functions"
