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
	  std::vector<std::vector<std::vector<bool>>> bool_map;
	  std::vector<std::vector<std::vector<double>>> double_map;

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
	  bool_map.resize(temp_dim[0]);
	  double_map.resize(temp_dim[0]);
	  for(int i = 0; i < temp_dim[0]; i++)
	  {
		  bool_map[i].resize(temp_dim[1]);
		  double_map[i].resize(temp_dim[1]);
		  for(int j = 0; j < temp_dim[1]; j++)
		  {
			  bool_map[i][j].resize(temp_dim[2]);
			  double_map[i][j].resize(temp_dim[2]);
		  }
	  }

	  //variable setting
	  SetMapcenterLocation(map_center_loc);
	  SetBoolCellMap(bool_map);
	  SetDoubleCellMapRep(double_map);
	  SetDoubleCellMapAtt(double_map);
	  SetDoubleCellMapGen(double_map);
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
  std::vector<int> pff::CellMap::GetMapSize4Cell()
  {
	  std::vector<int> map_size4cell; map_size4cell.resize(3);
	  map_size4cell[0] = bool_cell_map_.size();
	  map_size4cell[1] = bool_cell_map_[0].size();
	  map_size4cell[2] = bool_cell_map_[0][0].size();

	  return map_size4cell;
  }
  geometry_msgs::Vector3 pff::CellMap::GetMapcenterLocation()
  {
	  return map_center_location_;
  }
  std::vector<std::vector<std::vector<bool>>> pff::CellMap::GetBoolCellMap()
  {
	  return bool_cell_map_;
  }
  std::vector<std::vector<std::vector<double>>> pff::CellMap::GetDoubleCellMapRep()
   {
 	  return double_cell_map_rep_;
   }
  std::vector<std::vector<std::vector<double>>> pff::CellMap::GetDoubleCellMapAtt()
   {
 	  return double_cell_map_att_;
   }
  std::vector<std::vector<std::vector<double>>> pff::CellMap::GetDoubleCellMapGen()
   {
 	  return double_cell_map_gen_;
   }

  //Other functions
  void pff::CellMap::SaturatorGenMap(double saturation_val)
  {
	  for(int i = 0; i < double_cell_map_gen_.size(); i++)
	  {
		  for(int j = 0; j < double_cell_map_gen_[0].size(); j++)
		  {
			  for(int z = 0; z < double_cell_map_gen_[0][0].size(); z++)
			  {
				  if(double_cell_map_gen_[i][j][z] > saturation_val)
					  double_cell_map_gen_[i][j][z] = saturation_val;
			  }
		  }
	  }
  }
  void pff::CellMap::NormalizationGenMap(double normal_value)
  {

	  double temp_double;

 	  //research of the higher value in the map
 	  temp_double = 0.0;
 	  for(int i = 0; i < double_cell_map_gen_.size(); i++)
 	  {
 		  for(int j = 0; j < double_cell_map_gen_[0].size(); j++)
 		  {
 			  for(int z = 0; z < double_cell_map_gen_[0][0].size(); z++)
 			  {
 				  if(double_cell_map_gen_[i][j][z] > temp_double)
 				  {
 					  temp_double = double_cell_map_gen_[i][j][z];
 				  }
 			  }
 		  }
 	  }

 	  //normalizator
 	  temp_double = normal_value/temp_double;

	  //normalization
 	  for(int i = 0; i < double_cell_map_gen_.size(); i++)
 	  {
 		  for(int j = 0; j < double_cell_map_gen_[0].size(); j++)
 		  {
 			  for(int z = 0; z < double_cell_map_gen_[0][0].size(); z++)
 			  {
 				 double_cell_map_gen_[i][j][z] *= temp_double;
 			  }
 		  }
 	  }

  }

  std::vector<int> pff::CellMap::PointXYZtoIJK(geometry_msgs::Vector3 XYZ_location)
  {
	  std::vector<int> IJK2return; IJK2return.resize(3);
	  std::vector<std::vector<double>> trans_abs_point_matrix, trans_abs_center_matrix;
	  std::vector<std::vector<double>> trans_center_point_matrix;
	  std::vector<std::vector<double>> trans_000_center_matrix, trans_000_point_matrix;

	  //main program
	  //generation of transfer matrices
	  trans_abs_point_matrix = gsf::XYZ2transMatrix(XYZ_location);
	  trans_abs_center_matrix = gsf::XYZ2transMatrix(map_center_location_);
	  trans_000_center_matrix = gsf::XYZ2transMatrix(gsf::makeVector3(map_size_.x/2,map_size_.y/2,map_size_.z/2));

	  //change of referred frame from world to the center of the map
	  bsf::matrix2DINVERT(trans_abs_center_matrix);
	  bsf::matrix2DPROD(trans_abs_center_matrix,trans_abs_point_matrix,trans_center_point_matrix);

	  //change of referred frame from the map center to map [0;0;0] location
	  //necessary to convert XYZ into IJK (which are necessarily positive)
	  bsf::matrix2DPROD(trans_000_center_matrix,trans_center_point_matrix,trans_000_point_matrix);

	  //extraction of XYZ respect to the center of the map, from transfer matrix
	  XYZ_location = gsf::transMatrix2XYZ(trans_000_point_matrix);

	  XYZ_location.x /= cell_size_;
	  XYZ_location.y /= cell_size_;
	  XYZ_location.z /= cell_size_;

	  //conversion from XYZ to IJK
	  IJK2return[0] = bsf::round_f(XYZ_location.x);
	  IJK2return[1] = bsf::round_f(XYZ_location.y);
	  IJK2return[2] = bsf::round_f(XYZ_location.z);

	  //Final check, the point might be outside of the map
	  if(IJK2return[0] > map_size_.x/cell_size_ || IJK2return[0] < 0)
	  {
		  std::cout << "The coordinate X of the point would be " << IJK2return[0] << ", but it is located outside of the map. So, just '-1' has been returned!" << std::endl;
		  IJK2return[0] = -1;
	  }
	  if(IJK2return[1] > map_size_.y/cell_size_ || IJK2return[1] < 0)
	  {
		  std::cout << "The coordinate Y of the point would be " << IJK2return[1] << ", but it is located outside of the map. So, just '-1' has been returned!" << std::endl;
		  IJK2return[1] = -1;
	  }
	  if(IJK2return[2] > map_size_.z/cell_size_ || IJK2return[2] < 0)
	  {
		  std::cout << "The coordinate Z of the point would be " << IJK2return[2] << ", but it is located outside of the map. So, just '-1' has been returned!" << std::endl;
		  IJK2return[2] = -1;
	  }

	  return IJK2return;
  }

  geometry_msgs::Vector3 pff::CellMap::PointIJKtoXYZ(std::vector<int> IJK_location)
  {
	  geometry_msgs::Vector3 XYZ2return;
	  std::vector<std::vector<double>> trans_000_point_matrix, trans_000_center_matrix;
	  std::vector<std::vector<double>> trans_abs_point_matrix, trans_abs_center_matrix, trans_abs_000_matrix;

	  //check
	  if (IJK_location.size() != 3)
	  {
		  std::cout << "Error: every point has to be defined by 3 index (IJK), while it is defined by " << IJK_location.size() << "!" << std::endl;
		  std::cout << "A null geometry message has been returned" << std::endl;
		  return XYZ2return;
	  }

	  //main prog
	  //conversion from IJK to XYZ
	  XYZ2return.x = IJK_location[0]*cell_size_;
	  XYZ2return.y = IJK_location[1]*cell_size_;
	  XYZ2return.z = IJK_location[2]*cell_size_;

	  //generation of transfer matrices
	  trans_000_point_matrix = gsf::XYZ2transMatrix(XYZ2return);
	  trans_000_center_matrix = gsf::XYZ2transMatrix(gsf::makeVector3((map_size_.x+cell_size_)/2,(map_size_.y+cell_size_)/2,(map_size_.z+cell_size_)/2));
	  trans_abs_center_matrix = gsf::XYZ2transMatrix(map_center_location_);

	  //change of referred frame from map [0;0;0] location to the absolute of the map
	  bsf::matrix2DINVERT(trans_000_center_matrix);
	  bsf::matrix2DPROD(trans_abs_center_matrix,trans_000_center_matrix,trans_abs_000_matrix);
	  bsf::matrix2DPROD(trans_abs_000_matrix,trans_000_point_matrix,trans_abs_point_matrix);

	  //extraction of XYZ respect to the world, from transfer matrix
	  XYZ2return = gsf::transMatrix2XYZ(trans_abs_point_matrix);

	  return XYZ2return;
  }

  std::vector<std::vector<int>> pff::CellMap::GetCellsIJK_BoolOccupancy(bool boolean_val)
  {
	  std::vector<std::vector<int>> vector2return;
	  std::vector<int> tmp_IJK; tmp_IJK.resize(3);

	  std::vector<int> size4cell = GetMapSize4Cell();

	  //main prog
	  for(int i = 0; i < size4cell[0]; i++)
	  {
		  for(int j = 0; j < size4cell[1]; j++)
		  {
			  for(int k = 0; k < size4cell[2]; k++)
			  {
				  if(bool_cell_map_[i][j][k] == boolean_val)
				  {
					  tmp_IJK[0] = i;
					  tmp_IJK[1] = j;
					  tmp_IJK[2] = k;
					  vector2return.push_back(tmp_IJK);
				  }
			  }
		  }
	  }

	  return vector2return;
  }

  std::vector<geometry_msgs::Vector3> pff::CellMap::GetCellsXYZ_BoolOccupancy(bool boolean_val)
  {
	  std::vector<std::vector<int>> vector2returnIJK;
	  std::vector<geometry_msgs::Vector3> vector2returnXYZ;
	  geometry_msgs::Vector3 temp_vec3;

	  //main prog
	  vector2returnIJK = GetCellsIJK_BoolOccupancy(boolean_val);

	  vector2returnXYZ.resize(vector2returnIJK.size());
	  for(int i = 0; i < vector2returnIJK.size(); i++)
	  {
		  temp_vec3 = PointIJKtoXYZ(vector2returnIJK[i]);
		  vector2returnXYZ[i] = temp_vec3;
	  }

	  return vector2returnXYZ;
  }

  std::vector<std::vector<int>> pff::CellMap::GetCellsIJK_DoubleComparison(char comparison_symbol, double comparison_val)
  {
	  std::vector<std::vector<int>> vector2return;
	  std::vector<int> tmp_IJK; tmp_IJK.resize(3);
	  std::vector<char> comparison_chars; comparison_chars.resize(3);
	  comparison_chars[0] = '=';
	  comparison_chars[1] = '!';
	  comparison_chars[2] = '<';
	  comparison_chars[3] = '>';

	  std::vector<int> size4cell = GetMapSize4Cell();

	  //main prog
	  if(comparison_symbol == comparison_chars[0])
	  {
		  for(int i = 0; i < size4cell[0]; i++)
		  {
			  for(int j = 0; j < size4cell[1]; j++)
			  {
				  for(int k = 0; k < size4cell[2]; k++)
				  {
					  if(double_cell_map_gen_[i][j][k] == comparison_val)
					  {
						  tmp_IJK[0] = i;
						  tmp_IJK[1] = j;
						  tmp_IJK[2] = k;
						  vector2return.push_back(tmp_IJK);
					  }
				  }
			  }
		  }
		  return vector2return;
	  }else if((comparison_symbol == comparison_chars[1]))
	  {
		  for(int i = 0; i < size4cell[0]; i++)
		  {
			  for(int j = 0; j < size4cell[1]; j++)
			  {
				  for(int k = 0; k < size4cell[2]; k++)
				  {
					  if(double_cell_map_gen_[i][j][k] != comparison_val)
					  {
						  tmp_IJK[0] = i;
						  tmp_IJK[1] = j;
						  tmp_IJK[2] = k;
						  vector2return.push_back(tmp_IJK);
					  }
				  }
			  }
		  }
		  return vector2return;
	  }else if((comparison_symbol == comparison_chars[2]))
	  {
		  for(int i = 0; i < size4cell[0]; i++)
		  {
			  for(int j = 0; j < size4cell[1]; j++)
			  {
				  for(int k = 0; k < size4cell[2]; k++)
				  {
					  if(double_cell_map_gen_[i][j][k] < comparison_val)
					  {
						  tmp_IJK[0] = i;
						  tmp_IJK[1] = j;
						  tmp_IJK[2] = k;
						  vector2return.push_back(tmp_IJK);
					  }
				  }
			  }
		  }
		  return vector2return;
	  }else if((comparison_symbol == comparison_chars[3]))
	  {
		  for(int i = 0; i < size4cell[0]; i++)
		  {
			  for(int j = 0; j < size4cell[1]; j++)
			  {
				  for(int k = 0; k < size4cell[2]; k++)
				  {
					  if(double_cell_map_gen_[i][j][k] > comparison_val)
					  {
						  tmp_IJK[0] = i;
						  tmp_IJK[1] = j;
						  tmp_IJK[2] = k;
						  vector2return.push_back(tmp_IJK);
					  }
				  }
			  }
		  }
		  return vector2return;
	  }else  //[ERROR MSG]
	  {
		  bsc::UsefulCharString errors_str;
		  std::cout << errors_str.get_invalid_input_str() << std::endl;
		  std::cout << "Just the following chars are allowed:" << std::endl;
		  for(int i = 0; i < comparison_chars.size(); i++)
		  {
			  std::cout << comparison_chars[i] << "; ";
		  }
		  std::cout << std::endl;
		  std::cout << "An empty vector has been returned." << std::endl;
	  }

	  return vector2return;
  }

  std::vector<geometry_msgs::Vector3> pff::CellMap::GetCellsXYZ_DoubleComparison(char comparison_symbol, double comparison_val)
  {
	  std::vector<std::vector<int>> vector2returnIJK;
	  std::vector<geometry_msgs::Vector3> vector2returnXYZ;
	  geometry_msgs::Vector3 temp_vec3;

	  //main prog
	  vector2returnIJK = GetCellsIJK_DoubleComparison(comparison_symbol, comparison_val);

	  vector2returnXYZ.resize(vector2returnIJK.size());
	  for(int i = 0; i < vector2returnIJK.size(); i++)
	  {
		  temp_vec3 = PointIJKtoXYZ(vector2returnIJK[i]);
		  vector2returnXYZ[i] = temp_vec3;
	  }

	  return vector2returnXYZ;
  }

  bool pff::CellMap::ObjectInsertion(std::string obj_kind, geometry_msgs::Vector3 obj_position, geometry_msgs::Vector3 obj_orientation, std::vector<double> obj_valume_sizes)
  {
	  double curr_x, curr_y, curr_z;
	  double radius, height, base1, base2;
	  geometry_msgs::Vector3 temp_vector_XYZ;
	  std::vector<int> temp_vector_IJK; temp_vector_IJK.resize(3);
	  std::vector<geometry_msgs::Vector3> set_coord_XYZ;
	  std::vector<std::vector<double>> trans_abs_objcentre_matrix, trans_objcentre_point_matrix, trans_abs_point_matrix;

	  std::vector<std::string> obj_types; obj_types.resize(4);
	  obj_types[0] = "SPHERE";
	  obj_types[1] = "BOX";
	  obj_types[2] = "CYLINDER";
	  obj_types[3] = "CONE";

	  //input check

	  //main program
	  //generation of the point list of the hypothetical solid lated in the position and orientation [0,0,0;0,0,0]
	  if(obj_kind == obj_types[0]) //[SPHERE]
	  {
		  //check obj size
		  if(obj_valume_sizes.size() != 1)
		  {
			  std::cout << obj_types[0] << " volume is defined by 1 value (radius), while the object size vector contains " << obj_valume_sizes.size() << " values!" << std::endl;
			  goto return_0;
		  }
		  radius = obj_valume_sizes[0];

		  curr_z = 0.0;
		  while(curr_z*curr_z <= radius*radius)
		  {
			  curr_y = 0.0;
			  while(curr_y*curr_y + curr_z*curr_z <= radius*radius)
			  {
				  curr_x = 0.0;
				  while(curr_x*curr_x + curr_y*curr_y + curr_z*curr_z <= radius*radius)
				  {
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,curr_z));

					  curr_x += cell_size_;
				  }
				  curr_y += cell_size_;
			  }
			  curr_z += cell_size_;
		  }
	  }else if(obj_kind == obj_types[1]) //[BOX]
	  {
		  //check obj size
		  if(obj_valume_sizes.size() != 3)
		  {
			  std::cout << obj_types[1] << " volume is defined by 3 values (base size1, base size2, and height), while the object size vector contains " << obj_valume_sizes.size() << " values!" << std::endl;
			  goto return_0;
		  }
		  base1 = obj_valume_sizes[0];
		  base2 = obj_valume_sizes[1];
		  height = obj_valume_sizes[2];

		  curr_z = 0.0;
		  while(curr_z <= height/2)
		  {
			  curr_x = 0.0;
			  while(curr_x <= base1/2)
			  {
				  curr_y = 0.0;
				  while(curr_y <= base2/2)
				  {
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,curr_z));

					  curr_x += cell_size_;
				  }
				  curr_y += cell_size_;
			  }
			  curr_z += cell_size_;
		  }
	  }else if(obj_kind == obj_types[2]) //[CYLINDER]
	  {
		  //check obj size
		  if(obj_valume_sizes.size() != 2)
		  {
			  std::cout << obj_types[2] << " volume is defined by 2 values (base radius, and height), while the object size vector contains " << obj_valume_sizes.size() << " values!" << std::endl;
			  goto return_0;
		  }
		   radius = obj_valume_sizes[0];
		   height = obj_valume_sizes[1];

		  curr_z = 0.0;
		  while(curr_z <= height/2)
		  {
			  curr_x = 0.0;
			  while(curr_x*curr_x <= radius*radius)
			  {
				  curr_y = 0.0;
				  while(curr_x*curr_x + curr_y*curr_y <= radius*radius)
				  {
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,-curr_z));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,curr_z));

					  curr_x += cell_size_;
				  }
				  curr_y += cell_size_;
			  }
			  curr_z += cell_size_;
		  }
	  }else if(obj_kind == obj_types[3]) //[CONE]
	  {
		  //check obj size
		  if(obj_valume_sizes.size() != 2)
		  {
			  std::cout << obj_types[3] << " volume is defined by 2 values (base radius, and height), while the object size vector contains " << obj_valume_sizes.size() << " values!" << std::endl;
			  goto return_0;
		  }
		   double radius_base = obj_valume_sizes[0];
		   height = obj_valume_sizes[1];

		  curr_z = 0.0;
		  while(curr_z <= height)
		  {
			  radius = radius_base*(height-curr_z)/height;
			  curr_x = 0.0;
			  while(curr_x*curr_x <= radius*radius)
			  {
				  curr_y = 0.0;
				  while(curr_x*curr_x + curr_y*curr_y <= radius*radius)
				  {
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,-curr_y,curr_z-height/2));
					  set_coord_XYZ.push_back(gsf::makeVector3(-curr_x,curr_y,curr_z-height/2));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,-curr_y,curr_z-height/2));
					  set_coord_XYZ.push_back(gsf::makeVector3(curr_x,curr_y,curr_z-height/2));

					  curr_x += cell_size_;
				  }
				  curr_y += cell_size_;
			  }
			  curr_z += cell_size_;
		  }

	  }else //[ERROR MSG]
	  {
		  bsc::UsefulCharString errors_str;
		  std::cout << errors_str.get_invalid_input_str() << std::endl;
		  std::cout << "Just the following object kinds are allowed:" << std::endl;
		  for(int i = 0; i < obj_types.size(); i++)
		  {
			  std::cout << obj_types[i] << "; ";
		  }
		  std::cout << std::endl;
		  goto return_0;
	  }

	  //translation of the point list into the real world coordinates and then into the IJK map
	  //generation of transfer matrices
	  trans_abs_objcentre_matrix = gsf::XYZ2transMatrix(obj_position);
	  for(int i = 0; i < set_coord_XYZ.size(); i++)
	  {
		  //generation of transfer matrices
		  trans_objcentre_point_matrix = gsf::XYZ2transMatrix(set_coord_XYZ[i]);
		  bsf::matrix2DPROD(trans_abs_objcentre_matrix,trans_objcentre_point_matrix,trans_abs_point_matrix);
		  //coordinates change from XYZ into IJK
		  temp_vector_XYZ = gsf::transMatrix2XYZ(trans_abs_point_matrix);
		  temp_vector_IJK = PointXYZtoIJK(temp_vector_XYZ);
		  //insertion of the boolean value into the map
		  bool_cell_map_[temp_vector_IJK[0]][temp_vector_IJK[1]][temp_vector_IJK[2]] = 1;
	  }


	  return true;

	  return_0:
	  return false;
  }

  bool pff::CellMap::RepulsiveMap_generator(double repulsive_radius, double infinite_val)
  {
  	  std::vector< std::vector< std::vector< double > > > map2return;
  	  std::vector<int> temp_vec_IJK; temp_vec_IJK.resize(3);
 	  std::vector<std::vector<int>> matrix_coor_vector;
  	  std::vector<std::vector<int>> temp_matrix_coor_vector;
  	  std::vector<std::vector<int>> temp_matrix_coor_vector2;
  	  std::vector<std::vector<int>> temp_27_matrix_coor_vector;
  	  temp_27_matrix_coor_vector.resize(6);
  	  for(int i = 0; i < temp_27_matrix_coor_vector.size(); i++)
  		  temp_27_matrix_coor_vector[i].resize(3);
  	  double sat_val;
  	  int vec[3];
  	  int progression;
  	  bool done, present;
  	  long int t1, t2;

  	  //getting initial time
  	  t1 = bsf::getCurrentTime();

  	  //initial check
  	  if(repulsive_radius <= 0.0)
  	  {
  		  std::cout << "Error: the radius of the repulsive field must be strictly positive!" << std::endl;
  		  std::cout << "Repulsive field execution has been aborted..." << std::endl;
  		  return false;
  	  }

  	  if(repulsive_radius < cell_size_)
  	  {
  		  std::cout << "Warning: the radius of the repulsive field is smaller than the cell size. It means there is not repulsive field..." << std::endl;
  	  }


  	  //main program start
  	  int cell_involved = 1 + repulsive_radius/cell_size_;

  	  //repulsive algorithm
  	  map2return = double_cell_map_rep_;
	  for(int i = 0; i < map2return.size(); i++)
	  {
		  for(int j = 0; j < map2return[i].size(); j++)
		  {
			  for(int k = 0; k < map2return[i][j].size(); k++)
			  {
				  if(bool_cell_map_[i][j][k])
				  {
					  map2return[i][j][k] = infinite_val;
					  temp_vec_IJK[0] = i;
					  temp_vec_IJK[1] = j;
					  temp_vec_IJK[2] = k;
					  matrix_coor_vector.push_back(temp_vec_IJK);
				  }
			  }
		  }
	  }


  	  progression = 0;

  	  //check obstacle existence
  	  if(matrix_coor_vector.size() == 0)
  	  {
  		  std::cout << "Warning: apparently there are not obstacle in the map..." << std::endl;
  	  }

  	  done = false;
  	  while(!done)
  	  {

  		  temp_matrix_coor_vector.clear();

  		  temp_matrix_coor_vector = matrix_coor_vector;
  		  temp_matrix_coor_vector2.clear();
  		  matrix_coor_vector.clear();

  		  for(int i = 0; i < temp_matrix_coor_vector.size(); i++)
  		  {
  			  //value insertion (generation of the double cross including the considered point)
  			  //faster the the bubble composed by 26 elements
  			  for(int c = 0; c < temp_27_matrix_coor_vector.size(); c++)
  			  {
  				  temp_27_matrix_coor_vector[c] = temp_matrix_coor_vector[i];
  			  }
  			  temp_27_matrix_coor_vector[0][0] += 1;
  			  temp_27_matrix_coor_vector[1][0] -= 1;
  			  temp_27_matrix_coor_vector[2][1] += 1;
  			  temp_27_matrix_coor_vector[3][1] -= 1;
  			  temp_27_matrix_coor_vector[4][2] += 1;
  			  temp_27_matrix_coor_vector[5][2] -= 1;

  			  for(int z = 0; z < temp_27_matrix_coor_vector.size(); z++)
  			  {
  				  //new layer addiction
  				  //point not outside of the matrix
  				  if(temp_27_matrix_coor_vector[z][0] >= 0 &&
  						  temp_27_matrix_coor_vector[z][1] >= 0 &&
  						  temp_27_matrix_coor_vector[z][2] >= 0 &&
  						  temp_27_matrix_coor_vector[z][0] < map2return.size() &&
  						  temp_27_matrix_coor_vector[z][1] < map2return[0].size() &&
  						  temp_27_matrix_coor_vector[z][2] < map2return[0][0].size())
  				  {
  					  //if the point has not been already touched by the algorithm map or it is not a wall
  					  if(map2return[temp_27_matrix_coor_vector[z][0]][temp_27_matrix_coor_vector[z][1]][temp_27_matrix_coor_vector[z][2]] == 0.0)
  					  {
  						  //insertion in the new layer
  						  temp_matrix_coor_vector2.push_back(temp_27_matrix_coor_vector[z]);
  					  }
  				  }
  			  }
  		  }


  		  //delete all the duplicated points
  		  for(int i = 0; i < temp_matrix_coor_vector2.size(); i++)
  		  {
  			  present = false;
  			  for(int k = 0; k < i; k++)
  			  {
  				  if(temp_matrix_coor_vector2[i][0] == temp_matrix_coor_vector2[k][0] && temp_matrix_coor_vector2[i][1] == temp_matrix_coor_vector2[k][1] && temp_matrix_coor_vector2[i][2] == temp_matrix_coor_vector2[k][2])
  				  {
  					  present = true;
  					  break;
  				  }
  			  }
  			  if(!present)
  			  {
  				  matrix_coor_vector.push_back(temp_matrix_coor_vector2[i]);
  			  }
  		  }

  		  progression++;
  		  for(int i = 0; i < matrix_coor_vector.size(); i++)
  		  {
  			  sat_val = 1.0*(cell_involved-progression)/cell_involved;
  			  //values around objects
  			  map2return[matrix_coor_vector[i][0]][matrix_coor_vector[i][1]][matrix_coor_vector[i][2]] = sat_val;
  			  //values on the edge of the map could be introduced, but it does not improve the computation

  		  }

  		  //exit condition: there are no cells to cover yet or the system reached the edge of the potential field
  		  if(matrix_coor_vector.size() == 0 || cell_involved-progression <= 1)
  			  done = true;

  		  //feedback for the user
  		  if(progression % 10 == 0)
  			  std::cout << "potential field expansion number: " << progression << std::endl;
  	  }

  	  double_cell_map_rep_ = map2return;

  	  //getting final time
  	  t2 = bsf::getCurrentTime();
  	  std::cout << "The repulsive field has been generated successfully in " << t2-t1 << " seconds!" << std::cout;
  	  return true;
  }


  bool pff::CellMap::AttractiveMapCl_generator(geometry_msgs::Vector3 goal_position, double wave_step)
  {
	  std::vector< std::vector< std::vector< double > > > map2return;
	  std::vector<int> goal_IJK; goal_IJK.resize(3);
	  long int t1, t2;

	  //getting initial time
	  t1 = bsf::getCurrentTime();

	  //put the goal position into the matrix definition
	  goal_IJK = PointXYZtoIJK(goal_position);
	  //exit condition if the goal point is not included in the map
	  if(goal_IJK[0] < 0 || goal_IJK[1] < 0 || goal_IJK[2] < 0)
	  {
		  std::cout << "Error: the goal position is not included in the map! Attractive field execution has been aborted..." << std::endl;
		  return false;
	  }

	  //initial occupancy check for goal position
	  if(bool_cell_map_[goal_IJK[0]][goal_IJK[1]][goal_IJK[2]] == 1)
	  {
		  std::cout << "Warning:The goal position is located inside of an obstacle." << std::endl;
		  std::cout << "The cell (" << goal_IJK[0] << ", " << goal_IJK[1] << ", " << goal_IJK[2] << ") is: occupied by a obstacle!" << std::endl;
		  std::cout << "The attractive field generation is going on anyway..." << std::endl;
	  }

	  //map definition
	  map2return = double_cell_map_att_;
	  //classical algorithm
	  for(int i = 0; i < map2return.size(); i++)
	  {
		  for(int j = 0; j < map2return[i].size(); j++)
		  {
			  for(int k = 0; k < map2return[i][j].size(); k++)
			  {
				  map2return[i][j][k] = (goal_IJK[0]-i)+(goal_IJK[1]-j)+(goal_IJK[2]-k) +1;
			  }
		  }
	  }

	  double_cell_map_att_ = map2return;
	  //getting final time
	  t2 = bsf::getCurrentTime();
	  std::cout << "The attractive field has been generated successfully using the classical approach in " << t2-t1 << " seconds!" << std::cout;
	  return true;
  }


  bool pff::CellMap::AttractiveMapWF_generator(geometry_msgs::Vector3 goal_position, double wave_step)
  {
	  std::vector< std::vector< std::vector< double > > > map2return;
	  std::vector<int> temp_matrix_pos; temp_matrix_pos.resize(3);
	  std::vector<std::vector<int>> matrix_coor_vector;
	  std::vector<std::vector<int>> temp_matrix_coor_vector;
	  std::vector<std::vector<int>> temp_matrix_coor_vector2;
	  std::vector<std::vector<int>> temp_27_matrix_coor_vector;
	  temp_27_matrix_coor_vector.resize(6);
	  for(int i = 0; i < temp_27_matrix_coor_vector.size(); i++)
		  temp_27_matrix_coor_vector[i].resize(3);

	  int progression;
	  bool done, present;
	  long int t1, t2;

	  //getting initial time
	  t1 = bsf::getCurrentTime();

	  //put the goal position into the matrix definition
	  temp_matrix_pos = PointXYZtoIJK(goal_position);
	  //exit condition if the goal point is not included in the map
	  if(temp_matrix_pos[0] < 0 || temp_matrix_pos[1] < 0 || temp_matrix_pos[2] < 0)
	  {
		  std::cout << "Error: the goal position is not included in the map! Attractive field execution has been aborted..." << std::endl;
		  return false;
	  }

	  //initial occupancy check for goal position
	  if(bool_cell_map_[temp_matrix_pos[0]][temp_matrix_pos[1]][temp_matrix_pos[2]] == 1)
	  {
		  std::cout << "Warning:The goal position is located inside of an obstacle." << std::endl;
		  std::cout << "The cell (" << temp_matrix_pos[0] << ", " << temp_matrix_pos[1] << ", " << temp_matrix_pos[2] << ") is: occupied by a obstacle!" << std::endl;
		  std::cout << "The attractive field generation is going on anyway..." << std::endl;
	  }

	  //map initial definition
	  map2return = double_cell_map_att_;
	  for(int i = 0; i < map2return.size(); i++)
	  {
		  for(int j = 0; j < map2return[i].size(); j++)
		  {
			  for(int k = 0; k < map2return[i][j].size(); k++)
			  {
				  map2return[i][j][k] = bool_cell_map_[i][j][k]*wave_step;
			  }
		  }
	  }

	  //wave-front algorithm
	  matrix_coor_vector.push_back(temp_matrix_pos);

	  done = false;
	  progression = 0;
	  while(!done)
	  {
		  //potential value increasing
		  progression++;

		  //potential value update
		  for(int i = 0; i < matrix_coor_vector.size(); i++)
		  {
			  map2return[matrix_coor_vector[i][0]][matrix_coor_vector[i][1]][matrix_coor_vector[i][2]] = wave_step*progression;
		  }

		  //reset of the front-layer
		  temp_matrix_coor_vector.clear();

		  temp_matrix_coor_vector = matrix_coor_vector;
		  temp_matrix_coor_vector2.clear();
		  matrix_coor_vector.clear();

		  for(int i = 0; i < temp_matrix_coor_vector.size(); i++)
		  {

			  //value insertion (generation of the double cross including the considered point)
			  //faster the the bubble composed by 26 elements
			  for(int c = 0; c < temp_27_matrix_coor_vector.size(); c++)
			  {
				  temp_27_matrix_coor_vector[c] = temp_matrix_coor_vector[i];
			  }
			  temp_27_matrix_coor_vector[0][0] += 1;
			  temp_27_matrix_coor_vector[1][0] -= 1;
			  temp_27_matrix_coor_vector[2][1] += 1;
			  temp_27_matrix_coor_vector[3][1] -= 1;
			  temp_27_matrix_coor_vector[4][2] += 1;
			  temp_27_matrix_coor_vector[5][2] -= 1;

			  for(int z = 0; z < temp_27_matrix_coor_vector.size(); z++)
			  {
				  //new layer addiction
				  //point not outside of the matrix
				  if(temp_27_matrix_coor_vector[z][0] >= 0 &&
						  temp_27_matrix_coor_vector[z][1] >= 0 &&
						  temp_27_matrix_coor_vector[z][2] >= 0 &&
						  temp_27_matrix_coor_vector[z][0] < map2return.size() &&
						  temp_27_matrix_coor_vector[z][1] < map2return[0].size() &&
						  temp_27_matrix_coor_vector[z][2] < map2return[0][0].size())
				  {
					  //if the point has not been already touched by the algorithm map or it is not a wall
					  if(map2return[temp_27_matrix_coor_vector[z][0]][temp_27_matrix_coor_vector[z][1]][temp_27_matrix_coor_vector[z][2]] == 0)
					  {
						  //insertion in the new layer
						  temp_matrix_coor_vector2.push_back(temp_27_matrix_coor_vector[z]);
					  }
				  }
			  }
		  }

		  //delete all the duplicated points
		  for(int i = 0; i < temp_matrix_coor_vector2.size(); i++)
		  {
			  present = false;
			  for(int k = 0; k < i; k++)
			  {
				  if(temp_matrix_coor_vector2[i][0] == temp_matrix_coor_vector2[k][0] && temp_matrix_coor_vector2[i][1] == temp_matrix_coor_vector2[k][1] && temp_matrix_coor_vector2[i][2] == temp_matrix_coor_vector2[k][2])
				  {
					  present = true;
					  break;
				  }
			  }
			  if(!present)
				  matrix_coor_vector.push_back(temp_matrix_coor_vector2[i]);
		  }

		  //exit condition: there are no cells to cover yet
		  if(matrix_coor_vector.size() == 0)
			  done = true;

		  //feedback for the user
		  if(progression % 100 == 0)
			  std::cout << "potential field expansion number: " << progression << std::endl;
	  }

	  double_cell_map_att_ = map2return;
	  //getting final time
	  t2 = bsf::getCurrentTime();
	  std::cout << "The attractive field has been generated successfully using the wave-front approach in " << t2-t1 << " seconds!" << std::cout;
	  return true;
  }

  bool pff::CellMap::PotentialMap_generator(double rep_radius, geometry_msgs::Vector3 goal_position, double wave_step, bool att_wave_app)
  {
	  bool bool2return;
	  bool tempBool1, tempBool2;
	  double high_double;
	  long int t1, t2;

	  //getting initial time
	  t1 = bsf::getCurrentTime();

	  //attractive field
	  if(att_wave_app)
	  {
		  tempBool1 = AttractiveMapWF_generator(goal_position, wave_step);
	  }else{
		  tempBool1 = AttractiveMapCl_generator(goal_position, wave_step);
	  }

	  //repulsive field
	  high_double = 0.0;
	  for(int i = 0; i < double_cell_map_att_.size(); i++)
	  {
		  for(int j = 0; j < double_cell_map_att_[i].size(); j++)
		  {
			  for(int k = 0; k < double_cell_map_att_[i][j].size(); k++)
			  {
				  if(high_double < double_cell_map_att_[i][j][k])
					  high_double = double_cell_map_att_[i][j][k];
			  }
		  }
	  }
	  tempBool2 = RepulsiveMap_generator(rep_radius, high_double);

	  //general potential field
	  bool2return = tempBool1 + tempBool2;
	  if(!bool2return)
	  {
		  std::cout << "Error: the execution of the potential field map has been aborted..." << std::endl;
	  }else
	  {
		  bsf::matrix3DSUM(double_cell_map_att_,double_cell_map_rep_,double_cell_map_gen_);
		  //getting final time
		  t2 = bsf::getCurrentTime();
		  std::cout << "The general potential field has been generated successfully in " << t2-t1 << " seconds!" << std::cout;
	  }
	  return bool2return;
  }

  std::vector<std::vector<int>> pff::CellMap::pathExtractor_IJK(geometry_msgs::Vector3 initial_position)
  {
	  std::vector<int> tempIJK; tempIJK.resize(3);
	  std::vector<int> tempIJK2; tempIJK2.resize(3);
	  std::vector<std::vector<int>> vecIJK2return;
	  std::vector<std::vector<int>> tempVecIJK;
	  double abs_min_val, temp_val;
	  bool done, new_found;

	  tempIJK = PointXYZtoIJK(initial_position);
	  vecIJK2return.push_back(tempIJK);
	  //exit condition if the initial point is not included in the map
	  if(tempIJK[0] < 0 || tempIJK[1] < 0 || tempIJK[2] < 0)
	  {
		  std::cout << "Error: the initial position is not included in the map! Execution aborted..." << std::endl;
		  std::cout << "A vector containing just the initial point has been returned..." << std::endl;
		  return vecIJK2return;
	  }

	  //research of the minimum value in the map
	  abs_min_val = double_cell_map_gen_[0][0][0];
	  for(int i = 0; i < double_cell_map_gen_.size(); i++)
	  {
		  for(int j = 0; j < double_cell_map_gen_[i].size(); j++)
		  {
			  for(int k = 0; k < double_cell_map_gen_[i][j].size(); k++)
			  {
				  if(abs_min_val > double_cell_map_gen_[i][j][k])
					  abs_min_val = double_cell_map_gen_[i][j][k];
			  }
		  }
	  }

	  temp_val = double_cell_map_gen_[tempIJK[0]][tempIJK[1]][tempIJK[2]];
	  done = false;
	  while(!done)
	  {
		  //generation of the points to analyze around the considered one
		  tempVecIJK.clear();
		  //X
		  tempIJK2[0] = tempIJK[0]+1; tempIJK2[1] = tempIJK[1]; tempIJK2[2] = tempIJK[2];
		  tempVecIJK.push_back(tempIJK2);
		  tempIJK2[0] = tempIJK[0]-1; tempIJK2[1] = tempIJK[1]; tempIJK2[2] = tempIJK[2];
		  tempVecIJK.push_back(tempIJK2);
		  //Y
		  tempIJK2[0] = tempIJK[0]; tempIJK2[1] = tempIJK[1]+1; tempIJK2[2] = tempIJK[2];
		  tempVecIJK.push_back(tempIJK2);
		  tempIJK2[0] = tempIJK[0]; tempIJK2[1] = tempIJK[1]-1; tempIJK2[2] = tempIJK[2];
		  tempVecIJK.push_back(tempIJK2);
		  //Z
		  tempIJK2[0] = tempIJK[0]; tempIJK2[1] = tempIJK[1]; tempIJK2[2] = tempIJK[2]+1;
		  tempVecIJK.push_back(tempIJK2);
		  tempIJK2[0] = tempIJK[0]; tempIJK2[1] = tempIJK[1]; tempIJK2[2] = tempIJK[2]-1;
		  tempVecIJK.push_back(tempIJK2);

		  new_found = false;
		  for(int p1 = 0; p1 > tempVecIJK.size(); p1++)
		  {
			  if(temp_val > double_cell_map_gen_[tempVecIJK[p1][0]][tempVecIJK[p1][1]][tempVecIJK[p1][2]])
			  {
				  temp_val = double_cell_map_gen_[tempVecIJK[p1][0]][tempVecIJK[p1][1]][tempVecIJK[p1][2]];
				  tempIJK2[0] = tempVecIJK[p1][0];
				  tempIJK2[1] = tempVecIJK[p1][1];
				  tempIJK2[2] = tempVecIJK[p1][2];
				  new_found = true;
			  }
		  }

		  if(!new_found)
		  {
			  done = true;
			  std::cout << "The path extraction has been completed successfully, but a local minimum has been found!" << std::endl;
		  }

		  //new point update
		  tempIJK = tempIJK2;
		  vecIJK2return.push_back(tempIJK);

		  //if absolute minimum has been reached
		  if(temp_val == abs_min_val)
		  {
			  done = true;
			  std::cout << "The path extraction has been completed successfully until the absolute minimum!" << std::endl;
		  }
	  }
	  return vecIJK2return;
  }

  std::vector<geometry_msgs::Vector3> pff::CellMap::pathExtractor_XYZ(geometry_msgs::Vector3 initial_position)
  {
	  std::vector<std::vector<int>> vecIJK = pathExtractor_IJK(initial_position);
	  std::vector<geometry_msgs::Vector3> vecXYZ; vecXYZ.resize(vecIJK.size());

	  //conversion IJK --> XYZ
	  for(int i = 0; i < vecIJK.size(); i++)
	  {
		  vecXYZ[i] = PointIJKtoXYZ(vecIJK[i]);
	  }

	  return vecXYZ;
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
  bool pff::CellMap::SetBoolCellMap(std::vector<std::vector<std::vector<bool>>> bool_cell_map)
  {
	  int temp_int;

	  //function check
	  //matrix size check
	  temp_int = bool_cell_map[0].size();
	  for(int i = 0; i < bool_cell_map.size(); i++)
	  {
		  if(bool_cell_map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!" << std::endl;
			  goto return_0;
		  }
	  }
	  temp_int = bool_cell_map[0][0].size();
	  for(int i = 0; i < bool_cell_map.size(); i++)
	  {
		  for(int j = 0; j < bool_cell_map[0].size(); j++)
		  {
			  if(bool_cell_map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!" << std::endl;
				  goto return_0;
			  }
		  }
	  }


	  bool_cell_map_ = bool_cell_map;
	  return true;

	  return_0:
	  return false;
  }
  bool pff::CellMap::SetDoubleCellMapRep(std::vector<std::vector<std::vector<double>>> double_cell_map)
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


	  double_cell_map_rep_ = double_cell_map;
	  return true;

	  return_0:
	  return false;
  }
  bool pff::CellMap::SetDoubleCellMapAtt(std::vector<std::vector<std::vector<double>>> double_cell_map)
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


	  double_cell_map_att_ = double_cell_map;
	  return true;

	  return_0:
	  return false;
  }
  bool pff::CellMap::SetDoubleCellMapGen(std::vector<std::vector<std::vector<double>>> double_cell_map)
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


	  double_cell_map_gen_ = double_cell_map;
	  return true;

	  return_0:
	  return false;
  }




// End namespace "pot_field_functions"
