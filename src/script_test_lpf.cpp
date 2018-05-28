 /*Author: Emanuele Sansebastiano */

// libs
#include <ros/ros.h>
#include <potential_field_planner/lib_pot_field.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "script_test_lpf");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace bsc = basic_side_classes;
	namespace bsf = basic_side_functions;
	namespace gsf = geometry_side_functions;
	namespace pff = pot_field_functions;

	bsf::standardSleep();

	//real program init
	double map_size[] = {10.0, 10.0, 10.0};
	geometry_msgs::Vector3 map_size_xyz = gsf::makeVector3(map_size[0],map_size[1],map_size[2]);
	double cellsize = 0.1;
	double map_centre[] = {0.0,0.0,0.0};
	geometry_msgs::Vector3 vector_map_centre = gsf::makeVector3(map_centre[0],map_centre[1],map_centre[2]);

	pff::CellMap map2use(cellsize,map_size_xyz,vector_map_centre);

	std::cout << "Map cell size: " << map2use.GetCellSize() << std::endl;
	std::cout << "Origin map size: " << map_size_xyz.x << ", " << map_size_xyz.y << ", " << map_size_xyz.z << std::endl;
	map_size_xyz = map2use.GetMapSize();
	std::cout << "Actual Map size: " << map_size_xyz.x << ", " << map_size_xyz.y << ", " << map_size_xyz.z << std::endl;
	std::vector<int>map_size_xyz_v = map2use.GetMapSize4Cell();
	std::cout << "Actual Map size (cell number): " << map_size_xyz_v[0] << ", " << map_size_xyz_v[1] << ", " << map_size_xyz_v[2] << std::endl;

	std::vector<int> IJK;
	geometry_msgs::Vector3 XYZ;
	XYZ = map2use.GetMapcenterLocation();
	XYZ.x = 7.0;
	XYZ.y = -7.0;
	XYZ.z = .0;
	std::cout << "XYZ position: " << XYZ.x << ", " << XYZ.y << ", " << XYZ.z << std::endl;
	IJK = map2use.PointXYZtoIJK(XYZ);
	std::cout << "IJK position: " << IJK[0] << ", " << IJK[1] << ", " << IJK[2] << std::endl;
	XYZ = map2use.PointIJKtoXYZ(IJK);
	std::cout << "XYZ position: " << XYZ.x << ", " << XYZ.y << ", " << XYZ.z << std::endl;

	bool tamp_val_bool1;
	bool tamp_val_bool2;
	bool tamp_val_bool3;

	tamp_val_bool1 = 1;
	tamp_val_bool2 = 1;
	tamp_val_bool3 = 0;

	std::cout << tamp_val_bool3 << std::endl;

	tamp_val_bool3 += tamp_val_bool1;
	std::cout << tamp_val_bool3 << std::endl;

	tamp_val_bool3 += tamp_val_bool1;
	std::cout << tamp_val_bool3 << std::endl;

	tamp_val_bool3 += tamp_val_bool1;
	std::cout << tamp_val_bool3 << std::endl;

	tamp_val_bool3 -= tamp_val_bool1;
	std::cout << tamp_val_bool3 << std::endl;


	//obj insertion
	std::string str_ob = "SPHERE";
	geometry_msgs::Vector3 obj_pos;
	obj_pos.x = 1.0;
	obj_pos.y = 1.0;
	obj_pos.z = -1.45;
	geometry_msgs::Vector3 obj_ori;
	obj_ori.x = 0;
	obj_ori.y = 0;
	obj_ori.z = 0;
	std::vector<double> obj_sizes;
	obj_sizes.push_back(0.1);

	map2use.ObjectInsertion(str_ob,obj_pos,obj_ori,obj_sizes);
	std::cout << "passed_ out!" << std::endl;
	std::vector<std::vector<int>> occ_cells;
	occ_cells = map2use.GetCellsIJK_BoolOccupancy();
	for(int i = 0; i < occ_cells.size(); i++)
	{
		std::cout << "p" << i << ": " << occ_cells[i][0] << ", " << occ_cells[i][1] << ", " << occ_cells[i][2] << std::endl;
	}


	ros::shutdown();
	return 0;
}
