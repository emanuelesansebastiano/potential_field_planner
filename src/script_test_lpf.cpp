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
	map_size_xyz = map2use.GetMapSize4Cell();
	std::cout << "Actual Map size (cell number): " << map_size_xyz.x << ", " << map_size_xyz.y << ", " << map_size_xyz.z << std::endl;

	std::vector<int> IJK;
	geometry_msgs::Vector3 XYZ;
	XYZ.x = 2.05;
	XYZ.y = 1.0;
	XYZ.z = 0.0;
	IJK = map2use.PointXYZtoIJK(XYZ);
	std::cout << "IJK position: " << IJK[0] << ", " << IJK[1] << ", " << IJK[2] << std::endl;



	ros::shutdown();
	return 0;
}
