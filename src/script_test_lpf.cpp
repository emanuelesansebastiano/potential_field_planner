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

	bsf::standardSleep();
	//real program init


	ros::shutdown();
	return 0;
}
