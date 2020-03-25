#include "task_handle/task_handle.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "task_handle_node");

	TaskHandle task_handle;

	ros::spin();

	return 0;
}