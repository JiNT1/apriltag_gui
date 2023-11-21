#include "apriltag_gui/apriltag_gui.h"
#include <ros/ros.h>

int main(int argc,char **argv)
{

	ros::init(argc,argv,"apriltag_gui_node");
	std::cout<<"ho lanciato il nodo"<<std::endl;
	AprilTagGui gui;
	if(gui.setup())
		std::cout<<"tutto bene,sono fuori dal setup"<<std::endl;
	else
		std::cout<<"setup fallito"<<std::endl;
	while(ros::ok() && !(gui.stop())){
		gui.start();
		ros::spinOnce();
	}	
	ros::shutdown();
	return 0;
}
