#include "apriltag_gui/apriltag_gui.h"
#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc,char **argv){

	ros::init(argc,argv,"apriltag_gui_node");
	std::cout<<"ho lanciato il nodo"<<std::endl;
	AprilTagGui gui;
	std::cout<<"ho creato un oggetto della classe gui"<<std::endl;
	if(gui.setup())
		std::cout<<"tutto bene,sono fuori dal setup"<<std::endl;
	else
		std::cout<<"setup fallito"<<std::endl;
	
	//ros::Rate rate(100);
	while(ros::ok()){
		gui.start();
		ros::spinOnce();
		//rate.sleep();
	}
	
	//ros::shutdown();
	
	return 0;
}
