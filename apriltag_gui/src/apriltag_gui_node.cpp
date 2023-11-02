#include "apriltag_gui/Gui.h"
#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc,char **argv){

	ros::init(argc,argv,"apriltag_gui_node");
	std::cout<<"ho lanciato il nodo"<<std::endl;
	Gui gui;
	std::cout<<"ho creato un oggetto della classe gui"<<std::endl;
	if(gui.setup())
		std::cout<<"tutto bene,sono fuori dal setup"<<std::endl;
	else
		std::cout<<"setup fallito"<<std::endl;
	//cv::imshow("aaaaaaaaaaaaaa",gui.drawRect());
	//cv::waitKey(0);
	
  	/*
	ros::Rate rate(50);
	while(ros::ok()){
		cv::imshow("esempio",gui.drawRect());
		cv::waitKey(40);
		ros::spinOnce();
		rate.sleep();
	}
	ros::shutdown();
	*/
	//ros::spinOnce();
	
	
	/*
	cv::Mat image(1080, 1920, CV_8UC3);
	image = cv::Scalar::all(255);
	cv::drawMarker(image, cv::Point(100, 100), cv::Scalar(255, 255, 0),cv::MarkerTypes::MARKER_SQUARE, 60, 3, 8);
	cv::drawMarker(image, cv::Point(100, 100), cv::Scalar(0,255,0),cv::MARKER_SQUARE,50,3,8);
	int i = 0;
	while(i<10){
		cv::imshow("aaaaaaaaaaaaaa",image);
		cv::waitKey(1000);
		std::cout<<"--------------------------"<<std::endl;
		i++;
	}
	*/
	
	return 0;
}
