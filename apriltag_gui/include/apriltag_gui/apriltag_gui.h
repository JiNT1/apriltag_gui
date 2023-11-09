#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <apriltag_arm_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>

class AprilTagGui{
	public:
		//constructor 
		AprilTagGui();
		//destructor
		~AprilTagGui();
		//configure initial settings
		bool setup();
		//return original unmodified image
		cv::Mat getImage();
		//return marked image
		cv::Mat drawRect();/*TODO:trasform a range of probabilities into a markedSize value*/
		private:
		//callback functions
		void onReceivedImage(const sensor_msgs::ImageConstPtr& msg);
		void onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg);
		void onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg);
		//void onReceivedData(const sensor_msgs::ImageConstPtr& img,const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& detected_array,const std_msgs::Float32MultiArrayConstPtr& probs_array);
		void setValues(int size);
		

	private:
		ros::NodeHandle n;
		image_transport::ImageTransport image_transport_; // std::shared_ptr<image....> image_transport_
		
		image_transport::Subscriber sub_image_transport_;
		ros::Subscriber sub_detected_image;
		ros::Subscriber sub_prob;
		
		/*
		message_filters::Subscriber<sensor_msgs::Image> sub_image;
		message_filters::Subscriber<apriltag_arm_ros::AprilTagDetectionArray> sub_detected_image;
		message_filters::Subscriber<std_msgs::Float32MultiArray> sub_prob;
		*/
		
		//i topics
		std::string info_topic;
		std::string image_topic;
		std::string detections_topic;
		std::string prob_topic;
		
		//converted image
		cv::Mat opencv_image;
		//vector of center's positions of each detected apriltags
		std::vector<cv::Point2d> positions_2d;
		
		//vector of detected apriltag probabilities		
		std::vector<float> probs;
		
		//vector of ids
		std::vector<int> id_probs; 
		std::vector<int> id_pos;
		
		bool flag[3] = {false,false,false}; //flag[0] for image, flag[1] for center positions, flag[2] probabilities
};
