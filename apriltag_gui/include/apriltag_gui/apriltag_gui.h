#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <apriltag_arm_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
		void start();
		//void showImage(cv::Mat img);
		
		private:
		//callback functions
		void onReceivedImage(const sensor_msgs::ImageConstPtr& msg);
		void onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg);
		void onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg);
		void onReceivedEvent(const std_msgs::Int16ConstPtr& msg);
		void setUpdateValues(int size);
		//float markedSize(float value, float max, float min);
		void obtain_target(int value, int buffer);
		void drawRect(int pos);/*TODO:trasform a range of probabilities into a markedSize value*/
		void drawAndShow(int numObjs);
		void obtainNewPos(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg);
		
		

	private:
		//node
		ros::NodeHandle n;
		image_transport::ImageTransport image_transport_; // std::shared_ptr<image....> image_transport_
		
		//subscriber
		image_transport::Subscriber sub_image_transport_;
		ros::Subscriber sub_detected_image;
		ros::Subscriber sub_prob;
		ros::Subscriber sub_event_bus;
		
		//i topics
		std::string image_topic;
		std::string detections_topic;
		std::string prob_topic;
		std::string event_bus_topic;
		
		//converted image
		cv::Mat opencv_image;

		//vector of center's positions of each detected apriltags
		std::vector<cv::Point2d> positions_2d;

		//vector of detected apriltag probabilities		
		std::vector<float> probs;
		
		//vector of ids
		std::vector<float> id_pos;
		//std::vector<int> id_probs;

		//code
		int increasing_code = 500;
		int start_code = 0;
		int picking_code = 1000;
		int target_code = 5000;
		int target_id;
		int picking_id;
		int code;
		/// eliminate an object after picking, can use std::vector::erase()

		//picking mode
		bool picking;
		//home mode
		bool home;
		//clear position vector after picking
		bool needClear;

		//OpenCV color channel order is B,G,R
		// correct = green
		// wrong = red
		// target = light blue
		// other = violet
		// thickness = 3 to obatin better visualization
		cv::Scalar correct_color = cv::Scalar(0,255,0);
		cv::Scalar wrong_color = cv::Scalar(0,0,255);
		cv::Scalar target_color = cv::Scalar(255,153,51); 
		cv::Scalar other_color = cv::Scalar(204,0,204);
		int thickness = 3;

		//default value to draw rectangle with determinate misure in pixel
		float default_dim_rect = 200.0;

		//number of object detected
		int n_objects;

		//dinamic use
		bool continueUpdatePos;
		
		//flag[0] for image, flag[1] for center positions, flag[2] event_bus
		bool flag[3] = {false,false,false}; 

};
