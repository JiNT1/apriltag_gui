#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <apriltag_arm_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <std_srvs/Empty.h>
#include <rosneuro_msgs/NeuroEvent.h>

#include <std_msgs/Int16.h>

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
		//start graphic user interface
		void start();
		//stop the system
		bool stop();
		
		private:
			//callback functions
			void onReceivedImage(const sensor_msgs::ImageConstPtr& msg);
			void onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg);
			void onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg);
			void onReceivedEvent(const rosneuro_msgs::NeuroEventConstPtr& msg);
			//void onReceivedEvent(const std_msgs::Int16ConstPtr& msg);

			void setUpdateValues(int size);
			void obtain_target(int value, int buffer);
			void drawRect(int chosen_id, cv::Mat& img);
			void drawTarget(int chosen_id,cv::Mat& img);
			void draw(cv::Mat& img);
			void obtainNewPos(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg);
			bool readyCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			void drawCircle(cv::Mat& img);
		
		

	private:
		//node
		ros::NodeHandle n;
		image_transport::ImageTransport image_transport_; // std::shared_ptr<image....> image_transport_
		ros::ServiceServer service_;
		
		//subscriber
		image_transport::Subscriber sub_image_transport_;
		ros::Subscriber sub_detected_image;
		ros::Subscriber sub_prob;
		ros::Subscriber sub_event_bus;
		
		// topics
		std::string image_topic;
		std::string detections_topic;
		std::string prob_topic;
		std::string event_bus_topic;
		
		//converted image
		cv::Mat opencv_image;
		cv::Mat image_copy;

		//vector of center's positions of each detected apriltags
		std::vector<cv::Point2d> positions_2d;

		//vector of detected apriltag probabilities		
		std::vector<float> probs;
		
		//vector of ids
		std::vector<float> id_pos;

		//event code
		int increasing_code = 500;
		int start_code = 0;
		int picking_code = 1000; //picking code + increasing code = got the object
		int target_code = 5000;
		int continuos_feedback = 781;
		int stop_code = 9999;
		int target_id;
		int picking_id;
		int code;

		float offset = 60;

		//picking mode
		bool picking;

		//home mode
		bool home;

		//clear position vector after picking
		bool needClear;

		//finished flag
		bool finished;

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
		float dim_rect_min = 10.0;
		float dim_rect_max = 80.0;
		float default_circle_radius = 15.0;

		//number of object detected
		int n_objects;

		//dinamic use
		bool continueUpdatePos;
		
		//flag[0] for image, flag[1] for center positions, flag[2] for event
		bool flag[3] = {false,false,false}; 

		bool needTarget;

};
