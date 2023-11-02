#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <apriltag_arm_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class Gui{
	public:
		//costruttore
		Gui();
		//distruttore
		~Gui();
		//configura le impostazioni iniziali
		bool setup();
		//ritorna l'immagine acquisita
		cv::Mat getImage();
		//ritorna immagine marcata
		cv::Mat drawRect();/*TODO:trasformare range di probabilità in un valore markedSize, per il momento forse può bastare usare direttamente il valore di probabilità*/
		private:
		//funzioni callback
		void onReceivedImage(const sensor_msgs::ImageConstPtr& msg);/*TODO:serve convertire l'immagine ????????????????????*/
		void onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg);
		void onReceivedInfo(const sensor_msgs::CameraInfoConstPtr& msg);/*TODO:serve????????????????????????????????????????*/
		void onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg);
		

	private:
		ros::NodeHandle n;
		image_transport::ImageTransport image_transport_; // std::shared_ptr<image....> image_transport_
		
		ros::Subscriber sub_info;
		image_transport::Subscriber sub_image_transport_;
		ros::Subscriber sub_detected_image;
		ros::Subscriber sub_prob;
		
		//i topics
		std::string info_topic;
		std::string image_topic;
		std::string detections_topic;
		std::string prob_topic;
		
		//immagine convertita
		cv::Mat opencv_image;
		
		//vettore di posizioni dei centri degli apriltag rilevati
		std::vector<cv::Point2d> positions_2d;
		
		//parametri intrinseci R-D-K in questo ordine serve?
		std::vector<std::vector<double>> intrinsic_params;
				
		//vettore di probabilità degli apriltag rilevati		  
		std::vector<float> probs;
		
		//vettore di id nell'ordine acquisita dal topic
		std::vector<int> ids; 

};
