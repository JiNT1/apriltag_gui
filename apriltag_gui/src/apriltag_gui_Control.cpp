#include "apriltag_gui/apriltag_gui.h"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

/**
*default constructor
*it initializes, before constructor, image_transport_
*/
AprilTagGui::AprilTagGui() : n("~"),image_transport_(n) 
{

	this->info_topic = "/kinect2/hd/camera_info";
	this->image_topic = "/kinect2/hd/image_color";
	this->detections_topic = "/tag_detections";
	this->prob_topic = "/camera_tag_probabilities";
}

/**
*default destructor
*/
AprilTagGui::~AprilTagGui(){}

/**
*it receives published image and converts from ROS message to OpenCV image
*
*@param msg message of sensor_msgs::image type
*/
void AprilTagGui::onReceivedImage(const sensor_msgs::ImageConstPtr& msg)
{

	//std::cout<<"viene chiamato la callback per acquisire le immagini"<<std::endl;
	
	cv_bridge::CvImagePtr cv_ptr;
	// convert ros message image into opencv image
	cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8
	this->opencv_image = cv_ptr->image;
	this->flag[0]=true;
	//cv::imshow("aaaaaaaa",this->opencv_image);
	//cv::waitKey(10);
	//std::cout<<"immagine catturata e convertita"<<std::endl;
};

/**
*get the center positions of each detected apriltag
*
*@param msg message which contains an array of detected apriltags
*/
void AprilTagGui::onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg)
{
	//std::cout<<"viene chiamato la callback per gli apriltag rilevati"<<std::endl;
	//to gain every center of detected apriltags
	if(positions_2d.empty())
	{
		setValues(msg->detections.size());
	}
	for(int i = 0;i < positions_2d.size();i++)
	{
		//this->id_pos[i]=msg->detections[i].id[0];
		//this->positions_2d[id_pos[i]]=cv::Point(msg->detections[i].center_point[0],msg->detections[i].center_point[1]);
		this->positions_2d[msg->detections[i].id[0]] = cv::Point(msg->detections[i].center_point[0],msg->detections[i].center_point[1]);
	}
	
	/*for(int i = 0;i<4;i++){
		std::cout<<msg->detections[i].id[0]<<"   "<<i<<"  "<<positions_2d[i]<<std::endl;
	}*/
	this->flag[1] = true;
	//esempio coordinata x del centro del primo apriltag rilevato
	//std::cout<<this->positions_2d[0]<<std::endl;
}

/**
*get probabilities of each detected apriltags
*
*@param msg array containing ids and their corrisponding proabilities
*/
void AprilTagGui::onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg)
{

	//std::cout<<"viene chiamato la callback per le probabilità con i corrispettivi id"<<std::endl;
	for(int i = 0,j = 0;i<msg->data.size() && j<this->probs.size();i+=2,j++)
	{
		//std::cout<<i<<std::endl;
		this->id_probs[j] = msg->data[i];
		this->probs[j] = msg->data[i+1];
	}
	this->flag[2] = true;
	//std::cout<<"pxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0 :"<<this->probs[0]<<std::endl;
}

/**
*configure every subscribes
*
*@return true if every subscribe succeed, false otherwiser
*/
bool AprilTagGui::setup(){

	//bool check = true;
	std::cout<<"inizio setup"<<std::endl;
	
	//listening on image_color topic
	this->sub_image_transport_ = this->image_transport_.subscribe(this->image_topic,1,&AprilTagGui::onReceivedImage,this);
	if(sub_image_transport_ == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->image_topic);
		//std::cout<<"error in subscribing to "<<this->image_topic<<std::endl;
		//check = false;
	}
		
	//listening on tag_detections topic
	this->sub_detected_image = this->n.subscribe(this->detections_topic,1,&AprilTagGui::onReceivedDetectedImage,this);
	if(sub_detected_image == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->detections_topic);
		//std::cout<<"error in subscribing to "<<this->detections_topic<<std::endl;
		//check = false;
	}
	
	//listening on camera_tag_probabilities topic
	this->sub_prob = this->n.subscribe(this->prob_topic,1,&AprilTagGui::onReceivedProb,this);
	if(sub_prob == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->prob_topic);
		//std::cout<<"error in subscribing to "<<this->prob_topic<<std::endl;
		//check = false;
	}
	
	std::cout<<"setup finished"<<std::endl;
	return true;
}
/**
*drawing a rectangle on the image at the center of each apriltags
*
*@return a marked image where the most probable apriltag is highlighted with green and big rectangle and other apriltags are highlighted with red and small rectangle based on their probabilities 
*/
cv::Mat AprilTagGui::drawRect(){

	//OpenCV color channel order is B,G,R
	//green = cv::Scalar(0,255,0)
	//red = cv::Scalar(0,0,255)
	//thickness = 3
	//markerSize measuring in pixel
	
	//this->opencv_image = cv::imread(cv::samples::findFile("/home/jintommaso/jin_ws/src/apriltag_gui/src/test.jpeg"),cv::IMREAD_COLOR);
	// cv::drawMarker(Mat& img,Point position,const Scalar& color,int markerType,int markerSize,int thinckness,int line_type);
	// markedSize depends on probablities caught by function OnReceviedProb()
	//cv::drawMarker(marked_img,this->positions_2d[i],cv::Scalar(0,255,0),cv::MARKER_SQUARE,std::round(this->probs[i]),3,8);
	//return this->opencv_image;
	



	cv::Mat marked_img;
	std::vector<int>::iterator it;
	
	if(this->flag[0] == true)
	{
		this->opencv_image.copyTo(marked_img);
		//finding the max probability, the apriltag with this probability will have green rectangle
		if(this->flag[1] == true && this->flag[2] == true)
		{
			float max = *std::max_element(this->probs.begin(),this->probs.end());
			//std::cout<<max<<std::endl; 
			//marking every apriltags
			//std::cout<<positions_2d[0]<<std::endl; /*TODO: posizione dei centri varia nbel tempo come mai ???????????????????????*/
			for(int i=0;i<this->positions_2d.size() && i<this->probs.size();i++){
				//in the array every id is unique, it finds the id based on i
				it = find(this->id_probs.begin(),this->id_probs.end(),i);
				//std::cout<<id_probs[i]<<"   "<<probs[i]<<std::endl;
				//std::cout<<probs[*it]<<std::endl; 
				//std::cout<<*it<<std::endl;
				//not at the end, so it finds the id, otherwise it does not find the id 
				if(it != this->id_probs.end())
				{
					//center positions' array is always order, instead probability's array is not order. Choosing the right probability according to the id.
					cv::drawMarker(marked_img,this->positions_2d[i],(this->probs[i] == max ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255)),cv::MARKER_SQUARE,std::round(this->probs[*it]),3,8);
					cv::imshow("xxxxxxxxxxxxxxxxx",marked_img);
					cv::waitKey(10);
				}
				else
					std::cout<<"error finding the right apriltag"<<std::endl;
			}
		}
	}else
		std::cout<< "not received image "<<std::endl; 
		
	

	return marked_img;
}

/**
* get the published unmarked image
*
*@return image
*/
cv::Mat AprilTagGui::getImage()
{

	return this->opencv_image;
}

void AprilTagGui::setValues(int size)
{
	for(int i=0;i<size;i++){
		this->positions_2d.push_back(cv::Point2d(0,0));
		this->id_probs.push_back(0);
		this->id_pos.push_back(0);
		this->probs.push_back(0);
	}
}

/*
bool AprilTagGui::setup(){
	std::cout<<"inizio setup"<<std::endl;
	this->sub_image.subscribe(this->n,this->image_topic,1,ros::TransportHints().tcpNoDelay());
	this->sub_detected_image.subscribe(this->n,this->detections_topic,1,ros::TransportHints().tcpNoDelay());
	this->sub_prob.subscribe(this->n,this->prob_topic,1,ros::TransportHints().tcpNoDelay());
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,apriltag_arm_ros::AprilTagDetectionArray,std_msgs::Float32MultiArray> syncPolicy;
	message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10),this->sub_image,this->sub_detected_image,this->sub_prob);
	sync.registerCallback(boost::bind(&AprilTagGui::onReceivedData,_1,_2));
	std::cout<<"okkkkkkkkkkkkkkkkkkkkkkk"<<std::endl;
	return true;
}
void AprilTagGui::onReceivedData(const sensor_msgs::ImageConstPtr& img,const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& detected_array,const std_msgs::Float32MultiArrayConstPtr& probs_array){

	std::cout<<"fattoooooooooooooooooooo"<<std::endl;

}
*/
