#include "apriltag_gui/apriltag_gui.h"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

/**
*default constructor
*it initializes, before constructor, image_transport_
*/
AprilTagGui::AprilTagGui() : n("~"),image_transport_(n) 
{
	this->image_topic = "/kinect2/hd/image_color";
	this->detections_topic = "/tag_detections";
	this->prob_topic = "/camera_tag_probabilities";
	this->event_bus_topic = "/events/bus";
	this->picking = false;
	this->continueUpdatePos = false;
	this->home = false;
	this->needClear = false;
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
	//std::cout<<"immagine catturata e convertita"<<std::endl;
	//cv::imshow("esempio",this->opencv_image);
	//cv::waitKey(1);
};

/**
*set id_pos, position and probability vector
*
*@param msg message which contains an array of detected apriltags
*/
void AprilTagGui::onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg)
{
	//std::cout<<"viene chiamato la callback per gli apriltag rilevati"<<std::endl;
	//to gain every center of detected apriltags
	if(this->positions_2d.empty())// || msg->detections.size()!=this->n_objects)
	{
		//std::cout << "Detected size: " << msg->detections.size() << "Position2d size: " << this->positions_2d.size() << std::endl;
		setUpdateValues(msg->detections.size());
		for(int i=0;i<this->n_objects;i++)
		{
			this->id_pos[i] = msg->detections[i].id[0];
		}
		//std::cout << "New Detected size: " << msg->detections.size() << "New Position2d size: " << this->positions_2d.size() << std::endl;
		obtainNewPos(msg);
		return;
	}
	if(this->continueUpdatePos == true) //funzione per la modalità dinamica 
	{
		obtainNewPos(msg);
	}
	/*
	if(this->positions_2d.empty())
	{
		std::cout << "Detected size: " << msg->detections.size() << "Position2d size: " << this->positions_2d.size() << std::endl;
		setValues(msg->detections.size());
		for(int i=0;i<this->n_objects;i++)
		{
			this->id_pos[i] = msg->detections[i].id[0];
		}
		std::cout << "New Detected size: " << msg->detections.size() << "New Position2d size: " << this->positions_2d.size() << std::endl;
	}
	if(msg->detections.size() != this->n_objects)
	{
		updateValues(msg->detections.size());
		for(int i=0;i<this->n_objects;i++)
		{
			this->id_pos[i] = msg->detections[i].id[0];
		}
	}
	*/
	this->flag[1] = true;
}

/**
*fullfilled positions vector with cenetr of each detected apriltag
*
*@param msg message which contains an array of detected apriltags
*/
void AprilTagGui::obtainNewPos(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg)
{
	for(int i=0;i < this->n_objects;i++)
	{
		for(int j=0;j<this->n_objects;j++)
		{
			if(msg->detections[i].id[0] == this->id_pos[j])
			{
				this->positions_2d[j] = cv::Point(msg->detections[i].center_point[0],msg->detections[i].center_point[1]);
			}
		}
		
		//this->positions_2d[msg->detections[i].id[0]] = cv::Point(msg->detections[i].center_point[0],msg->detections[i].center_point[1]);
	}
}

/**
*get probabilities of each detected apriltags
*
*@param msg array containing ids and their corrisponding proabilities
*/
void AprilTagGui::onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg)
{

	//std::cout<<"viene chiamato la callback per le probabilità con i corrispettivi id"<<std::endl;
	//probabilities without apriltag positions is useless, so it gets probability after getting positions
	if(this->flag[1] == true)
	{
		for(int i=0;i<msg->data.size();i+=2)
		{
			for(int j=0;j<this->n_objects;j++)
			{
				if(msg->data[i] == this->id_pos[j])
				{
					this->probs[j] = msg->data[i+1];
				}
			}
		}
	}
}

/**
*get the code to command the graphic user interface
*
*@param msg message that contains a code
*/
void AprilTagGui::onReceivedEvent(const rosneuro_msgs::NeuroEventConstPtr& msg)
{

	this->code=msg->event;
	this->flag[2] = true;
}


bool AprilTagGui::readyCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{
	return true;
}

/**
*configure every subscribes
*
*@return true if every subscribe succeed, false otherwiser
*/
bool AprilTagGui::setup()
{

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

	//listening on event/bus topic
	this->sub_event_bus = this->n.subscribe(this->event_bus_topic,1,&AprilTagGui::onReceivedEvent,this);
	if(sub_event_bus == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->event_bus_topic);
	}

	this->service_ = this->n.advertiseService<std_srvs::Empty>("gui_ready", AprilTagGui::readyCallback);



	std::cout<<"setup finished"<<std::endl;
	return true;
}

/**
*drawing a rectangle on the image at the center of each apriltags
*
*@param chosen_id chosen_id is used on drawing the rectangle on the position defined by id
*/
void AprilTagGui::drawRect(int chosen_id)
{

	cv::Scalar color;
	//finding the max probability, the apriltag with this probability will have green rectangle
	//auto min_max = std::minmax_element(this->probs.begin(),this->probs.end());
	//-------------------------------------------------------------------------

	//std::cout<< "i:" <<this->id_pos[chosen_id] << "	pos:"<<this->positions_2d[chosen_id] << "	prob:"<<this->probs[chosen_id]<<std::endl;
	if(this->home == true)
	{
		//std::cout<< "i:" <<this->id_pos[chosen_id] << "	pos:"<<this->positions_2d[chosen_id] << "	prob:"<<this->probs[chosen_id]<<std::endl;
		cv::drawMarker(this->opencv_image,this->positions_2d[chosen_id],this->other_color,cv::MARKER_SQUARE,this->default_dim_rect*(this->probs[chosen_id]),this->thickness);
	}
	else if(this->picking == true)
	{
		//std::cout<< "i:" <<this->id_pos[chosen_id] << "	pos:"<<this->positions_2d[chosen_id] << "	prob:"<<this->probs[chosen_id]<<std::endl;
		if(chosen_id == this->target_id)
			color = this->correct_color;
		else
			color = this->wrong_color; 
		//auto it = std::find(this->id_pos.begin(),this->id_pos.end(),float(i));
		for(int i=0;i<this->n_objects;i++)
		{
			if(chosen_id == id_pos[i])
				cv::drawMarker(this->opencv_image,this->positions_2d[i],color,cv::MARKER_SQUARE,this->default_dim_rect*(this->probs[i]),this->thickness);
		}
	}
	else
	{
		//std::cout<< "i:" <<this->id_pos[chosen_id] << "	pos:"<<this->positions_2d[chosen_id] << "	prob:"<<this->probs[chosen_id]<<std::endl;
		if(this->id_pos[chosen_id] == this->target_id)
			color = this->target_color;
		else
			color = this->other_color;
		cv::drawMarker(this->opencv_image,this->positions_2d[chosen_id],color,cv::MARKER_SQUARE,this->default_dim_rect*(this->probs[chosen_id]),this->thickness);
	}
	//cv::drawMarker(marked_img,this->positions_2d[i],color,cv::MARKER_SQUARE,this->default_dim_rect*(this->probs[i]),3);
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

/**
*set or update position,probability and id vector
*
*@param size vector's size
*@param isSet decides if it need to set probability 
*/
void AprilTagGui::setUpdateValues(int size)
{
	this->n_objects = size;
	std::vector<cv::Point2d> temp_pos(size,cv::Point2d(0,0));
	std::vector<float> temp_id_pos(size,0);
	this->positions_2d.assign(temp_pos.begin(),temp_pos.end());
	this->id_pos.assign(temp_id_pos.begin(),temp_id_pos.end());
	std::vector<float> temp_prob(size,1/float(size));
	this->probs.assign(temp_prob.begin(),temp_prob.end());
	//this->id_probs.assign(temp_id_pos.begin(),temp_id_pos.end());
}

/**
*throught the received code, getting the id
*
*@param value the received code
*@param buffer default code used to recognize the right id
*/
void AprilTagGui::obtain_target(int value,int buffer)
{
	if(buffer == this->target_code)
		this->target_id = value - buffer;
	if(buffer == this->picking_code)
		this->picking_id = value -buffer;
}

/**
*draw numObjs rectangles
*
*@param numObjs number of drawing
*/
void AprilTagGui::drawAndShow(int numObjs)
{
		if(numObjs == 1)
		{
			drawRect(this->picking_id);
		}else
		{
			//marking every apriltags
			for(int i=0;i<numObjs;i++)
			{
				drawRect(i);
			}
		}
	std::cout<<"-------------------------------------------------------"<<std::endl;
	cv::imshow("esempio",this->opencv_image);
	cv::waitKey(1);
}
/*
float AprilTagGui::markedSize(float value, float max, float min)
{
	float minOutRange = 20;
	float maxOutRange = 80;
	float minInRange = min ;
	float maxInRange = max;
	if(value == max && value == min)
	{
		return 40.0;
	}
	return minOutRange + (value - minInRange) * (maxOutRange - minOutRange) / (maxInRange - minInRange);
}
*/

/**
*it begins to execute according to the received event
* 
*/
void AprilTagGui::start()
{
	if(flag[0] == true)
	{
		if(flag[2] == true && flag[1] == true)
		{
			if(this->code ==  this->start_code)
			{
				this->picking = false;
				this->home = true;
				//only after a picking, it can return to home positions
				if(this->needClear == true)
				{
					this->positions_2d.clear();
					//for(int i=0;i<this->probs.size();i++)
					//{
					//	this->probs.pop_back();
					//}
					this->needClear = false;
				}
				drawAndShow(this->n_objects);
			}

			if(this->code >= this->target_code && this->code < (this->target_code + this->increasing_code))
			{
				this->picking = false;
				this->home = false;
				this->needClear = false;
				obtain_target(this->code,this->target_code);
				drawAndShow(this->n_objects);
			}

			if(this->code >= this->picking_code && this->code < (this->picking_code + this->increasing_code))
			{
				this->picking = true;
				this->home = false;
				this->needClear = true;
				obtain_target(this->code,this->picking_code);
				int onlyOne = 1; //just one element
				drawAndShow(onlyOne);
			}
			
		}
		cv::imshow("esempio",this->opencv_image);
		cv::waitKey(1);

	}


}
