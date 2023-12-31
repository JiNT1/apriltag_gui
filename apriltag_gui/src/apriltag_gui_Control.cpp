#include "apriltag_gui/apriltag_gui.h"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

/**
*default constructor
*it initializes, before constructor, image_transport_
*/
AprilTagGui::AprilTagGui() : n("~"),image_transport_(n) 
{
	
	this->image_topic = "/bci_camera/hd/image_color";
	this->detections_topic = "/bci_camera/tag_detections";
	//this->image_topic = "/kinect2/hd/image_color";
	//this->detections_topic = "/tag_detections";
	this->prob_topic = "/camera_tag_probabilities";
	this->event_bus_topic = "/events/bus";
	this->picking = false;
	this->continueUpdatePos = false;
	this->home = false;
	this->needClear = false;
	this->finished = false;
	this->needTarget = false;
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
	this->opencv_image.copyTo(this->image_copy);
}

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
		this->flag[1] = true;
		return;
	}
	if(this->continueUpdatePos == true) //funzione per la modalità dinamica 
	{
		obtainNewPos(msg);
		this->flag[1] = true;
	}
	
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
	//std::cout<<"ho ricevuto il comando"<<std::endl;
	this->code=msg->event;
	this->flag[2] = true;
}

/*
void AprilTagGui::onReceivedEvent(const std_msgs::Int16ConstPtr& msg)
{

	//std::cout<<"ho ricevuto il comando"<<std::endl;
	this->code=msg->data;
	this->flag[2] = true;
}
*/

/**
*service to notice client that all subscribes are succeed
* 
*@return true when service is created
*/
bool AprilTagGui::readyCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{

	//std::cout<<"sono nella service"<<std::endl;
	return true;
}

/**
*configure every subscribes
*
*@return true if every subscribe succeed, false otherwiser
*/
bool AprilTagGui::setup()
{

	//std::cout<<"inizio setup"<<std::endl;
	
	//listening on image_color topic
	this->sub_image_transport_ = this->image_transport_.subscribe(this->image_topic,1,&AprilTagGui::onReceivedImage,this);
	if(sub_image_transport_ == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->image_topic);
	}
		
	//listening on tag_detections topic
	this->sub_detected_image = this->n.subscribe(this->detections_topic,1,&AprilTagGui::onReceivedDetectedImage,this);
	if(sub_detected_image == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->detections_topic);
	}
	
	//listening on camera_tag_probabilities topic
	this->sub_prob = this->n.subscribe(this->prob_topic,1,&AprilTagGui::onReceivedProb,this);
	if(sub_prob == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->prob_topic);
	}

	//listening on events/bus topic
	this->sub_event_bus = this->n.subscribe(this->event_bus_topic,1,&AprilTagGui::onReceivedEvent,this);
	if(sub_event_bus == 0)
	{
		ROS_ERROR("error in subscribing to %s",this->event_bus_topic);
	}

	this->service_ = this->n.advertiseService("/gui_ready", &AprilTagGui::readyCallback,this);
	
	//std::cout<<"setup finished"<<std::endl;
	return true;
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
void AprilTagGui::draw(cv::Mat& img)
{
	//this->opencv_image.copyTo(this->image_copy);
    cv::Scalar color;

	if(this->picking)
	{
		if(this->picking_id == this->target_id)
			color = this->correct_color;
		else
			color = this->wrong_color; 
		for(int i=0;i<this->n_objects;i++)
		{
			if(this->picking_id == this->id_pos[i])
				cv::drawMarker(img,this->positions_2d[i],color,cv::MARKER_SQUARE,this->dim_rect_min+(this->probs[i]*(this->dim_rect_max-this->dim_rect_min)),this->thickness);
		}
	}else
	{
		//marking every apriltags
		for(int i=0;i<this->n_objects;i++)
		{
			color = this->other_color;
		    cv::drawMarker(img, this->positions_2d[i],this->other_color,cv::MARKER_SQUARE,this->dim_rect_min+(this->probs[i]*(this->dim_rect_max-this->dim_rect_min)),this->thickness);
			drawCircle(img);
		}
	}

}

void AprilTagGui::drawCircle(cv::Mat& img)
{

	for(int i=0;i<this->n_objects;i++)
	{
		if(this->target_id == this->id_pos[i])
			cv::circle(img,cv::Point2d(this->positions_2d[i].x,this->positions_2d[i].y-this->offset),this->default_circle_radius,this->target_color,cv::FILLED);
	}


}

/**
*get if the system is finished or not
*
* @return true if the system is finished, otherwise false
*/
bool AprilTagGui::stop()
{

	return this->finished;
}

/**
*it begins to execute according to the received event
*/

void AprilTagGui::start()
{

	if(this->flag[0])
	{
		if(this->flag[1] && this->flag[2])
		{
			// caso 0 non più utile
			/*
			if(this->code ==  this->start_code)
			{
				this->picking = false;
				//this->home = true;
				//this->needClear = false;
				//only after a picking, it can return to home positions
				if(this->needClear)
				{
					this->positions_2d.clear();
					this->probs.clear();
					this->needClear = false;
					this->flag[1] = false;
				}

				//drawAndShow(this->n_objects);

				draw(this->needTarget,this->image_copy);

			}
			*/
			if(this->code >= this->target_code && this->code < (this->target_code + this->increasing_code))
			{
				this->picking = false;
				this->needClear = false;
				obtain_target(this->code,this->target_code);
				drawCircle(this->image_copy);
			}


			if(this->code == this->continuos_feedback )
			{
				this->picking = false;
				this->needClear = false;
				drawCircle(this->image_copy);
				draw(this->image_copy);

			}


			if(this->code >= this->picking_code && this->code < (this->picking_code + this->increasing_code))
			{
				this->picking = true;
				this->needClear = true;
				obtain_target(this->code,this->picking_code);
				draw(this->image_copy);
			}

			if(this->code == (this->picking_code + this->increasing_code))
			{
				this->picking = false;
				//if(this->needClear)
				//{
					this->probs.clear();
					this->id_pos.clear();
					this->positions_2d.clear();
					this->needClear = false;
					this->flag[1] = false;
				//}
			}

			if(this->code == this->stop_code)
			{
				this->finished = true;
			}	
		}

		//if(this->code != 0){
			cv::imshow("BCI",this->image_copy);
			cv::waitKey(1);
		//}else if(this->code >= this->picking_code && this->code < (this->picking_code + this->increasing_code)){

		//}

		/*
		if(this->code != 0)
		{
			cv::imshow("BCI",this->image_copy);
			cv::waitKey(1);

		cv::imshow("BCI",this->image_copy);
		cv::waitKey(1);
		/*
		if(this->code != 0)
		{
			cv::imshow("BCI",this->image_copy);
			cv::waitKey(1);
		}else
		{
			cv::imshow("BCI",this->opencv_image);
			cv::waitKey(1);		
		}
		*/

	}
}
