#include "apriltag_gui/Gui.h"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

//Costruttore, inizializzo prima della chiamata a costruttore l'image_trasport
Gui::Gui() : n("~"),image_transport_(n) 
{

	this->info_topic = "/hd/camera_info";
	this->image_topic = "kinect2/hd/image_color";
	this->detections_topic = "/tag_detections";
	this->prob_topic = "/camera_tag_probabilities";
}

//distruttore
Gui::~Gui(){}

//riceve l'immagine pubblicata ed effettua la conversione tra immagine ROS a immagine OpenCV
void Gui::onReceivedImage(const sensor_msgs::ImageConstPtr& msg)
{

	std::cout<<"viene chiamato la callback per acquisire le immagini"<<std::endl;
	
	cv_bridge::CvImagePtr cv_ptr;
	// convert ros message image into opencv image
	cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8
	this->opencv_image = cv_ptr->image;
	
	std::cout<<"immagine catturata e convertita"<<std::endl;
};

//ricava le posizioni dei centro degli apriltag 
void Gui::onReceivedDetectedImage(const apriltag_arm_ros::AprilTagDetectionArrayConstPtr& msg)
{
	// viene restituito un array di apriltag ordinati in base all'id
	std::cout<<"viene chiamato la callback per gli apriltag rilevati"<<std::endl;
	//recupero tutti i centri dei apriltag rilevati, grazie al messaggio aggiornato con posizione e punti degli angoli dell'apriltag
	for(int i = 0;i < msg->detections.size();i++){
		this->positions_2d.push_back(cv::Point(msg->detections[i].center_point[0],msg->detections[i].center_point[1]));
	}
	//esempio coordinata x del centro del primo apriltag rilevato
	//std::cout<<this->positions_2d[0].x<<std::endl;
}

//estrae i parametri della telecamera  serve?????
void Gui::onReceivedInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{

	std::cout<<"viene chiamato la callback per le info"<<std::endl;
	
	//R matrice rotazione(in questo caso vettore)
	std::vector<double> R;
	R.insert(R.begin(),std::begin(msg->R),std::end(msg->R)); 
	
	//D vettore parametri di distorsione
	std::vector<double> D;
	D.insert(D.begin(),std::begin(msg->D),std::end(msg->D));
	
	//K matrice parametri intriseci della camera
	std::vector<double> K; 
	K.insert(K.begin(),std::begin(msg->K),std::end(msg->K)); 
	
	//mi serve?????
	this->intrinsic_params.push_back(R);
	this->intrinsic_params.push_back(D);
	this->intrinsic_params.push_back(K);
}

//ottengo le probabilità di ogni apriltag rilevato
void Gui::onReceivedProb(const std_msgs::Float32MultiArrayConstPtr& msg)
{

	std::cout<<"viene chiamato la callback per le probabilità con i corrispettivi"<<std::endl;
	for(int i = 0;i<msg->data.size();i+2)
	{
		this->ids.push_back(msg->data[i]);
	}
	for(int i = 1;i<msg->data.size();i+2)
	{
		this->probs.push_back(msg->data[i]);
	}
}


/*
void Gui::projection(){

	cv::Mat K_matrix = (cv::Mat_<double>(3,3)<< 
						this->intrinsic_params[2][0],
						this->intrinsic_params[2][1],
						this->intrinsic_params[2][2],
						this->intrinsic_params[2][3],
						this->intrinsic_params[2][4],
						this->intrinsic_params[2][5],
						this->intrinsic_params[2][6],
						this->intrinsic_params[2][7],
						this->intrinsic_params[2][8]);
	
	//i paramteri estrinseci da verificare 
	cv::projectPoints(this->positions_3d,this->intrinsic_params[0],0,K_matrix,this->intrinsic_params[1],this->positions_2d);
}
*/

//configuro i vari subscribe
bool Gui::setup(){

	bool check = true;
	std::cout<<"inizio setup"<<std::endl;
	
	//listening sul topic camera_info
	this->sub_info = this->n.subscribe(this->info_topic,1,&Gui::onReceivedInfo,this);
	if(sub_info == 0)
	{
		std::cout<<"error in subscribing to "<<this->info_topic<<std::endl;
		check = false;
	}
	
	//listening sul topic image_color
	this->sub_image_transport_ = this->image_transport_.subscribe(this->image_topic,1,&Gui::onReceivedImage,this);
	if(sub_image_transport_ == 0)
	{
		std::cout<<"error in subscribing to "<<this->image_topic<<std::endl;
		check = false;
	}
		
	//listening sul topic tag_detections
	this->sub_detected_image = this->n.subscribe(this->detections_topic,1,&Gui::onReceivedDetectedImage,this);
	if(sub_detected_image == 0)
	{
		std::cout<<"error in subscribing to "<<this->detections_topic<<std::endl;
		check = false;
	}
	
	//listening sul topic camera_tag_probabilities
	this->sub_prob = this->n.subscribe(this->prob_topic,1,&Gui::onReceivedProb,this);
	if(sub_prob == 0)
	{
		std::cout<<"error in subscribing to "<<this->prob_topic<<std::endl;
		check = false;
	}
	
	std::cout<<"setup terminato"<<std::endl;
	return check;
}

cv::Mat Gui::drawRect(){

	//OpenCV color channel order is B,G,R
	//verde = cv::Scalar(0,255,0)
	//rosso = cv::Scalar(0,0,255)
	//thickness = 3
	//markerSize measuring in pixel
	
	//this->opencv_image = cv::imread(cv::samples::findFile("/home/jintommaso/jin_ws/src/apriltag_gui/src/test.jpeg"),cv::IMREAD_COLOR);
	// cv::drawMarker(Mat& img,Point position,const Scalar& color,int markerType,int markerSize,int thinckness,int line_type);
	// markedSize depends on probablities caught by function OnReceviedProb()
	//cv::drawMarker(marked_img,this->positions_2d[i],cv::Scalar(0,255,0),cv::MARKER_SQUARE,std::round(this->probs[i]),3,8);
	//return this->opencv_image;
	
	cv::Mat marked_img;
	this->opencv_image.copyTo(marked_img);
	std::vector<int>::iterator it;
	
	//trovo la probabilità massima, quella che avrà il segno verde
	int max = *std::max_element(this->probs.begin(),this->probs.end());
	
	//marchio tutti gli apriltag
	for(int i=0;i<this->positions_2d.size() && i<this->probs.size();i++){
		//nell'array ogni id di un apriltag è unico, trovo l'id i corrispondente
		it = find(this->ids.begin(),this->ids.end(),i);
		//non sono alla fine, ho trovato l'id, altrimenti non ho trovato l'id
		if(it != this->ids.end())
			//la posizione dei centri è sempre ordinata, invece quella delle probabilità no. Dunque la scelta della probabilità è effettuata in base all'id.
			cv::drawMarker(marked_img,this->positions_2d[i],(this->probs[i] == max ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255)),cv::MARKER_SQUARE,std::round(this->probs[*it]),3,8);
		else
			std::cout<<"error finding the right apriltag"<<std::endl;
	}
	return marked_img;
}

cv::Mat Gui::getImage(){

	return this->opencv_image;
}



