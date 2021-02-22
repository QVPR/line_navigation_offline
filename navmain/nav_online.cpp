#include "linenav/kimread.h"
#include "navigation.h"
#include "linenav/dispnav.h"
#include "RobotInterface.h"
#include "ImagesOffline.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

ros::Publisher twistPub;

navigation nav;
kimRead teachRunImages;

/* Robot velocities */
double base_forward_velocity;
double max_turning_speed;
double turning_slow_factor;
double turning_threshold;
double image_ratio;

bool initialised = false;
int image_count = 0;

cv::Size image_size;


void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
	cv::Mat image_fullsize = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	cv::Mat image;
	if (image_fullsize.size().width == image_size.width && image_fullsize.size().height == image_size.height){
		image = image_fullsize;
	}
	else{
		cv::resize(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image, image, image_size);
	}

	if(image.channels()==3){
		cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);
	}

	if (!initialised) {
		nav.initlocalisation(image, teachRunImages);
		initialised = true;
		return;
	}

	int flag = nav.step(image);

	if (flag == 0){
		image_count = 0;
	}
	else if (flag == 1){
		image_count++;
	}
	else if (flag < 0){
		// stop!
		geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
		twistPub.publish(cmd_vel);
	}
	else{
		return;
	}

	float v, w;

	w = nav.getRotVel(); // code inside here returns a max turning speed of 0.3
	w *= max_turning_speed / 0.3;

	if(fabs(w) > turning_threshold) 
		v = base_forward_velocity*turning_slow_factor;
	else 
		v = base_forward_velocity;

	if(image_count > 2){
		int islast = nav.SwitchtoNewKeyImages(teachRunImages);
		if(islast){
			std::cout<<"end of topological navigation"<<std::endl;
		}
	}

	geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
	cmd_vel.linear.x = v;
	cmd_vel.angular.z = w;
	twistPub.publish(cmd_vel);
}

int main(int argc, char** argv)
{
	std::cout << "starting" << std::endl;
	ros::init(argc, argv, "line_navigation");
	ros::NodeHandle n("~");
	// couldn't get image transport to work properly
	// image_transport::ImageTransport it(n);
	// image_transport::Subscriber imageSub = it.subscribe("image", 1, imageCallback);
	ros::Subscriber imageSub = n.subscribe("image", 1, imageCallback);
	twistPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	/* Get params */
	n.param("base_forward_velocity", base_forward_velocity, 0.15);
	n.param("max_turning_speed", max_turning_speed, 0.3);
	n.param("turning_slow_factor", turning_slow_factor, 0.3);
	n.param("turning_threshold", turning_threshold, 0.08);
	n.param("image_ratio", image_ratio, 1.0);
	double K11 = n.param("K11", 296.3437428191) * image_ratio;
	double K13 = n.param("K13", 158.2733787287) * image_ratio;
	double K22 = n.param("K22", 296.6441607237) * image_ratio;
	double K23 = n.param("K23", 118.0622191474) * image_ratio;
	image_size = cv::Size(n.param("image_width",320) * image_ratio, n.param("image_height",240) * image_ratio);

	/* Read teach reference images */
	teachRunImages = kimRead(argv[1]);

	/* Setup image display */
	dispNav *d;
	d=new dispNav;
	d->saveimage(false);
	d->displayimage(true);
	d->setDisptime(100);

	/* Setup camera matrix */
	cv::Mat K = (cv::Mat_<double>(3, 3) <<K11, 0, K13, 0, K22, K23, 0, 0, 1);

	/* setup navigation */
	nav = navigation(d, K);

	ros::spin();
}




