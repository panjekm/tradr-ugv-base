#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

#include "omnicamera_msgs/GetVirtualCameraConfig.h"
#include "omnicamera_msgs/VirtualCameraConfig.h"

namespace omnicamera {
class Whitebalancing : public nodelet::Nodelet{
public:
    Whitebalancing(){}
    
private:
 
boost::shared_ptr<image_transport::ImageTransport> it;
image_transport::Subscriber sub; 
int white_bal_BU;
int white_bal_RU;
int white_bal_BU_old;
int white_bal_RU_old;
int count;

double vertical_startposition;
double vertical_endposition;
double horizontal_startposition;
double horizontal_endposition;

double cor_blue;
double cor_red;

cv::Vec3i mean_col;

void onInit() {
         NODELET_INFO("Auto white balancing initialising");
         
         ros::NodeHandle &nh = getNodeHandle();
         it.reset(new image_transport::ImageTransport(nh));
         sub = it->subscribe("in", 1, &Whitebalancing::ballancing, this);
         
         // The positions of the reference area relative to the image size
         nh.param("vertical_startposition", vertical_startposition, 0.5);
         nh.param("vertical_endposition", vertical_endposition, 0.65);
         nh.param("horizontal_startposition", horizontal_startposition, 0.94);
         nh.param("horizontal_endposition", horizontal_endposition, 1.0);
         // only read the image every 10 published images
         nh.param("wait_images",count,10);
         
         nh.param("cor_blue",cor_blue,1.0);
         nh.param("cor_red",cor_red,1.0);
         
         // intial white balancing settings
         white_bal_BU=700;
         white_bal_RU=512;
         
         // Using system calls for dynamic reconfigure. 
         system("rosrun dynamic_reconfigure dynparam set /viz/camera1394_driver white_balance_BU 700");
         sleep(0.5);
         system("rosrun dynamic_reconfigure dynparam set /viz/camera1394_driver white_balance_RV 512");
         sleep(0.5);
         
         NODELET_INFO("Auto white balancing started");
}

void ballancing(const sensor_msgs::ImageConstPtr& msg){ //callback method
  if (count<10){
    count++;
    return;
  }
  count =0;
  cv_bridge::CvImagePtr cv_msg;
  cv::Mat* im;
  cv_msg= cv_bridge::toCvCopy( msg, std::string());
  im=&(cv_msg->image);
  if ((*im).empty()){
    ROS_INFO("Image message is empty, can not perform auto white balancing");
  }
  else {
    cv::Vec3i mean_col=getMeanColour(im,true);//(bgr)
    
    // calculation of the parameters for white balance adaption
    float b_c=mean_col(1)*cor_blue/(mean_col(0));
    float r_c=mean_col(1)*cor_red/(mean_col(2));
    
    b_c=1.0+((b_c-1)/2.0);
    r_c=1.0+((r_c-1)/2.0);
    
    //if the difference is small, do not change the parameters
        if(white_bal_BU*b_c-white_bal_BU>100){
            b_c=(white_bal_BU+100)/(double)(white_bal_BU);
        }
        if(white_bal_RU*r_c-white_bal_RU>100){
            r_c=(white_bal_RU+100)/(double)(white_bal_RU);
        }
        if(white_bal_BU-white_bal_BU*b_c>100){
            b_c=(white_bal_BU-100)/(double)(white_bal_BU);
        }
        if(white_bal_RU-white_bal_BU*r_c>100){
            r_c=(white_bal_RU-100)/(double)(white_bal_RU);
        }
        white_bal_BU=(int)white_bal_BU*b_c;
        white_bal_RU=(int)white_bal_RU*r_c;
        
    if (white_bal_BU>1024)
        white_bal_BU=1024;
    if (white_bal_RU>1024)
        white_bal_RU=1024;
    
    std::stringstream ss;
    ss << white_bal_BU;
    
    // Using system calls to change the parameters of the camera. Is there a better way using c++ ?
    system(("rosrun dynamic_reconfigure dynparam set /viz/camera1394_driver white_balance_BU "+ss.str()).c_str());
//    ROS_INFO("changing BU");
//    std::cout <<white_bal_BU<<"\n"<<white_bal_RU<<"\n"<<b_c<<"\n"<<r_c<<"\n"<<mean_col(1);
    
    sleep(0.5);
    
    ss.clear();
    ss.str(std::string());
    ss << white_bal_RU;
    
    system(("rosrun dynamic_reconfigure dynparam set /viz/camera1394_driver white_balance_RV "+ss.str()).c_str());
    
    sleep(0.5);
}

}
cv::Vec3i getMeanColour(cv::Mat* im,bool sat_ign){ 
    cv::Vec3i mean_col;
    cv::Vec3i col;
    
    int rows=im->rows;
    int cols=im->cols;
    
    double temp;
    
    // Vector initialisation
    int S=0;
    for (int i=0;i<3;i++){
        mean_col(i)=0;
        }
    for(int i=vertical_startposition*rows;i<vertical_endposition*rows;i++){       
         for(int j=horizontal_startposition*cols;j<horizontal_endposition*cols;j++){
            col =(*im).at<cv::Vec3b>(i,j); 
            //if (col(1)<253 && col(2)<253 && col(3)<253){ // ignore too bright pixels, flare spots, gives problems
            mean_col=mean_col+col;
            S++;
            //}
         }
    }
    for (int k =0;k<3;k++){
        temp=((double)mean_col(k))/S;
        mean_col(k)=(int)temp;
    }
    return mean_col;
}

}; //End class Whitebalancing

} // ENd namespace

PLUGINLIB_DECLARE_CLASS(omnicamera, omnicam_whitebalance, omnicamera::Whitebalancing, nodelet::Nodelet)

