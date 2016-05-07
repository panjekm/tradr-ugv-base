
#include <iostream>
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

#include "omnicamera_msgs/GetVirtualCameraConfig.h"
#include "omnicamera_msgs/VirtualCameraConfig.h"

#define ARRAY_LENGTH 30

//#define SATURATION_VAL1 250
//#define SATURATION_VAL2 200
//#define vertical_startposition 0.5
//#define vertical_endposition 0.65
//#define horizontal_startposition 0.94
//#define horizontal_endposition 1

namespace omnicamera{

class ColourCorrection : public nodelet::Nodelet{
public:
    ColourCorrection() {
    }
    
private:
ros::NodeHandle n; 
boost::shared_ptr<image_transport::ImageTransport> it;
image_transport::Publisher cameraPub;
image_transport::Subscriber sub;
int iter;
bool full;
float b_list [ARRAY_LENGTH];
float g_list [ARRAY_LENGTH];
float r_list [ARRAY_LENGTH];
double vertical_startposition;
double vertical_endposition;
double horizontal_startposition;
double horizontal_endposition;
int SATURATION_VAL1;
int SATURATION_VAL2;

double corb;
double corg;
double corr;

void onInit(){
    NODELET_INFO("Starting image correction");
    ros::NodeHandle &n = getNodeHandle();
    it.reset(new image_transport::ImageTransport(n));
    sub = it->subscribe("in", 1, &ColourCorrection::correct_col, this);
    cameraPub = it->advertise("out", 1);
    
    iter=-1;
    full=false;
    
    // startposition is a value between 0 and 1, giving the relative position of the first
    // pixel of the reference area. It is multiplied by the number of rows/columns
    n.param("vertical_startposition", vertical_startposition, 0.5);
    n.param("vertical_endposition", vertical_endposition, 0.65);
    n.param("horizontal_startposition", horizontal_startposition, 0.94);
    n.param("horizontal_endposition", horizontal_endposition, 1.0);
    n.param("saturation_value1",SATURATION_VAL1,250);
    n.param("saturation_value2",SATURATION_VAL2,200);
    
    // correction factors for the colour of the wheels ( not perfect grey)
    n.param("cor_b",corb,1.07);
    n.param("cor_g",corg,1.0);
    n.param("cor_r",corr,0.95);
}

// function returns the RGB value of the mean wheel colour
cv::Vec3i getMeanColour(cv::Mat* im,bool sat_ign){ 
    cv::Vec3i mean_col;
    cv::Vec3i col;
   // colour reference area is rectangular
    int rows=im->rows;
    int cols=im->cols;
   
   // Vector initialisation?
    int S=0;
    for (int i=0;i<3;i++){
        mean_col(i)=0;
        }
    for(int i=vertical_startposition*rows;i<vertical_endposition*rows;i++){     
         for(int j=horizontal_startposition*cols;j<horizontal_endposition*cols;j++){
            col =(*im).at<cv::Vec3b>(i,j);
            if ( col(1)<253 && col(2)<253 && col(3)<253){
            mean_col=mean_col+col;
            S++;
            }
         }
    }
    mean_col=mean_col/S;
    return mean_col;
}

void linear_correction(float b_c, float g_c,float r_c,cv::Mat* im){
int i,j;
float cors [3] ={b_c,g_c,r_c};
int temp;
bool sat=false;
for (i=0;i<(*im).rows;i++){
    for(j=0;j<(*im).cols;j++){
        sat=false;
        cv::Vec3b* a =(*im).ptr<cv::Vec3b>(i,j);
        // Some tricks to make sure saturated pixels keep the maximal value
        for (int k=0;k<3;k++){
            if((*a)(k)>(uchar)SATURATION_VAL1)
                sat=true;
        }
        if (sat){
            for(int k=0;k<3;k++){
                if((*a)(k)<(uchar)SATURATION_VAL2){
                    sat=false;
                }
            }
        }
        if (!sat){
            for (int k=0;k<3;k++){
                // changing the RGB value of the pixels
                temp=(*a)(k)*cors[k];
                if(temp<255){
                    (*a)(k)= (uchar) temp;
                }else{
                    (*a)(k)=(uchar)255;
                }
             }
        }
    }
}

}

void correct_col(const sensor_msgs::ImageConstPtr& msg){ //callback method
  cv_bridge::CvImagePtr cv_msg;
  cv::Mat* im;
  cv_msg= cv_bridge::toCvCopy( msg, std::string());
  im=&(cv_msg->image);
 
  if ((*im).empty()){
    ROS_INFO("No Image for colour correction");
  }
  else {
   // cv::namedWindow( "Display window",CV_WINDOW_NORMAL );// Create a window for display, to be removed
    //cv::waitKey(3);
    cv::Vec3i mean_col=getMeanColour(im,true);
    uint grey_col = ((uint)mean_col(0)+(uint)mean_col(1)+(uint)mean_col(2)) /3;

    // calculation of the three colour correction factors
    float b_c= grey_col*corb/(mean_col(0));
    float g_c= grey_col*corg/(mean_col(1));
    float r_c= grey_col*corr/(mean_col(2));
    
    iter++;
    if (iter==ARRAY_LENGTH){
        iter=0;
        full=true;
    }
    b_list[iter]=b_c;
    g_list[iter]=g_c;
    r_list[iter]=r_c;
    b_c=0;
    g_c=0;
    r_c=0;
    int list_end;
    if(full){
        list_end=ARRAY_LENGTH;
    }else{
        list_end=iter+1;
    }
    for(int i=0;i<list_end;i++){
        b_c+=b_list[i];
        g_c+=g_list[i];
        r_c+=r_list[i];
    }
    b_c=b_c/list_end;
    g_c=g_c/list_end;
    r_c=r_c/list_end;
    
    linear_correction(b_c,g_c,r_c,im);
    
    // NODELET_INFO("Image published");
    this->cameraPub.publish(cv_msg->toImageMsg());
  }
}

}; //End class colourCorrection
} // End namespace omnicamera

PLUGINLIB_DECLARE_CLASS(omnicamera,bag_colour_correction, omnicamera::ColourCorrection, nodelet::Nodelet)

