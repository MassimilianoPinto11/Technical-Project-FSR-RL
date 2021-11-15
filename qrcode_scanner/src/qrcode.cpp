#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int32.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "zbar.h"

using namespace cv;
using namespace zbar;

class ROS_IMG_READER {
  ros::NodeHandle _nh;
  ros::Subscriber _ros_img_sub; 
  ros::Publisher  _opencv_image_pub;
  

  int data=0;

public:
  ROS_IMG_READER() {
  _ros_img_sub = _nh.subscribe("/dogbot/camera/image_raw", 1, &ROS_IMG_READER::imageCb, this);
  _opencv_image_pub = _nh.advertise<sensor_msgs::Image>("/opencv/raw_image", 1);



  }

  void QRscan(cv_bridge::CvImagePtr cv_ptr){

      ImageScanner scanner;
      scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); //Config settings
      cv::Mat frame;
      cv::Mat frame_grayscale;

      frame = cv_ptr -> image; //Use a frame from cv messages
      cvtColor(frame,frame_grayscale,CV_BGR2GRAY); //Gray scale conversion

      //Get Dimensions and pointer
      int width = frame_grayscale.cols;
      int height = frame_grayscale.rows;
      uchar *raw = (uchar *)(frame_grayscale.data);

      //Define Image
      Image image(width, height, "Y800", raw, width*height);

      //Scan the Image
      scanner.scan(image);
      
      for(Image::SymbolIterator symbol = image.symbol_begin(); symbol!= image.symbol_end(); ++symbol){
         data = stoi(symbol->get_data());
         if(data== 1 || data == 2 || data ==3){
         std::cout<<"QRCODE IDENTIFICATO: "<< data <<std::endl;
         
         }      
      }

  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;
    cv::Mat img;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            img = cv_ptr->image;
            /* imshow("Display window", img);
            cv::waitKey(10); Wait for a keystroke in the window */
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    QRscan(cv_ptr);
    _opencv_image_pub.publish(cv_ptr->toImageMsg());
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ROS_IMG_READER");
  ROS_IMG_READER ic;
  ros::spin();
  return 0;

}

