//C++
#include <string>
//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//Boost
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>


#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#include "VideoExample.h"
using std::string;

static params_config ros_params;
class PostProcess
{
public:
  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
  processthread_(NULL),
  processthreadfinished_ (false),
  dnn_detector_(ros_params)
  {
    init();
  }
  ~PostProcess()
  {
    processthreadfinished_ = true;
    cv::destroyAllWindows();
    processthread_->join();
  }

  void init()
  {
    sub_image_ = nodehandle_.subscribe<sensor_msgs::Image>("/frontal_camera/image",1, &PostProcess::image_callback, this);
    //begin main thread process
    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    this->camera_image_raw_ = cv_ptr->image;
  }

  void process()
  {
    dnn_detector_.Process();
  }

private:
  ros::NodeHandle& nodehandle_;
  boost::thread* processthread_;
  bool processthreadfinished_;

  //ROS subscriber and publisher
  ros::Subscriber sub_image_;

  SSDMobileNetExample dnn_detector_;

  cv::Mat camera_image_raw_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multitarget_tracker");
  ros::NodeHandle nh;
  PostProcess postprocess(nh);

  //get parameters
  string show_logs;
  int exampleNum;
  ros::param::get("~example",exampleNum);
  ros::param::get("~show_logs",show_logs);
  ros_params["show_logs"] = show_logs;

  ros::spin();
  return 0;
}
