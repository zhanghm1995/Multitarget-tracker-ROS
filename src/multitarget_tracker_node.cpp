//C++
#include <string>
#include <iostream>
#include <memory>
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
#include "abstract_target_tracker.h"
using std::string; using std::cout; using std::endl;

//全局变量定义
static params_config ros_params;
class PostProcess
{
public:
  PostProcess(ros::NodeHandle& nodehandle, AbstractTargetTracker* abstract_tracker):nodehandle_(nodehandle),
  processthread_(NULL),
  processthreadfinished_ (false)
//  dnn_tracker_(ros_params)
  {
    target_tracker_.reset(abstract_tracker);
    init();
  }
  ~PostProcess()
  {
    processthreadfinished_ = true;
    processthread_->join();
  }

  void init()
  {
    ROS_INFO("Enter in init function...");
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
    while(!processthreadfinished_ && ros::ok()) {
      if(this->camera_image_raw_.empty()) {
        ROS_WARN_THROTTLE(10, "no camera image!");
      }
//      dnn_tracker_.SetImageInput(this->camera_image_raw_);
//      dnn_tracker_.Process2();
      target_tracker_->SetImageInput(this->camera_image_raw_);
      target_tracker_->Process2();
    }
  }

private:
  ros::NodeHandle& nodehandle_;
  boost::thread* processthread_;
  bool processthreadfinished_;

  //ROS subscriber and publisher
  ros::Subscriber sub_image_;

//  SSDMobileNetTracker dnn_tracker_;
  std::unique_ptr<AbstractTargetTracker> target_tracker_ = nullptr;

  cv::Mat camera_image_raw_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multitarget_tracker_node");
  ros::NodeHandle nh;
  //get parameters
  string show_logs = "1", modelConfiguration, modelBinary, confidenceThreshold, maxCropRatio;
  int exampleNum;
  ros::param::get("example",exampleNum);

  AbstractTargetTracker* abstract_tracker = NULL;
  switch(exampleNum) {
  case 4 :{
    ROS_INFO("exampleNum is 4, using SSDMobileNetTracker!");
    ros::param::get("show_logs",show_logs);
    ros::param::get("modelConfiguration",modelConfiguration);
    ros::param::get("modelBinary",modelBinary);
    ros::param::get("confidenceThreshold",confidenceThreshold);
    ros::param::get("maxCropRatio",maxCropRatio);
    ros_params["show_logs"] = show_logs;
    ros_params["modelConfiguration"] = modelConfiguration;
    ros_params["modelBinary"] = modelBinary;
    ros_params["confidenceThreshold"] = confidenceThreshold;
    ros_params["maxCropRatio"] = maxCropRatio;

    abstract_tracker = new SSDMobileNetTracker(ros_params);
  }
  }

  PostProcess postprocess(nh, abstract_tracker);

  ros::spin();
  return 0;
}
