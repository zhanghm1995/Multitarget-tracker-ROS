//C++
#include <string>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

#include "MouseExample.h"
#include "VideoExample.h"
using std::string;

static params_config ros_params;
const char* keys =
{
    "{ @1             |../data/atrium.avi  | movie file | }"
    "{ e  example     |1                   | number of example 0 - MouseTracking, 1 - MotionDetector, 2 - FaceDetector, 3 - PedestrianDetector, 4 - MobileNet SSD detector, 5 - Yolo detector | }"
    "{ sf start_frame |0                   | Start a video from this position | }"
    "{ ef end_frame   |0                   | Play a video to this position (if 0 then played to the end of file) | }"
    "{ ed end_delay   |0                   | Delay in milliseconds after video ending | }"
    "{ o  out         |                    | Name of result video file | }"
    "{ sl show_logs   |1                   | Show Trackers logs | }"
    "{ g gpu          |0                   | Use OpenCL acceleration | }"
};


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
  cv::CommandLineParser parser(argc, argv, keys);
  //get parameters

  string show_logs;
  int exampleNum;
  ros::param::get("~example",exampleNum);
  ros::param::get("~show_logs",show_logs);
  ros_params["show_logs"] = show_logs;

//  bool useOCL = parser.get<int>("gpu") ? 1 : 0;//zhanghm: not use OpenCL by default
//  cv::ocl::setUseOpenCL(useOCL);
//  std::cout << (cv::ocl::useOpenCL() ? "OpenCL is enabled" : "OpenCL not used") << std::endl;


  ros::spin();
  return 0;
}
