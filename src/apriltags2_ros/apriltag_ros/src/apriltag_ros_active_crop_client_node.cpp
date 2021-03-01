#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

static const std::string OPENCV_WINDOW = "Image window";
std::vector<geometry_msgs::Point> corner_points;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
public:
  ImageConverter()
    : it_(nh_){
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

      // int array_length = 10;
      // if(array_length >=3){
      //    for (int i = 0; i < array_length-1; i++)
      //    {
      //       line(cv_ptr->image,cv::Point(corner_points[i].x,corner_points[i].y),cv::Point(corner_points[i+1].x,corner_points[i+1].y),cv::Scalar(0,0xff,0));
      //    }
      //    line(cv_ptr->image,cv::Point(corner_points[0].x,corner_points[0].y),cv::Point(corner_points[array_length-1].x,corner_points[array_length-1].y),cv::Scalar(0,0xff,0));
      // }

      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    
  }

};

  void callback(const apriltag_ros::AprilTagDetectionArray msg){
   int length_array = msg.detections.size();
   for (int i = 0; i < msg.detections.size(); i++)
   {  
      if ( msg.detections[i].id[0] <= 10 )
      {
         geometry_msgs::Point pushbackpoint;
         pushbackpoint = msg.detections[i].position_in_image;
         pushbackpoint.z = msg.detections[i].id[0];
         corner_points[msg.detections[i].id[0]].x = msg.detections[i].position_in_image.x;
         corner_points[msg.detections[i].id[0]].y = msg.detections[i].position_in_image.y;
         corner_points[msg.detections[i].id[0]].z = 1.0;
      }
   }
   }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltags_ros_active_crop_client");
  ros::NodeHandle nh;
  // ros::MultiThreadedSpinner test(2);
  ImageConverter ic;
  ros::Subscriber sub_tags = nh.subscribe("tag_detections",1,callback);
  ros::Subscriber sub = nh.subscribe("tag_detections_image",1,&ImageConverter::imageCb,&ic);
  // test.spin();
  ros::spin();
  return 0;
}



// #include <ros/ros.h>
// #include <apriltag_ros/AprilTagDetectionArray.h>
// #include <geometry_msgs/Point.h>
// #include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <ros/console.h>
// #include <vector>

// std::vector<geometry_msgs::Point> corner_points;


// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//   try
//   {
//     cv::imshow("view", cv_bridge::toCvShare(msg, "rgb8")->image);
//     cv::waitKey(30);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
// }


// void callback_images(const sensor_msgs::ImageConstPtr msg){
//    try
//   {
//       cv::Mat temp_image = cv_bridge::toCvShare(msg, "bgr8")->image;
//       // int array_length = corner_points.size();
//       // if(array_length >=3){
//       //    for (int i = 0; i < array_length-1; i++)
//       //    {
//       //       line(temp_image,cv::Point(corner_points[i].x,corner_points[i].y),cv::Point(corner_points[i+1].x,corner_points[i+1].y),cv::Scalar(0,0xff,0));
//       //    }
//       //    line(temp_image,cv::Point(corner_points[0].x,corner_points[0].y),cv::Point(corner_points[array_length-1].x,corner_points[array_length-1].y),cv::Scalar(0,0xff,0));
//       //    // line(temp_image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
//       //    // cv::Point((int)det->p[1][0], (int)det->p[1][1]),
//       //    // cv::Scalar(0, 0xff, 0)); // green
//       // }
      

//       cv::imshow("view", temp_image);
//       cv::waitKey(0);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
// }

// void callback(const apriltag_ros::AprilTagDetectionArray msg){
//    int length_array = msg.detections.size();
//    for (int i = 0; i <= msg.detections.size(); i++)
//    {  
//       if ( msg.detections[i].id[0] <= 10 )
//       {
//          geometry_msgs::Point pushbackpoint;
//          pushbackpoint = msg.detections[i].position_in_image;
//          pushbackpoint.z = msg.detections[i].id[0];
//          corner_points.push_back(pushbackpoint);
//       }
//    }
// }

// int main(int argc, char** argv) {
//     ros::init(argc,argv,"apriltags_ros_active_crop_client");
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");
//     image_transport::ImageTransport it(nh);
//     ros::Subscriber sub = nh.subscribe("tag_detections",1,callback);
//     image_transport::Subscriber sub_image = it.subscribe("/tag_detections_image",10,imageCallback);
//     cv::namedWindow("view");
//     cv::startWindowThread();
//     ros::spin();
//     cv::destroyWindow("view");
//     }
