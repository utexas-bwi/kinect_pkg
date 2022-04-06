#include "kinect_pkg/KFRAprilCorners.h"
#include "kinect_pkg/KinectFrame.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.hpp>

#include <cstring>

KFRAprilCorners::KFRAprilCorners(ros::NodeHandle &nh) :
    _it(nh), _pub(_it.advertise("/azure_kinect/bgr_april", 1)) {}

KFRAprilCorners::~KFRAprilCorners() {}
    
void KFRAprilCorners::receiveFrame(KinectFrame *frame) {
    memcpy(frame->_cvMats["bgr_april"]->data, frame->_cvMats["bgr"]->data, 2160 * 3840 * 3);
    cv::Mat *img = frame->_cvMats["bgr_april"];
    Eigen::MatrixXd detections = frame->_cornerDetections;
    //if(detections.cols()){
    for(int j = 0; j < detections.cols(); j++){
        cv::Point point(detections(0,j), detections(1,j));
        cv::circle(*img, point, 25, cv::Scalar(255, 0, 255), -3);
    }

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img).toImageMsg();
    _pub.publish(msg);
}