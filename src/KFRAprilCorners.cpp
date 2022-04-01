#include "kinect_pkg/KFRAprilCorners.h"
#include "kinect_pkg/KinectFrame.h"

#include <opencv/cv.hpp>

#include <cstring>


KFRAprilCorners::KFRAprilCorners() {}
    
void KFRAprilCorners::receiveFrame(KinectFrame *frame) {
    memcpy(frame->_cvMats["bgr_april"]->data, frame->_cvMats["bgr"]->data, 2160 * 3840 * 3);
    cv::Mat *img = frame->_cvMats["bgr_april"];
    Eigen::MatrixXd detections = frame->_cornerDetections;
    if(detections.cols()){
    //for(int j = 0; j < detections.cols(); j++){
        cv::Point point(detections(0,2), detections(1,2));
        cv::circle(*img, point, 25, cv::Scalar(255, 0, 255), -3);
    }
}