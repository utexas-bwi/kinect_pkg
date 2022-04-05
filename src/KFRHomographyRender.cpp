#include <kinect_pkg/KFRHomographyRender.h>
#include <kinect_pkg/KinectFrame.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


KFRHomographyRender::KFRHomographyRender(ros::NodeHandle &nh) :
    _it(nh), _pub(_it.advertise("/azure_kinect/bgr_homography", 1)),
    _modelCorners(Eigen::MatrixXd(3,9)),
    _outCorners(Eigen::MatrixXd(3,9)) {
    int ctr = 0;
    for(int i = -1; i <= 1; i++) {
        for(int j = -1; j <= 1; j++) {
            _modelCorners(0, ctr) = i;
            _modelCorners(1, ctr) = j;
            _modelCorners(2, ctr) = 1;
            ctr++;
        }
    }

}

KFRHomographyRender::~KFRHomographyRender() {}

void KFRHomographyRender::receiveFrame(KinectFrame *frame) {
    cv::Mat *img = frame->_cvMats["bgr_homography"];
    frame->_cvMats["bgr"]->copyTo(*img);
    _outCorners = frame->_homography * _modelCorners;

    if(frame->_tagDetected) {
        for(int i = 0; i < 9; i++) {
            cv::Point point(_outCorners(0,i) / _outCorners(2,i), _outCorners(1,i) / _outCorners(2,i));
            cv::circle(*img, point, 25, cv::Scalar(255, 0, 255), -3);
        }
    }

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img).toImageMsg();
    _pub.publish(msg);
}