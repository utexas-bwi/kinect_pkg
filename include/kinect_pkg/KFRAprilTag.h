#ifndef KFR_APRIL_TAG_H
#define KFR_APRIL_TAG_H

#include <kinect_pkg/KinectFrameRecipient.h>

#include <apriltag/apriltag.h>

#include <Eigen/Dense>
#include <opencv/cv.hpp>


class KFRAprilTag : public KinectFrameRecipient {
public:
    KFRAprilTag();
    
    void receiveFrame(KinectFrame *frame);
    
protected:
    Eigen::MatrixXd detectCorners(cv::Mat &img);
    apriltag_detector_t *td;
    apriltag_family_t *tf;
};

#endif