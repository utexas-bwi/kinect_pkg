#ifndef KFR_HOMOGRAPHY_RENDER_H
#define KFR_HOMOGRAPHY_RENDER_H

#include <kinect_pkg/KinectFrameRecipient.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <Eigen/Dense>

class KFRHomographyRender : public KinectFrameRecipient {
protected:
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    Eigen::MatrixXd _modelCorners, _outCorners;

public:
    KFRHomographyRender(ros::NodeHandle &nh);
    ~KFRHomographyRender();

    void receiveFrame(KinectFrame *frame);
};

#endif