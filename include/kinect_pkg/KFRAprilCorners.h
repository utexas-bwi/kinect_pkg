#ifndef KFR_APRIL_CORNERS_H
#define KFR_APRIL_CORNERS_H

#include <kinect_pkg/KinectFrameRecipient.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

class KFRAprilCorners : public KinectFrameRecipient {
protected:
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;

public:
    KFRAprilCorners(ros::NodeHandle &nh);
    ~KFRAprilCorners();
    
    void receiveFrame(KinectFrame *frame);
    
};

#endif