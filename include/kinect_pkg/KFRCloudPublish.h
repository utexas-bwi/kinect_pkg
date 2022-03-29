#ifndef KFR_CLOUD_PUBLISH_H
#define KFR_CLOUD_PUBLISH_H

#include <kinect_pkg/KinectFrameRecipient.h>
#include <ros/ros.h>

class KFRCloudPublish : public KinectFrameRecipient {
protected:
    ros::Publisher _pub;

public:
    KFRCloudPublish(ros::NodeHandle &nh);
    ~KFRCloudPublish();

    void receiveFrame(KinectFrame *frame);
};

#endif