#ifndef KFR_POSE_PUBLISH_H
#define KFR_POSE_PUBLISH_H

#include "kinect_pkg/KinectFrameRecipient.h"
#include <ros/ros.h>

#include <vector>

class KFRPosePublish : public KinectFrameRecipient {
protected:
    ros::Publisher _pub;

public:
    KFRPosePublish(ros::NodeHandle &nh);
    ~KFRPosePublish();

    void receiveFrame(KinectFrame *frame);
};

#endif