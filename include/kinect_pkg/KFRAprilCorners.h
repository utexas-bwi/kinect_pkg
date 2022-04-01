#ifndef KFR_APRIL_CORNERS_H
#define KFR_APRIL_CORNERS_H

#include <kinect_pkg/KinectFrameRecipient.h>


class KFRAprilCorners : public KinectFrameRecipient {
public:
    KFRAprilCorners();
    
    void receiveFrame(KinectFrame *frame);
    
};

#endif