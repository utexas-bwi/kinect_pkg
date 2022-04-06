#ifndef KFR_HOM_TO_RT_H
#define KFR_HOM_TO_RT_H

#include "kinect_pkg/KinectFrameRecipient.h"

#include <vector>

class KFRHomToRT : public KinectFrameRecipient {
public:
    KFRHomToRT();
    ~KFRHomToRT();

    void receiveFrame(KinectFrame *frame);
};

#endif