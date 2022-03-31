#ifndef KFR_ARRAY_H
#define KFR_ARRAY_H

#include "kinect_pkg/KinectFrameRecipient.h"

#include <vector>

class KFRArray : public KinectFrameRecipient {
public:
    KFRArray();
    ~KFRArray();

    void receiveFrame(KinectFrame *frame);
    void addKFR(KinectFrameRecipient *kfr);

protected:
    std::vector<KinectFrameRecipient *> _kfr;
};

#endif