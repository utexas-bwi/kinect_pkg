#ifndef KFR_HOM_TO_RT_H
#define KFR_HOM_TO_RT_H

#include "kinect_pkg/KinectFrameRecipient.h"

#include <Eigen/Eigen>

#include <vector>

class KFRHomography;

class KFRHomToRT : public KinectFrameRecipient {
protected:
    KFRHomography &_hom;
    Eigen::MatrixXd _modelChessboard;

public:
    KFRHomToRT(KFRHomography &hom);
    ~KFRHomToRT();

    void receiveFrame(KinectFrame *frame);
};

#endif