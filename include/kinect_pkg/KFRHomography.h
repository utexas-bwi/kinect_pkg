#ifndef KFR_HOMOGRAPHY_H
#define KFR_HOMOGRAPHY_H

#include <kinect_pkg/KinectFrameRecipient.h>

#include <Eigen/Eigen>

class KFRHomography : public KinectFrameRecipient {
protected:
    friend class KFRHomToRT;
    Eigen::MatrixXd _modelChessboard;
    Eigen::MatrixXd _x;

public:
    KFRHomography();
    ~KFRHomography();

    void receiveFrame(KinectFrame *frame);
};

#endif