#include <kinect_pkg/KFRHomToRT.h>
#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/KinectWrapper.h>

#include <Eigen/Eigen>

KFRHomToRT::KFRHomToRT() {}
KFRHomToRT::~KFRHomToRT() {}

void KFRHomToRT::receiveFrame(KinectFrame *frame) {
    Eigen::MatrixXd a = frame->_kw->_intrinsicsInverse * frame->_homography;
    Eigen::Vector3d rotA(a(0,0), a(1,0), a(2,0));
    Eigen::Vector3d rotB(a(0,1), a(1,1), a(2,1));
    double aNorm = rotA.norm();
    double bNorm = rotB.norm();
    rotA /= aNorm;
    rotB /= bNorm;
    Eigen::Vector3d rotC = rotA.cross(rotB);
}