#include <kinect_pkg/KFRHomToRT.h>
#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/KinectWrapper.h>

#include <Eigen/Eigen>

KFRHomToRT::KFRHomToRT() {}
KFRHomToRT::~KFRHomToRT() {}

void KFRHomToRT::receiveFrame(KinectFrame *frame) {
    //if(false) {
    if(frame->_tagDetected) {
        frame->_homography /= frame->_homography(2,2);
        Eigen::MatrixXd a = frame->_kw->_intrinsicsInverse * frame->_homography;
        Eigen::Vector3d rotA(a(0,0), a(1,0), a(2,0));
        Eigen::Vector3d rotB(a(0,1), a(1,1), a(2,1));
        double aNorm = rotA.norm();
        double bNorm = rotB.norm();
        rotA /= aNorm;
        rotB /= bNorm;
        double transNorm = (aNorm + bNorm) / 2.0;
        Eigen::Vector3d rotC = rotA.cross(rotB);
        for(int i = 0; i < 3; i++) {
            frame->_rigidTransform(i,0) = rotA(i);
            frame->_rigidTransform(i,1) = rotB(i);
            frame->_rigidTransform(i,2) = rotC(i);
            frame->_rigidTransform(i,3) = a(i,2);
            frame->_rigidTransform(3,i) = 0;
        }
        frame->_rigidTransform(3,3) = 1;
    }
}