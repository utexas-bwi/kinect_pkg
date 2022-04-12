#include <kinect_pkg/KFRHomToRT.h>
#include <kinect_pkg/KFRHomography.h>
#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/KinectWrapper.h>

#include <Eigen/Eigen>

#include <ros/ros.h>

KFRHomToRT::KFRHomToRT(KFRHomography &hom) : _hom(hom), _modelChessboard(4,4) {
    for(int i = 0; i < 4; i++) {
        _modelChessboard(0,i) = _hom._modelChessboard(0,i);
        _modelChessboard(1,i) = _hom._modelChessboard(1,i);
        _modelChessboard(2,i) = 0;
        _modelChessboard(3,i) = 1;
    }
}

KFRHomToRT::~KFRHomToRT() {}

void KFRHomToRT::receiveFrame(KinectFrame *frame) {
    //if(false) {
    if(frame->_tagDetected) {
        //frame->_homography /= frame->_homography(2,2);
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
            frame->_rigidTransform(i,3) = (a(i,2) / transNorm) * (173.0 / 2.0) / 1000.0;
            frame->_rigidTransform(3,i) = 0;
        }
        frame->_rigidTransform(3,3) = 1;

        /*
        Eigen::MatrixXd result = frame->_rigidTransform * _modelChessboard;
        double d = 0.0;
        Eigen::MatrixXd ptA, ptB, ptDiff;
        for(int i = 0; i < 3; i++) {
            ptA = result.block(0,i,4,1);
            ptB = result.block(0,i+1,4,1);
            ptA /= ptA(3,0);
            ptB /= ptA(3,0);
            ptDiff = ptA - ptB;
            d += ptDiff.norm();
        }

        ptA = result.block(0,3,4,1);
        ptB = result.block(0,0,4,1);
        ptA /= ptA(3,0);
        ptB /= ptA(3,0);
        ptDiff = ptA - ptB;
        d += ptDiff.norm();
        d /= 4.0;
        //173mm
        //frame->_rigidTransform *= 173.0/2.0;

        ROS_INFO_STREAM(d);
        */
    }
}