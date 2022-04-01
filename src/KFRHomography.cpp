#include <kinect_pkg/KFRHomography.h>
#include <kinect_pkg/KinectFrame.h>

#include <Eigen/SVD>

#include <iostream>

using namespace Eigen;
using namespace std;

KFRHomography::KFRHomography() : _modelChessboard(2,4), _x(8,9) {
    //First point, lower left
    _modelChessboard(0,0) = -1;
    _modelChessboard(1,0) =  1;

    //Second point, lower right
    _modelChessboard(0,1) =  1;
    _modelChessboard(1,1) =  1;

    //Third point, upper right
    _modelChessboard(0,2) =  1;
    _modelChessboard(1,2) = -1;

    //Fourth point, upper left
    _modelChessboard(0,3) = -1;
    _modelChessboard(1,3) = -1;

}

KFRHomography::~KFRHomography() {}

void KFRHomography::receiveFrame(KinectFrame *frame) {
    if(frame->_cornerDetections.cols()) {
        for(int i = 0; i < 4; i++) {
            _x(i * 2 + 0,0) = -_modelChessboard(0,i);
            _x(i * 2 + 0,1) = -_modelChessboard(1,i);
            _x(i * 2 + 0,2) = -1;

            _x(i * 2 + 0,3) = 0;
            _x(i * 2 + 0,4) = 0;
            _x(i * 2 + 0,5) = 0;

            _x(i * 2 + 0,6) = _modelChessboard(0,i) * frame->_cornerDetections(0,i);
            _x(i * 2 + 0,7) = _modelChessboard(1,i) * frame->_cornerDetections(0,i);
            _x(i * 2 + 0,8) =  frame->_cornerDetections(0,i);

            _x(i * 2 + 1,0) = 0;
            _x(i * 2 + 1,1) = 0;
            _x(i * 2 + 1,2) = 0;

            _x(i * 2 + 1,3) = -_modelChessboard(0,i);
            _x(i * 2 + 1,4) = -_modelChessboard(1,i);
            _x(i * 2 + 1,5) = -1;

            _x(i * 2 + 1,6) = _modelChessboard(0,i) * frame->_cornerDetections(1,i);
            _x(i * 2 + 1,7) = _modelChessboard(1,i) * frame->_cornerDetections(1,i);
            _x(i * 2 + 1,8) =  frame->_cornerDetections(1,i);
        }
    }

    JacobiSVD<MatrixXd> svd(_x, ComputeThinU | ComputeThinV);
    MatrixXd b(9,1);
    svd.solve(b);
    cout << "b: " << b << endl;
}