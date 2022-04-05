#include <kinect_pkg/KFRHomography.h>
#include <kinect_pkg/KinectFrame.h>

#include <ros/ros.h>

#include <Eigen/SVD>

#include <iostream>
#include <sstream>

using namespace Eigen;
using namespace std;

KFRHomography::KFRHomography() : _modelChessboard(2,4), _x(12,9) {
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
            _x(i * 3 + 0,0) = 0;
            _x(i * 3 + 0,1) = 0;
            _x(i * 3 + 0,2) = 0;

            _x(i * 3 + 0,3) = -_modelChessboard(0,i);
            _x(i * 3 + 0,4) = -_modelChessboard(1,i);
            _x(i * 3 + 0,5) = -1;

            _x(i * 3 + 0,6) = _modelChessboard(0,i) * frame->_cornerDetections(1,i);
            _x(i * 3 + 0,7) = _modelChessboard(1,i) * frame->_cornerDetections(1,i);
            _x(i * 3 + 0,8) =  frame->_cornerDetections(1,i);

            _x(i * 3 + 1,0) = _modelChessboard(0,i);
            _x(i * 3 + 1,1) = _modelChessboard(1,i);
            _x(i * 3 + 1,2) = 1;

            _x(i * 3 + 1,3) = 0;
            _x(i * 3 + 1,4) = 0;
            _x(i * 3 + 1,5) = 0;

            _x(i * 3 + 1,6) = -_modelChessboard(0,i) * frame->_cornerDetections(0,i);
            _x(i * 3 + 1,7) = -_modelChessboard(1,i) * frame->_cornerDetections(0,i);
            _x(i * 3 + 1,8) = -frame->_cornerDetections(0,i);

            _x(i * 3 + 2,0) = -_modelChessboard(0,i) * frame->_cornerDetections(1,i);
            _x(i * 3 + 2,1) = -_modelChessboard(1,i) * frame->_cornerDetections(1,i);
            _x(i * 3 + 2,2) = -frame->_cornerDetections(1,i);

            _x(i * 3 + 2,3) = _modelChessboard(0,i) * frame->_cornerDetections(0,i);
            _x(i * 3 + 2,4) = _modelChessboard(1,i) * frame->_cornerDetections(0,i);
            _x(i * 3 + 2,5) =  frame->_cornerDetections(0,i);

            _x(i * 3 + 2,6) = 0;
            _x(i * 3 + 2,7) = 0;
            _x(i * 3 + 2,8) = 0;

        }

        /*
        stringstream mathematica;
        mathematica << "{";
        for(int i = 0; i < _x.rows(); i++) {
            mathematica << "{";
            for(int j = 0; j < _x.cols(); j++) {
                mathematica << _x(i,j);
                if(j < (_x.cols() - 1)) {
                    mathematica << ",";
                }
            }
            mathematica << "}" << endl;
            if(i < (_x.rows() - 1)) {
                mathematica << ",";
            }
        }
        mathematica << "}";
        ROS_INFO_STREAM(mathematica.str());
        */

        JacobiSVD<MatrixXd> svd(_x, ComputeFullU | ComputeFullV);
        //MatrixXd b(9,1);s
        //for(int i = 0; i < 9; i++) b(i,0) = 0;
        //MatrixXd soln = svd.solve(b);
        MatrixXd singVal = svd.singularValues();
        int lowest = 0;
        double smallest = singVal(0,0);
        for(int i = 1; i < 9; i++) {
            double val = singVal(i,0);
            if((val < smallest) && (val > 0.001)) {
                smallest = val;
                lowest = i;
            }
        }
        MatrixXd soln = svd.matrixV().block(0,8,9,1);
        //ROS_INFO_STREAM("D: " << svd.singularValues());
        //ROS_INFO_STREAM("U: " << svd.matrixU());
        //ROS_INFO_STREAM("U: " << svd.matrixV());
        //ROS_INFO_STREAM("soln: " << soln);
        
        int ctr = 0;
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                frame->_homography(i,j) = soln(ctr++, 0);

        //ROS_INFO_STREAM("H: " << frame->_homography);

        frame->_tagDetected = true;
    } else {
        frame->_tagDetected = false;
    }
}