#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include <k4a/k4a.hpp>

#include <Eigen/Eigen>

#include <vector>
#include <string>

class KinectFrame;
class KinectFrameRecipient;
class KFRHomToRT;

class KinectWrapper {
public:
    KinectWrapper(KinectFrameRecipient &kfr);
    ~KinectWrapper();

    void update();

protected:
    friend class KinectFrame;
    friend class KFRHomToRT;

    KinectFrameRecipient &_kfr;
    Eigen::MatrixXd _intrinsics, _intrinsicsInverse;
    
    k4a::device _device;
    k4a_device_configuration_t _config;
    k4a::calibration _sensor_calibration;
    k4a::transformation _transformation;
};

#endif
