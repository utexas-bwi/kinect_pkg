#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/KinectWrapper.h>

#include <iostream>

using namespace std;

KinectFrame::KinectFrame(KinectWrapper *kw) : _kw(kw), _tagDetected(false), _homography(Eigen::MatrixXd(3,3)) {
    initCVMat(2160, 3840, CV_8UC4, "bgra");
    initCVMat(2160, 3840, CV_8UC3, "bgr");
    initCVMat(2160, 3840, CV_8UC3, "bgr_april");
    initCVMat(2160, 3840, CV_8UC3, "bgr_homography");
}

KinectFrame::~KinectFrame() {
    for(int i = 0; i < _fieldNames.size(); i++) {
        delete _cvMats[_fieldNames[i]];
    }
}

void KinectFrame::initCVMat(int r, int c, int matType, std::string fieldName) {
    //cout << "KinectFrame::initCVMat:    " << r << " " << c << " " << fieldName << endl;
    _fieldNames.push_back(fieldName);
    _cvMats[fieldName] = new cv::Mat(r, c, matType);
}

void KinectFrame::extractImages() {
    _colorImage = _capture.get_color_image();
    _depthImage = _capture.get_depth_image();
    
    _xyzImage =
        k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
            _depthImage.get_width_pixels(), _depthImage.get_height_pixels(),
            _depthImage.get_width_pixels() * 3 * sizeof(int16_t));
    _colorDepthImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        _depthImage.get_width_pixels(), _depthImage.get_height_pixels(),
        _depthImage.get_width_pixels() * 4 * sizeof(int8_t));
}

void KinectFrame::convertKinectImageToBGRA() {
    uint8_t *buffer = _colorImage.get_buffer();
    cv::Mat *m = _cvMats["bgra"];
    memcpy(m->data, buffer, 2160 * 3840 * 4);
}

void KinectFrame::convertBGRAToBGR() {
    cv::Mat *bgra = _cvMats["bgra"];
    cv::Mat *bgr = _cvMats["bgr"];
    cv::cvtColor(*bgra, *bgr, cv::COLOR_BGRA2BGR);
}

void KinectFrame::computeDepthInfo() {
    _kw->_transformation.depth_image_to_point_cloud(_depthImage, K4A_CALIBRATION_TYPE_DEPTH, &_xyzImage);
    _kw->_transformation.color_image_to_depth_camera(_depthImage, _colorImage, &_colorDepthImage);
}