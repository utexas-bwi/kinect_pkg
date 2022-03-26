#include <kinect_pkg/KinectFrame.h>

#include <iostream>

using namespace std;

KinectFrame::KinectFrame() {
    initCVMat(2160, 3840, CV_8UC4, "bgra");
    initCVMat(2160, 3840, CV_8UC3, "bgr");
    
    _xyzImage =
        k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
            320, 288,
            320 * 3 * sizeof(int16_t));
    _colorDepthImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        320, 288,
        320 * 4 * sizeof(int8_t));
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

void KinectFrame::convertKinectImageToBGRA() {
    k4a::image image = _capture.get_color_image();
    uint8_t *buffer = image.get_buffer();
    cv::Mat *m = _cvMats["bgra"];
    memcpy(m->data, buffer, 2160 * 3840 * 4);
}

void KinectFrame::convertBGRAToBGR() {
    cv::Mat *bgra = _cvMats["bgra"];
    cv::Mat *bgr = _cvMats["bgr"];
    cv::cvtColor(*bgra, *bgr, cv::COLOR_BGRA2BGR);
}

void KinectFrame::computeDepthInfo() {
    //_depthImage = _capture[i].get_depth_image();
    //_wrappers.at(i)->transformation.depth_image_to_point_cloud(depth_image, K4A_CALIBRATION_TYPE_DEPTH, &_xyz_image);


    //k4a::image colorImage = _capture[i].get_color_image();
    //_wrappers.at(i)->transformation.color_image_to_depth_camera(depth_image, colorImage, &colorDepthImage);
}