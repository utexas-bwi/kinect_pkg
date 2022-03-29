#ifndef KINECT_FRAME_H
#define KINECT_FRAME_H

#include <k4a/k4a.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

class KinectWrapper;

class KinectFrame {
protected:
    friend class KinectWrapper;
    KinectWrapper *_kw;

    k4a::capture _capture;

    std::vector< std::string > _fieldNames;

public:
    KinectFrame(KinectWrapper *kw);
    ~KinectFrame();

    void initCVMat(int r, int c, int matType, std::string fieldName);
    void extractImages();
    void convertKinectImageToBGRA();
    void convertBGRAToBGR();
    void computeDepthInfo();

    std::map<std::string, cv::Mat *> _cvMats;
    k4a::image _colorImage, _depthImage, _xyzImage, _colorDepthImage;
};

#endif