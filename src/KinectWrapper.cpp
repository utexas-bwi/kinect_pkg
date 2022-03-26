#include <kinect_pkg/KinectWrapper.h>
#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/KinectFrameRecipient.h>
#include <cstdio>
#include <cstdlib>

using namespace std;

KinectWrapper::KinectWrapper(KinectFrameRecipient &kfr) : _kfr(kfr) {
    _config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    _config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    _config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    _config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    _config.synchronized_images_only = true;
    _config.depth_delay_off_color_usec = 0;
    _config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    _config.subordinate_delay_off_master_usec = 0;
    _config.disable_streaming_indicator = false;

    _device = k4a::device::open(0);
    _sensor_calibration = _device.get_calibration(_config.depth_mode, _config.color_resolution);
    _transformation = k4a::transformation(_sensor_calibration);

    _device.start_cameras(&_config);
}

KinectWrapper::~KinectWrapper() {
    _device.stop_cameras();
    _device.close();
}

void KinectWrapper::update() {
    KinectFrame kf;
    _device.get_capture(&kf._capture, std::chrono::milliseconds(K4A_WAIT_INFINITE));
    kf.convertKinectImageToBGRA();
    kf.convertBGRAToBGR();
    _kfr.receiveFrame(&kf);
}