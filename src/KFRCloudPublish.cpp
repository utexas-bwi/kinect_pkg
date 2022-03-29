#include <kinect_pkg/KFRCloudPublish.h>
#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/MSUtil.h>


KFRCloudPublish::KFRCloudPublish() :
    _pub(nh.advertise("/azure_kinect/points", 1)) {

}

KFRCloudPublish::~KFRCloudPublish() {}

void KFRCloudPublish::receiveFrame(KinectFrame *frame) {
    k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud);
    cv::Mat *m = frame->_cvMats["bgr"];
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", *m).toImageMsg();
    _pub.publish(msg);
}