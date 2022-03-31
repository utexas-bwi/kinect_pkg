#include <kinect_pkg/KFRCloudPublish.h>
#include <kinect_pkg/KinectFrame.h>
#include <kinect_pkg/MSUtil.h>


KFRCloudPublish::KFRCloudPublish(ros::NodeHandle &nh) :
    _pub(nh.advertise<sensor_msgs::PointCloud2>("/azure_kinect/points", 1)) {

}

KFRCloudPublish::~KFRCloudPublish() {}

void KFRCloudPublish::receiveFrame(KinectFrame *frame) {
    sensor_msgs::PointCloud2 pointCloud;
    pointCloud.header.frame_id = "depth_camera_link";
    fillColorPointCloud(frame->_xyzImage, frame->_colorDepthImage, &pointCloud);
    _pub.publish(pointCloud);
}