#include <kinect_pkg/KFRPosePublish.h>
#include <kinect_pkg/KinectFrame.h>
#include <geometry_msgs/PoseStamped.h>


KFRPosePublish::KFRPosePublish(ros::NodeHandle &nh) :
    _pub(nh.advertise<geometry_msgs::PoseStamped>("/azure_kinect/tag_pose", 1)) {}

KFRPosePublish::~KFRPosePublish() {}

void KFRPosePublish::receiveFrame(KinectFrame *frame) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "depth_camera_link";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = frame->_rigidTransform(0,3);
    msg.pose.position.y = frame->_rigidTransform(1,3);
    msg.pose.position.z = frame->_rigidTransform(2,3);
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    _pub.publish(msg);
}