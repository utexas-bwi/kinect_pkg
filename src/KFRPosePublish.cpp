#include <kinect_pkg/KFRPosePublish.h>
#include <kinect_pkg/KinectFrame.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>

KFRPosePublish::KFRPosePublish(ros::NodeHandle &nh) :
    _pub(nh.advertise<geometry_msgs::PoseStamped>("/azure_kinect/tag_pose", 1)) {}

KFRPosePublish::~KFRPosePublish() {}

void KFRPosePublish::receiveFrame(KinectFrame *frame) {
    if(frame->_tagDetected) {
        Eigen::Quaterniond q(frame->_rigidTransform.block<3,3>(0,0));
        q.normalize();

        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "azure_kinect/camera_body";
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = frame->_rigidTransform(0,3);
        msg.pose.position.y = frame->_rigidTransform(1,3);
        msg.pose.position.z = frame->_rigidTransform(2,3);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        _pub.publish(msg);
    }
}