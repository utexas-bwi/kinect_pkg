#include <kinect_pkg/KinectWrapper.h>
#include <kinect_pkg/KFRAprilTag.h>
#include <kinect_pkg/KFRAprilCorners.h>
#include <kinect_pkg/KFRArray.h>
#include <kinect_pkg/KFRCloudPublish.h>
#include <kinect_pkg/KFRHomography.h>
#include <kinect_pkg/KFRHomographyRender.h>
#include <kinect_pkg/KFRImagePublish.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinectApril");
    ros::NodeHandle nh;
    KFRArray kfra;
    KFRCloudPublish kfrcp(nh);
    KFRImagePublish kfrip(nh);
    KFRAprilTag kfraprilTag;
    KFRAprilCorners kfraprilCorners;
    KFRHomography kfrHomography;
    KFRHomographyRender kfrHomographyRender(nh);
    kfra.addKFR(&kfraprilTag);
    kfra.addKFR(&kfraprilCorners);
    kfra.addKFR(&kfrHomography);
    kfra.addKFR(&kfrHomographyRender);
    kfra.addKFR(&kfrcp);
    kfra.addKFR(&kfrip);
    KinectWrapper kw(kfra);

    ros::Rate loop_rate(15);
    while(ros::ok()) {
        kw.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}