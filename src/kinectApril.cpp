#include <kinect_pkg/KinectWrapper.h>
#include <kinect_pkg/KFRAprilTag.h>
#include <kinect_pkg/KFRArray.h>
#include <kinect_pkg/KFRCloudPublish.h>
#include <kinect_pkg/KFRImagePublish.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinectApril");
    ros::NodeHandle nh;
    KFRArray kfra;
    KFRCloudPublish kfrcp(nh);
    KFRImagePublish kfrip(nh);
    KFRAprilTag kfrapril;
    kfra.addKFR(&kfrcp);
    kfra.addKFR(&kfrip);
    kfra.addKFR(&kfrapril);
    KinectWrapper kw(kfra);

    ros::Rate loop_rate(15);
    while(ros::ok()) {
        kw.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}