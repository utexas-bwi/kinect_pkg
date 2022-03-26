#include <kinect_pkg/KinectWrapper.h>
#include <kinect_pkg/KFRImagePublish.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinectApril");
    ros::NodeHandle nh;
    KFRImagePublish kfrip(nh);
    KinectWrapper kw(kfrip);

    ros::Rate loop_rate(15);
    while(ros::ok()) {
        kw.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}