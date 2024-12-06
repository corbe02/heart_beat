#include "camera_pose_estimation.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_pose_estimation");
    ros::NodeHandle nh;

    OpticalFlowPose opticalFlowPose(nh);

    ros::spin();
    return 0;
}