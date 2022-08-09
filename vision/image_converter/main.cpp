#include "image_converter.h"
void SigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle h_node;
    //signal(SIGINT, SigintHandler);
    ros::Rate loop_rate(1); //program speed limit
    
    Vision cam;
    //Vision cam("/usb_cam/image_raw");
    //Vision cam("/raspicam_node/image/compressed");

    fflush(stdout); //更新文字緩衝區

    while (ros::ok())
    {
        //Mat src;
        //src = cam.get_source();
        //if(!src.empty()){
        //	imshow("src",src);
        //	cv::waitKey(10);
        //}
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
