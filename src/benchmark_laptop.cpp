#include <ros/ros.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    ros::init(argc, argv, "benchmark_laptop");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    while (ros::ok())
    {
        ROS_INFO("test print");
        rate.sleep();
    }

    return 0;
}
