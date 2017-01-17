#include <ros/ros.h>

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>

#include<ros/console.h>

using namespace std;

visualization_msgs::Marker load_marker(ros::NodeHandle &nh, std::string name){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // "map" is our global coordinate system
    marker.header.stamp = ros::Time::now();

    marker.ns = name;
    marker.id = 0;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();

    std::vector<double> pos,size,color;
    nh.getParam(name+"/position",pos);
    nh.getParam(name+"/size",size);
    nh.getParam(name+"/color",color);
    
    marker.color.r = color[0] / 255.;
    marker.color.g = color[1] / 255.;
    marker.color.b = color[2] / 255.;
    marker.color.a = 1.;
    marker.pose.position.x = pos[0];
    marker.pose.position.y = pos[1];
    marker.pose.position.z = pos[2];
    marker.scale.x = size[0];
    marker.scale.y = size[1];
    marker.scale.z = size[2];   
    
    return marker;
}


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    ros::init(argc, argv, "benchmark_laptop");
    ros::NodeHandle nh("~");

    // visualization publisher setup
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);

    // Setting up publishers takes a little bit of time
    // Wait for 1 sec for publisher to be ready
    ros::Duration(1.0).sleep();

    // visualization marker array
    visualization_msgs::MarkerArray marker_array;

    marker_array.markers.push_back(load_marker(nh,"table"));
    marker_array.markers.push_back(load_marker(nh,"laptop"));


    // Run main loop
    ros::Rate rate(1);
    ROS_INFO("start");

    while (ros::ok())
    {

        // TODO: communicating with a NLP module vis ros msgs

        // TODO: running motion planner and get the resulting trajectory

        // TODO: if using gazebo or real robot, send the trajectory to control the robot

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
          if (!ros::ok())
          {
            return 0;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }
        marker_pub.publish(marker_array);
        rate.sleep();
    }
    return 0;
}
