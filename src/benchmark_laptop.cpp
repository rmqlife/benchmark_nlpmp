#include <ros/ros.h>

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>

#include<ros/console.h>

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    ros::init(argc, argv, "benchmark_laptop");
    ros::NodeHandle nh("~");

    // Loading parameters from .yaml files.
    // The .yaml files must be loaded under /benchmark_laptop namespace.
    // Check out http://docs.ros.org/kinetic/api/roscpp/html/classros_1_1NodeHandle.html for how to use ros::NodeHandle::getParam().
    // Check out http://xmlrpcpp.sourceforge.net/doc/classXmlRpc_1_1XmlRpcValue.html for how to use XmlRpc parser.
    // Check out launch/benchmark_laptop.launch for loading .yaml files under /benchmark_laptop namespace.
    XmlRpc::XmlRpcValue camera;
    nh.getParam("camera/position", camera);

    XmlRpc::XmlRpcValue camera_position_xml = camera["position"];
    Eigen::Vector3d camera_position;
    for (int i=0; i<3; i++)
        camera_position(i) = camera_position_xml[i];

    XmlRpc::XmlRpcValue camera_orientation_xml = camera["orientation"];
    Eigen::Quaterniond camera_orientation;
    camera_orientation.w() = camera_orientation_xml[0];
    camera_orientation.x() = camera_orientation_xml[1];
    camera_orientation.y() = camera_orientation_xml[2];
    camera_orientation.z() = camera_orientation_xml[3];
    
    std::cout << "camera" << camera << std::endl;
    std::cout << "camrea position xml" << camera["position"] << std::endl;
    std::cout << "camera position: " << camera_position.transpose() << std::endl;
    std::cout << "camera orientation: " << camera_orientation.coeffs().transpose() << std::endl; // it outputs x, y, z, w in order


    XmlRpc::XmlRpcValue table;
    nh.getParam("table", table);

    XmlRpc::XmlRpcValue table_position_xml = table["position"];
    Eigen::Vector3d table_position;
    for (int i=0; i<3; i++)
        table_position(i) = table_position_xml[i];

    XmlRpc::XmlRpcValue table_size_xml = table["size"];
    Eigen::Vector3d table_size;
    for (int i=0; i<3; i++)
        table_size(i) = table_size_xml[i];

    std::cout << "table position: " << table_size_xml << std::endl;
    std::cout << "table size: " << table_size.transpose() << std::endl;


    // visualization publisher setup
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);

    // Setting up publishers takes a little bit of time
    // Wait for 1 sec for publisher to be ready
    ros::Duration(1.0).sleep();

    // visualization marker array
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map"; // "map" is our global coordinate system
    marker.header.stamp = ros::Time::now();

    marker.ns = "table";
    marker.id = 0;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();

    marker.color.r = 139. / 255.;
    marker.color.g = 69. / 255.;
    marker.color.b = 19. / 255.;
    marker.color.a = 1.;
    
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    
    
    tf::pointEigenToMsg(table_position, marker.pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(), marker.pose.orientation);
    tf::vectorEigenToMsg(table_size * 2., marker.scale);

    marker_array.markers.push_back(marker);

    // Run main loop
    ros::Rate rate(1);
    ROS_INFO("start");
    //ROS_INFO("Hello %d", table_size_xml[0]);
    ROS_INFO_STREAM( "table size haha: " << table_size_xml[0] << table_size_xml[1] << table_size_xml[2] );
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
