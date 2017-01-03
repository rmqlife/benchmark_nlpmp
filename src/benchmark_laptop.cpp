#include <ros/ros.h>

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>


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
    nh.getParam("camera", camera);

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

    std::cout << "table position: " << table_position.transpose() << std::endl;
    std::cout << "table size: " << table_size.transpose() << std::endl;


    // visualization publisher setup
    ros::Publisher visualization_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);

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

    marker.color.r = 139. / 255.;
    marker.color.g = 69. / 255.;
    marker.color.b = 19. / 255.;
    marker.color.a = 1.;

    tf::pointEigenToMsg(table_position, marker.pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(), marker.pose.orientation);

    tf::vectorEigenToMsg(table_size * 2., marker.scale);

    marker_array.markers.push_back(marker);

    visualization_marker_array_publisher.publish(marker_array);


    // Run main loop
    ros::Rate rate(2);

    while (ros::ok())
    {
        ROS_INFO("Main loop");

        // TODO: communicating with a NLP module vis ros msgs

        // TODO: running motion planner and get the resulting trajectory

        // TODO: if using gazebo or real robot, send the trajectory to control the robot

        rate.sleep();
    }


    return 0;
}
