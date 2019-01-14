#include "reconstruct/processing/ImageProcessor.h"
#include <scout3d_motor/MotorPosition.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>

ros::Publisher publisher;
ImageProcessor* processor;

void motorPositionCallback(const scout3d_motor::MotorPosition::ConstPtr& msg)
{
    Eigen::Affine3d transform = Eigen::Affine3d(Eigen::AngleAxisd(-msg->position * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
    Eigen::Affine3d calibTrafo = Eigen::Affine3d::Identity();
    calibTrafo.translate(Eigen::Vector3d(0,-0.188,0));
    calibTrafo = calibTrafo * Eigen::Affine3d(Eigen::AngleAxisd(-14.15 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
    Eigen::Matrix4d pose = (transform * calibTrafo).matrix();

    tf::Transform rosTF;
    tf::transformEigenToTF(Eigen::Affine3d(pose), rosTF);

    tf::StampedTransform trans;
    trans = tf::StampedTransform(rosTF, msg->timestamp, "/scanner_base", "/camera");

    static tf::TransformBroadcaster br;
    br.sendTransform(trans);

    std::cout << "Publishing Transform!" << std::endl;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if (processor->process(msg)) {
        publisher.publish(processor->cloud());
    }
}

int main(int argc, char **argv)
{
    omp_set_num_threads(2);

    ros::init (argc, argv, "scanner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string configurationFile;
    nh_private.param<std::string>("configuration_file", configurationFile, "configuration.xml");
    processor = new ImageProcessor(configurationFile);

    publisher = nh.advertise<sensor_msgs::PointCloud>("cloud", 1);

    ros::Subscriber motorMessageSubscriber = nh.subscribe("/motor/motorPosition", 1, motorPositionCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_color", 1, imageCallback, image_transport::TransportHints("compressed"));

    ros::spin();
    return 0;
}
