#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace boost;
using namespace cv;

namespace std {
ostream& operator<<(ostream &os, const vector<string> &vec) {
    for (auto item : vec) {
        os << item << " ";
    }
    return os;
}
}

int main(int argc, char* argv[])
{
    string bagfile;
    string topic;
    double startTime;
    double endTime;
    string outdir;

    program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("bag,b", program_options::value<string>(&bagfile)->required(), "input ros bag file")
            ("topic,t", program_options::value<string>(&topic)->default_value("/camera/image_color/compressed"), "Image topic")
            ("start-time", program_options::value<double>(&startTime)->default_value(946684800), "Start timestamp of export")
            ("end-time", program_options::value<double>(&endTime)->default_value(4102444800), "End timestamp of export")
            ("output,o", program_options::value<string>(&outdir)->required(), "output folder")
            ;

    program_options::variables_map vm;
    program_options::store(program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return EXIT_SUCCESS;
    }

    program_options::notify(vm);

    filesystem::create_directory(outdir);

    rosbag::Bag bag(bagfile);
    rosbag::View viewImages(bag, rosbag::TopicQuery(topic));

    for (rosbag::MessageInstance const m : viewImages) {
        cv_bridge::CvImagePtr cv_ptr;
        ros::Time stamp;

        if (m.isType<sensor_msgs::CompressedImage>()) {
            sensor_msgs::CompressedImage::ConstPtr message = m.instantiate<sensor_msgs::CompressedImage>();

            if (message->header.stamp < ros::Time(startTime) || message->header.stamp > ros::Time(endTime)) {
                continue;
            }

            stamp = message->header.stamp;

            try {
                cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
        } else if (m.isType<sensor_msgs::Image>()) {
            sensor_msgs::Image::ConstPtr message = m.instantiate<sensor_msgs::Image>();

            if (message->header.stamp < ros::Time(startTime) || message->header.stamp > ros::Time(endTime)) {
                continue;
            }

            stamp = message->header.stamp;

            try {
                cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
        } else {
            continue;
        }

        cv::Mat image = cv_ptr->image;

        std::string filename = boost::lexical_cast<std::string>(stamp) + ".jpg";
        cv::imwrite(outdir + "/" + filename, image);
    }

    return 0;
}
