#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

#include "BOOSTING/trackerAdaBoosting.hpp"
#include "BOOSTING/roiSelector.hpp"
#include "usbCamera.hpp"

#include <ros/ros.h>
#include "ros_visual_localization/pose.h" // generated from msg/pose.msg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

static const char* keys =
{   "{help h usage ?    | | print usage message}"
    "{useCamera         | | choose Camera or video}"
    "{d                 | | Camera device number}"
    "{v                 | | video name        }"
    "{r                 |5| ros rate}"};

static void help()
{
    std::cout << "parameters: \n"
                 "{help h usage ?    | | print usage message}\n"
                 "{useCamera         | | choose Camera or video}\n"
                 "{d                 | | Camera device number}\n"
                 "{v                 | | video name        }\n"
                 "{r                 |5| ros rate}\n\n";

    std::cout << "\n use example: ./visual_localization -useCamera=true -d=1 -r=5\n"
                 "\n use example: ./visual_localization -useCamera=false -v=xx.avi -r=5\n"
                 << std::endl;

    std::cout << "\n\nHot keys: \n"
       "\tq - quit the program\n"
       "\tp - pause/start video\n";
}

bool updateROI(cv::Mat &img, cv::Ptr<BOOSTING::Tracker> &tracker, cv::Rect2d &roi);
bool getNextImage(cv::Mat &img, cv::VideoCapture &cap);
void createImage(const cv::Mat &image, const std_msgs::Header &header, sensor_msgs::Image &msgImage);
void publishImage( const std_msgs::Header &header, 
                   const cv::Mat &image, 
                   const ros::Publisher &imagePub);
void createHeader(std_msgs::Header& header);


int main( int argc, char** argv ){
    ros::init(argc, argv, "visual_localization");
    ros::NodeHandle nh;
    ros::Publisher posePub = nh.advertise<ros_visual_localization::pose>("pose_from_vl", 1);
    ros::Publisher imagePub = nh.advertise<sensor_msgs::Image>("image_from_vl", 1);

    cv::CommandLineParser parser( argc, argv, keys );
    if(parser.has("help") || parser.has("h") || parser.has("usage") || parser.has("?")){
        help();
        return 0;
    }
    bool useCamera;
    int device;
    std::string video_name;
    if(!parser.has("useCamera"))
    {
        help();
        return 0;
    }
    useCamera = parser.get<bool>("useCamera");
    if(useCamera){
        if(!parser.has("d")){
            help();
            return 0;
        }
        device = parser.get<int>("d");
    }    
    else
    {
        if(!parser.has("v")){
            help();
            return 0;
        }
        video_name = parser.get<std::string>("v");
    }

    cv::VideoCapture cap;
    if(useCamera)
        cap.open(device);
    else 
        cap.open(video_name);

    if(!cap.isOpened()){
        std::stringstream ss;
        if(useCamera)
            ss << "ERROR! Unable to open camera device: " << device;
        else
            ss << "ERROR! Unable to open Video: " << video_name;
        std::cout << ss.str() << std::endl;
        help();
        return 0;
    }

    // instantiates the specific Tracker
    cv::Ptr<BOOSTING::Tracker> tracker = BOOSTING::TrackerBoosting::create();
    if(tracker == NULL){
        std::cout << "\nError in the instantiation of the tracker" << std::endl;
        return -1;
    }
    cv::Mat image;
    cv::Rect2d roi;
    cv::namedWindow("visual_localization", 1);

    // initialize the roi of tracking object
    if(getNextImage(image, cap))
        roi = BOOSTING::selectROI("visual_localization", image);

    bool tracker_initialized = false;
    bool pause_tracker = false;

    int ros_rate = parser.get<int>("r");
    ros::Rate loop_rate(ros_rate);

    while(ros::ok())
    {
        ros_visual_localization::pose pose_msg;
        std_msgs::Header header;

        do{
            if(!pause_tracker)
            {
                if(!tracker_initialized){
                    // initialize the tracker
                    if(!tracker->init(image, roi))
                    {
                        std::cout << "\n Could not initialize the tracker... \n";
                        return -1;
                    }
                    tracker_initialized = true;
                    continue;
                }
                if(updateROI(image, tracker, roi)){
                    pose_msg.x = roi.x + roi.width / 2;
                    pose_msg.y = roi.y + roi.height / 2;
                    pose_msg.theta = 0;
                    posePub.publish(pose_msg);
                }
                //cv::imshow("visual_localization", image);
            }
            char c = (char)cv::waitKey(2);
            if(c == 'q')
                break;
            if(c == 'p')
                pause_tracker = !pause_tracker;

            createHeader(header);
            publishImage(header, image, imagePub);

        }while(getNextImage(image, cap));

        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}


void createHeader(std_msgs::Header& header)
{
    //header.seq = xx;
    header.stamp = ros::Time::now();
    //header.frame_id = xx;
}

/* Read image, if success return ture
*/
bool getNextImage(cv::Mat &img, cv::VideoCapture &cap)
{
    cap.read(img);
    if(img.empty())
    {
        std::cout << "\n blank image grabbed!\n";
        return false;
    }
    return true;
}

/*  update localizatin roi
*/
bool updateROI(cv::Mat &img, cv::Ptr<BOOSTING::Tracker> &tracker, cv::Rect2d &roi)
{
    if(tracker->update(img, roi))
    {
        cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 2, 1);
        return true;
    }
    else
        return false;
}

void publishImage( const std_msgs::Header &header, 
                   const cv::Mat &image, 
                   const ros::Publisher &imagePub)
{
    sensor_msgs::Image imageMsg;
    createImage(image, header, imageMsg);
    imagePub.publish(imageMsg);
}

// input image, output msgImage
void createImage(const cv::Mat &image, const std_msgs::Header &header, sensor_msgs::Image &msgImage)
{
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    msgImage.encoding = sensor_msgs::image_encodings::BGR8;

    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size); // uint8[] data
    memcpy(msgImage.data.data(), image.data, size);
}