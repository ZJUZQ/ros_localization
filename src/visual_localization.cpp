#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

#include "BOOSTING/trackerAdaBoosting.hpp"
#include "BOOSTING/roiSelector.hpp"
#include "video_ucam_Cap.hpp" // for usb camera and video

#include <ros/ros.h>
#include "ros_visual_localization/pose.h" // generated from msg/pose.msg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

/*
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
*/

bool updateROI(cv::Mat &img, cv::Ptr<BOOSTING::Tracker> &tracker, cv::Rect2d &roi);
void createImage(const cv::Mat &image, const std_msgs::Header &header, sensor_msgs::Image &msgImage);
void publishImage( const std_msgs::Header &header, 
                   const cv::Mat &image, 
                   const ros::Publisher &imagePub);
void createHeader(std_msgs::Header& header);


int main( int argc, char** argv ){
    ros::init(argc, argv, "visual_localization");
    ros::NodeHandle nh("~");   
    /*
    cv::CommandLineParser parser( argc, argv, keys );
    if(parser.has("help") || parser.has("h") || parser.has("usage") || parser.has("?")){
        help();
        return 0;
    }
    */
    std::string intrinsic_file;
    nh.getParam("intrinsic", intrinsic_file);
    cv::FileStorage fs;
    fs.open(intrinsic_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      std::cerr << "failed to open " << intrinsic_file << std::endl;
      return 1;
    }
    cv::Mat cameraMatrix, distCoeffs;
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs.release();

    video_usbCam::video_usbCam cam;
    std::string camera_type;
    nh.getParam("camera_type", camera_type);
    if(camera_type == "usb")
    {
        int device = -1;
        nh.getParam("camera_device", device);
        if(!cam.init(device, cameraMatrix, distCoeffs)){
            std::cout << "ERROR! Unable to open camera device: " << device << std::endl;
            return -1;
        }
    }
    else if(camera_type == "video")
    {
        std::string video_name;
        nh.getParam("video_name", video_name);
        if(!cam.init(video_name, cameraMatrix, distCoeffs)){
            std::cout << "ERROR! Unable to open Video: " << video_name << std::endl;
            return -1;
        }
    }

    // instantiates the specific Tracker
    cv::Ptr<BOOSTING::Tracker> tracker = BOOSTING::TrackerBoosting::create();
    if(tracker == NULL){
        std::cout << "\nError in the instantiation of the tracker" << std::endl;
        return -1;
    }
    cv::Mat image;
    cv::Rect2d roi;
    cv::namedWindow("visual_localization", 0);

    // initialize the roi of tracking object
    if(!cam.getNextRectifiedImage(image))
    {
        std::cout << "ERROR! failed to read image" << std::endl;
        return -1;
    }
    roi = BOOSTING::selectROI("visual_localization", image);
    std::cout << "debug: 1" << std::endl;

    bool tracker_initialized = false;
    bool pause_tracker = false;

    std::string camera_name;
    nh.getParam("camera_name", camera_name);
    ros::Publisher posePub = nh.advertise<ros_visual_localization::pose>("pose_" + camera_name, 1);
    ros::Publisher imagePub = nh.advertise<sensor_msgs::Image>("image_" + camera_name, 1);

    int rosRate;
    nh.getParam("rosRate", rosRate);
    ros::Rate loop_rate(rosRate);

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
                cv::imshow("visual_localization", image);
            }
            char c = (char)cv::waitKey(2);
            if(c == 'q')
                break;
            if(c == 'p')
                pause_tracker = !pause_tracker;

            createHeader(header);
            publishImage(header, image, imagePub);

        }while(cam.getNextRectifiedImage(image));

        ros::spinOnce();
        loop_rate.sleep();
    }
    cam.releaseCap();
    cv::destroyAllWindows();
    return 0;
}

/*
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

bool getNextImage(cv::Mat &image, usbCamera::usbCamera ucam)
{
    if(!ucam.getNextImage(image)){
        std::cout << "\n blank image grabbed!\n";
        return false;
    }
    ucam.rectifyImage(image);
    return true;
}
*/

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

void createHeader(std_msgs::Header& header)
{
    //header.seq = xx;
    header.stamp = ros::Time::now();
    //header.frame_id = xx;
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