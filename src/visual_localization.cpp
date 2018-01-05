#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>
#include <sstream>

#include "BOOSTING/trackerAdaBoosting.hpp"
#include "BOOSTING/roiSelector.hpp"
#include "ros_vl/video_ucam_Cap.hpp" // for usb camera and video
#include "ros_vl/common_utility.hpp"

#include <ros/ros.h>
#include "ros_visual_localization/pose.h" // generated from msg/pose.msg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <time.h>
#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

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

bool refresh_H = false;
bool tracker_initialized = false;
void actionCallback(const std_msgs::String::ConstPtr& msg){
    if(msg->data.find("refresh_H")){ // refresh the H_to_bg
        printf("\n Will refresh the H_to_bg !\n");
        refresh_H = true;
    }
    if(msg->data.find("initialize_tracker")){
        printf("\n Will reinitialize the tracker ! \n");
        tracker_initialized = false;
    }
}

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
    boost::asio::io_service ioService;
    std::string serialPortName;
    nh.getParam("serialPortName", serialPortName);
    serialPortName = "/dev/" + serialPortName;
    std::cout << "debug: serialPortName = \n" << serialPortName << std::endl;
    boost::asio::serial_port sPort(ioService, serialPortName);
    //boost::asio::serial_port sPort(ioService, "/dev/ttyUSB0");
    // set post's parameter
    sPort.set_option( boost::asio::serial_port::baud_rate(9600) );
    sPort.set_option( boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none) );
    sPort.set_option( boost::asio::serial_port::parity(boost::asio::serial_port::parity::odd) );
    sPort.set_option( boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one) );
    sPort.set_option( boost::asio::serial_port::character_size(8) );



    std::string intrinsic_file;
    cv::Mat cameraMatrix, distCoeffs;
    int rectify = 1;
    nh.getParam("rectify", rectify);
    //std::cout << "debug: rectify = " << rectify << std::endl;

    if(rectify) // when read image, need to rectify the image
    {
        nh.getParam("intrinsic", intrinsic_file);

        cv::FileStorage fs;
        fs.open(intrinsic_file, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
          std::cerr << "failed to open " << intrinsic_file << std::endl;
          return 1;
        }

        fs["Camera_Matrix"] >> cameraMatrix;
        fs["Distortion_Coefficients"] >> distCoeffs;
        fs.release();
    }

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
        std::string video_file;
        nh.getParam("video_file", video_file);
        if(!cam.init(video_file, cameraMatrix, distCoeffs)){
            std::cout << "ERROR! Unable to open Video: " << video_file << std::endl;
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
    if(rectify)
    {
        if(!cam.getNextRectifiedImage(image))
        {
            std::cout << "ERROR! failed to read image" << std::endl;
            return -1;
        }
    }
    else // do not rectify the image
    {
        if(!cam.getNextImage(image))
        {
            std::cout << "ERROR! failed to read image" << std::endl;
            return -1;
        }
    }

    std::string camera_name;
    nh.getParam("camera_name", camera_name);
    ros::Publisher posePub = nh.advertise<ros_visual_localization::pose>("pose_" + camera_name, 1);
    ros::Publisher imagePub = nh.advertise<sensor_msgs::Image>("image_" + camera_name, 1);
    ros::Subscriber actionSub = nh.subscribe("action", 1, actionCallback);
    //ros::Publisher imageCompressedPub = nh.advertise<sensor_msgs::CompressedImage>("image_" + camera_name, 1);

    int rosRate;
    nh.getParam("rosRate", rosRate);
    ros::Rate loop_rate(rosRate);

    ros_visual_localization::pose pose_msg;
    std_msgs::Header header;

    cv::namedWindow("visual_localization", 0);
    std::string imgName_bg;
    nh.getParam("bgImage", imgName_bg);
    cv::Mat img_bg, img_bg_display;
    img_bg = cv::imread(imgName_bg, cv::IMREAD_COLOR);
    img_bg.copyTo(img_bg_display);
    /*
    printf("\ndebug: img's cols == %d", img_bg_display.cols);
    printf("\ndebug: img's rows == %d", img_bg_display.rows);
    printf("\ndebug: img's depth == %d", img_bg_display.depth());
    */

    cv::Ptr<cv::xfeatures2d::SURF> feature_detector = cv::xfeatures2d::SURF::create(400);
    std::string detector_method = "SURF";

    std::vector<cv::KeyPoint> kps_frame;
    cv::Mat descriptors_frame;
    feature_detector->detectAndCompute(image, cv::Mat(), kps_frame, descriptors_frame); // Detects keypoints and computes the descriptors 
    
    std::vector<cv::KeyPoint> kps_bg;
    cv::Mat descriptors_bg;
    feature_detector->detectAndCompute(img_bg_display, cv::Mat(), kps_bg, descriptors_bg);

    cv::Mat H_to_bg, H_to_frame;
    ros_vl::compute_homography(detector_method, H_to_frame, H_to_bg, kps_frame, kps_bg, descriptors_frame, descriptors_bg);
    ros_vl::part_warpPerspective(image, img_bg_display, cv::Rect2d(0, 0, img_bg.cols, img_bg.rows), H_to_frame);

    cv::Rect2d roi = BOOSTING::selectROI("visual_localization", img_bg_display);
    cv::Rect2d bb_for_perspective =  ros_vl::enlargeRect(img_bg_display, roi, 3);
    cv::destroyWindow("visual_localization");

    bool pause_tracker = false;

    while(ros::ok())
    {
        clock_t t = clock();
        if(!tracker_initialized){   // reinitialize the tracker
            if(!tracker->init(img_bg_display, roi))
            {
                std::cout << "\n Could not initialize the tracker... \n";
                return -1;
            }
            tracker_initialized = true;
        }
    
        bool hasNewImage = false;
        if(rectify)
            hasNewImage = cam.getNextRectifiedImage(image);
        else
            hasNewImage = cam.getNextImage(image);

        if(refresh_H){  // refresh the H_to_bg
            kps_frame.clear();
            feature_detector->detectAndCompute(image, cv::Mat(), kps_frame, descriptors_frame);
            ros_vl::compute_homography(detector_method, H_to_frame, H_to_bg, kps_frame, kps_bg, descriptors_frame, descriptors_bg);
        }

        if(hasNewImage)
        {
            if(!pause_tracker)
            {
                
                bb_for_perspective = ros_vl::enlargeRect(img_bg_display, roi, 3);
                // perspective the image to the img_bg_display's ROI image: 
                img_bg.copyTo(img_bg_display);
                ros_vl::part_warpPerspective(image, img_bg_display, bb_for_perspective, H_to_frame);
                cv::rectangle(img_bg_display, bb_for_perspective, cv::Scalar(0, 0, 255), 1, 1);

                if(updateROI(img_bg_display, tracker, roi))
                {
                    pose_msg.x = roi.x + roi.width / 2;
                    pose_msg.y = roi.y + roi.height / 2;
                    pose_msg.theta = 0;
                    pose_msg.camera_name.data = camera_name;
                    posePub.publish(pose_msg);

                    char buffers[25];
                    sprintf(buffers, "x%7.1f", double(pose_msg.x));
                    sprintf(buffers + 8, "y%7.1f", double(pose_msg.y));
                    sprintf(buffers + 16, "t%7.1f", double(pose_msg.theta));
                    buffers[24] = '\n';

                    boost::asio::write(sPort, boost::asio::buffer(buffers) );
                }  
            }
            //createHeader(header);
            //publishImage(header, img_bg_display, imagePub);
            
            //cv::imshow("visual_localization", img_bg_display);
        }
        t = clock() - t;
        printf("\n One frame consumes %.1f ms!\n", ((float)t)*1000/CLOCKS_PER_SEC);

        char c = (char)cv::waitKey(2);
        if(c == 'q')
            break;
        if(c == 'p')
            pause_tracker = !pause_tracker;

        ros::spinOnce();
        loop_rate.sleep();
    }
    cam.releaseCap();
    cv::destroyAllWindows();
    return 0;
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