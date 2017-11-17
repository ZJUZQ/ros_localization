#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

#include "BOOSTING/trackerAdaBoosting.hpp"
#include "BOOSTING/roiSelector.hpp"

using namespace std;
using namespace cv;

static const char* keys =
{   "{help h usage ?    | | print usage message}"
    "{useCamera         | | choose Camera or video}"
    "{d                 | | Camera device number}"
    "{v                 | | video name        }"};

static void help()
{
    std::cout << "\n use example: ./visual_localization -useCamera=true -d=1\n"
                 "\n use example: ./visual_localization -useCamera=false -v=xx.avi\n"
                 << std::endl;

    std::cout << "\n\nHot keys: \n"
       "\tq - quit the program\n"
       "\tp - pause/start video\n";
}

bool updateROI(cv::Mat &img, cv::Ptr<BOOSTING::Tracker> &tracker, cv::Rect2d &roi);
bool getNextImage(cv::Mat &img, cv::VideoCapture &cap);

int main( int argc, char** argv ){

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
            updateROI(image, tracker, roi);
            cv::imshow("visual_localization", image);
        }
        char c = (char)cv::waitKey(2);
        if(c == 'q')
            break;
        if(c == 'p')
            pause_tracker = !pause_tracker;

    }while(getNextImage(image, cap));

    return 0;
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
