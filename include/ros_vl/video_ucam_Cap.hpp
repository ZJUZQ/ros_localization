#ifndef VIDEOUSBCAM_DEFINE_HPP
#define VIDEOUSBCAM_DEFINE_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

namespace video_usbCam
{
    class video_usbCam
    {
    private:
        int _device;        // camera device number
        std::string _vfile; // video file name
        cv::VideoCapture _cap;
        cv::Mat _cameraMatrix;
        cv::Mat _distCoeffs;
        
    public:
        //cv::Mat _image; 

        video_usbCam(){
            _vfile = "";
            _device = -1;

        }

        /* initialization */
        bool init(const std::string vfile, cv::Mat cameraMatrix, cv::Mat distCoeffs){
            _vfile = vfile;
            _cameraMatrix = cameraMatrix;
            _distCoeffs = distCoeffs;
            _cap.open(_vfile);
            _cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
            _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
            if(!_cap.isOpened())
                return false;
            return true;
        }

        bool init(int device, cv::Mat cameraMatrix, cv::Mat distCoeffs){
            _device = device;
            _cameraMatrix = cameraMatrix;
            _distCoeffs = distCoeffs;
            _cap.open(_device);
            _cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
            _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
            if(!_cap.isOpened())
                return false;
            return true;
        }

        bool getNextImage(cv::Mat &img){
            _cap.read(img);
            if(img.empty())
                return false;
            return true;
        }
        
        bool getNextRectifiedImage(cv::Mat &img){
            _cap.read(img);
            if(img.empty())
                return false;
            cv::Mat tmp = img.clone();
            cv::undistort(tmp, img, _cameraMatrix, _distCoeffs);
            return true;
        }

        void rectifyImage(cv::Mat &img){
            cv::Mat originImg = img.clone();
            cv::undistort(originImg, img, _cameraMatrix, _distCoeffs);
        }
        void releaseCap(){
            _cap.release();
        }
    };

};

#endif 