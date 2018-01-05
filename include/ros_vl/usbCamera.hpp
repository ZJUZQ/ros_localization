#ifndef USBCAMERA_DEFINE_HPP
#define USBCAMERA_DEFINE_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

namespace usbCamera
{
    class usbCamera
    {
    private:
        int _device;    // camera device number
        cv::VideoCapture _cap;
        cv::Mat _cameraMatrix;
        cv::Mat _distCoeffs;
        
    public:
        //cv::Mat _image; 

        usbCamera(){
            _device = -1;
        }

        /* initialization */
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
    };

};

#endif 