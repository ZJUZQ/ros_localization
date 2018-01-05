#ifndef VIDEOCAP_DEFINE_HPP
#define VIDEOCAP_DEFINE_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

namespace videoCap
{
    class videoCap
    {
    private:
        std::string _vfile;    // camera device number
        cv::VideoCapture _cap;
        cv::Mat _cameraMatrix;
        cv::Mat _distCoeffs;
        
    public:
        //cv::Mat _image; 

        videoCap(){
            _vfile = "";
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