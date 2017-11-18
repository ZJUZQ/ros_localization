#ifndef USBCAMERA_DEFINE_HPP
#define USBCAMERA_DEFINE_HPP

#include <opencv2/opencv.hpp>

namespace usbCamera
{
    class usbCamera
    {
    private:
        int _device;    // camera device number
        cv::VideoCapture _cap;
        
    public:
        cv::Mat _image; 

        usbCamera(){
            _device = 0;
        }

        /* initialization */
        bool init(int device){
            _device = device;
            _cap.open(_device);
            if(!_cap.isOpened())
                return false;
            return true;
        }

        bool getNextImage(){
            _cap.read(_image);
            if(_image.empty())
                return false;
            return true;
        }
    };

};

#endif 