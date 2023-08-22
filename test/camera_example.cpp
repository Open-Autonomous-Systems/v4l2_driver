//
// Created by mithun on 7/17/23.
//

#include "v4l2_driver.h"
//cpp
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
std::shared_ptr<v4l2_ns::V4l2_Camera> camPtr_;
void getFrame(v4l2_ns::imageWTs& imgStruct)
{
    camPtr_->getNextFrame(imgStruct);
}

int main(int argc, char** argv)
{
    v4l2_ns::CamConfig config;
    config.camID = "cam_0";
    config.deviceSerial = 31415;
    config.devicePath = "/dev/video0"; // for testing
    config.sensorWidth = 480;
    config.sensorHeight = 640;
    config.pixFmtENum = v4l2_ns::PIX_FMT::YVU420;
    //
    std::string logPath = "~/";

    camPtr_ = std::make_shared<v4l2_ns::V4l2_Camera>(config, logPath);
    camPtr_->initCamera();

    cv::namedWindow("Display window");
    while(1)
    {
        v4l2_ns::imageWTs imgStruct;
        getFrame(imgStruct);
        cv::imshow("Display window", imgStruct.img);
        int k = cv::waitKey(500); // Wait for a keystroke in the window
    }
    cv::destroyAllWindows();
//    imwrite("/home/mithun/ha.jpg", imgStruct.img);
//    getFrame(imgStruct);
//    imwrite("/home/mithun/ha2.jpg", imgStruct.img);
//    getFrame(imgStruct);
//    imwrite("/home/mithun/ha1.jpg", imgStruct.img);

    return 0;
}