//
// Created by mithun on 7/7/23.
//

#ifndef V4L2_DRIVER_H
#define V4L2_DRIVER_H

// cpp
#include <iostream>
#include <string>

// Linux / system
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

// opencv
#include <opencv2/opencv.hpp>

// glog
#include <glog/logging.h>
#include <glog/log_severity.h>

/**
  credits and auxillary Licenses:
  // source, type of License, Location of License
  1. This pkg, MIT license, root of repo/LICENSE
  2. ROS_wrapper single cam: Flir_boson_usb from AStuff, license at link
  3. cpp_variant: BosonUSB from FLIR, MIT license at link
  4. JHSmith tutorial blog at link, license not specified explictly.
  5. v4l2 api and documentation from Linux Kernel Media
*/
namespace v4l2_ns {
    /**
     * @ CamConfig Struct to pass camera information to V4l2Camera class
     *
     *
     */
    struct CamConfig {
        std::string camID; // name of cam, eg: cam_0, cam_left
        int deviceSerial;
        std::string devicePath;
        int sensorWidth;
        int sensorHeight;
        int pixFmtENum = 1; // output format: {1:yuv420, 2:y16}
    };

    /**
     * @ imageWTs Struct to retrieve camera frame from V4l2Camera class
     *
     *
     */
    struct imageWTs {
        cv::Mat img;
        int64_t timeStampNS;
        std::string camID;
    };

    /** V4L2 FPIX FMT shorthand to pass to config
     * Not covering all supported v4l2 PIX FMTs
     * Currently supports YUV420, Y16.
     * PL. Feel free to add more FMTs and PR.
     */
    enum PIX_FMT {
        YVU420 = 1, Y16 = 2, YUYV = 5
    };

    /**
     * @ V4l2Camera cLass
     * inputs: CamConfig, EpochOffset, logFilePath(optional)
     *
     */
    class V4l2_Camera
    {
    public:
        V4l2_Camera(v4l2_ns::CamConfig config, std::string logFilePath,
                    long EpochOffsetMS = 0);

        ~V4l2_Camera()
        {
            deInitV4l2Camera();
        };

        bool initCamera()
        {
            initFlag_ = initV4l2Camera(); //initFlag_ used later for validation.
            return initFlag_;
        };

        void getNextFrame(imageWTs& imageWTsPtr)
        {
            return getFrame(imageWTsPtr);
        };

    private:
        CamConfig config_;
        std::string cameraInfoStr_;
        bool initFlag_;
        int32_t cameraFileDescriptor_;
        struct v4l2_capability v4l2_cap_;
        struct v4l2_requestbuffers buffer_request_;
        struct v4l2_buffer bufferInfo_;
        void *bufferStart_;
        cv::Mat imgFromBuffer_;
        long EpochOffsetMS_;
        std::string logFilePath_;

        bool initV4l2Camera();

        void deInitV4l2Camera();

        void getFrame(imageWTs& imageWTsPtr);

        long getEpochTimeShift();


    };  // V4l2Camera

} // V4l2_ns

#endif //V4L2_DRIVER_H
