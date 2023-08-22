//
// Created by mithun on 7/17/23.
//
#include "v4l2_driver.h"

v4l2_ns::V4l2_Camera::V4l2_Camera(v4l2_ns::CamConfig config,
                                  std::string logFilePath,
                                  long EpochOffsetMS) :
                                  config_(config), logFilePath_(logFilePath)
{
    cameraInfoStr_ = config_.camID + " " + std::to_string(config_.deviceSerial)
                        + " " + config_.devicePath;
    // Allows users to set consistent EpochOffsetMS_ across multi-cam
    // acquisition if they choose to. Otherwise
    EpochOffsetMS_ = (EpochOffsetMS != 0) ? EpochOffsetMS : getEpochTimeShift();
    //GLOG_log_dir = logFilePath_;
    google::InitGoogleLogging("v4l2_driver");
    LOG(INFO)<<"[V4L2_driver]:\tConstructor for v4l2 cam: " << cameraInfoStr_
    .c_str();
};

bool v4l2_ns::V4l2_Camera::initV4l2Camera()
{
    // io _NONBLOCK
    std::string device_path_ = config_.devicePath;
    if ((cameraFileDescriptor_ = open(device_path_.c_str(), O_RDWR )) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tFailed to open Cam: "<<cameraInfoStr_
        .c_str();
        return false;
    }
    else
    {
        LOG(INFO) <<"[V4L2_driver]:\tOpened Cam: "<< cameraInfoStr_.c_str();
    }

    // Check if VideoCapture mode is supported
    if (ioctl(cameraFileDescriptor_, VIDIOC_QUERYCAP, &v4l2_cap_) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tVIDIOC_QUERYCAP, Video Capture mode is "
                    "not supported for Cam: " << cameraInfoStr_.c_str();
        return false;
    }

    if (!(v4l2_cap_.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        LOG(ERROR) << "[V4L2_driver]:\tThe Cam: "<< cameraInfoStr_.c_str() << \
        " cannot handle single-planar video capture.";
        return false;
    }

    struct v4l2_format format; // check
    int height, width;
    int colorSpace;

    if (config_.pixFmtENum == PIX_FMT::YVU420)
    {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420;
        //   for luma, 8bits YCbCr mode in YUV
        height = 1.5 * config_.sensorHeight; //512+ 512/2
        width = config_.sensorWidth; // 640
        colorSpace = CV_8UC1;
    }
    else if (config_.pixFmtENum == PIX_FMT::Y16)
    {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
        height = config_.sensorHeight; //512
        width  = config_.sensorWidth; // 640
        colorSpace = CV_16U;
    }
    else if (config_.pixFmtENum == PIX_FMT::YUYV)
    {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        height = config_.sensorHeight;
        width  = config_.sensorWidth;
        colorSpace = CV_8UC4;
    }
    else
    {
        LOG(ERROR)<<"[V4L2_driver]: Unsupported format,  Check supported "
                    "types. If not feel free to PR";
        return false;
    }
    /* TODO: add support for following FMT
     * lEPTON Supports
     * GREY 8
     * UYVY 4x4 image as yvu420
     * Y16 16
     * RGBP 16
     * BGR3 24
     */

    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = config_.sensorWidth;
    format.fmt.pix.height = config_.sensorHeight;

    // request video Format
    if (ioctl(cameraFileDescriptor_, VIDIOC_S_FMT, &format) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:  VIDIOC_S_FMT, Cam: "<<\
        cameraInfoStr_.c_str() <<" doesn't support the requested video format";
        return false;
    }

    // buffer request
    buffer_request_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer_request_.memory = V4L2_MEMORY_MMAP;
    buffer_request_.count = 1;   // number of frames to buffer

    if (ioctl(cameraFileDescriptor_, VIDIOC_REQBUFS, &buffer_request_) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tVIDIOC_REQBUFS, Cam: "<< \
        cameraInfoStr_.c_str()<<" failed to allocate buffer";
        return false;
    }

    // allocate memory
    memset(&bufferInfo_, 0, sizeof(bufferInfo_));

    bufferInfo_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferInfo_.memory = V4L2_MEMORY_MMAP;
    bufferInfo_.index = 0;

    if (ioctl(cameraFileDescriptor_, VIDIOC_QUERYBUF, &bufferInfo_)< 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tVIDIOC_QUERYBUF, Failed to retreive" \
        "buffer information for Cam: "<<cameraInfoStr_.c_str();
        return false;
    }
    // map fd+offset into a process location (kernel will decide due
    // to our NULL), length and properties are also passed

    bufferStart_ = mmap(NULL,bufferInfo_.length,
                        PROT_READ | PROT_WRITE, MAP_SHARED,
                        cameraFileDescriptor_,
                        bufferInfo_.m.offset);


    if ( bufferStart_ == MAP_FAILED)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tmmap, For Cam: "<<cameraInfoStr_.c_str() \
        <<" Failed to create a memory map for buffer.";
        return false;
    }

    // Initialization: Fill the buffer with zeros. (Optional)
    memset(bufferStart_ , 0, bufferInfo_.length);

    // Activate streaming
    int type = bufferInfo_.type;
    if (ioctl(cameraFileDescriptor_, VIDIOC_STREAMON, &type) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tVIDIOC_STREAMON, Failed to activate" \
        "streaming for Cam: "<<cameraInfoStr_.c_str();
        return false;
    }

    imgFromBuffer_ = cv::Mat(height, width, colorSpace, bufferStart_);

    return true;
};

void v4l2_ns::V4l2_Camera::deInitV4l2Camera()
{
    // glog_DEBUG("[V4L2_Driver]:\tDestructor: %s,"cameraInfoStr_.c_str());

    if(initFlag_)
    {
        int type = bufferInfo_.type;
        if (ioctl(cameraFileDescriptor_, VIDIOC_STREAMOFF, &type) < 0)
        {
            LOG(ERROR)<<"[V4L2_Driver]:\tVIDIOC_STREAMOFF, Failed to disable "\
                        "streaming on Cam: "<< cameraInfoStr_.c_str();
        }
    }
    // release camera file descriptor
    close(cameraFileDescriptor_);

    // clear the memory alloted for buffer
    munmap(bufferStart_, bufferInfo_.length);
    //
    LOG(INFO)<<"[V4L2_Driver]:\tDestructor Success: "<<cameraInfoStr_.c_str();
};

void v4l2_ns::V4l2_Camera::getFrame(v4l2_ns::imageWTs& imageStruct)
{
    //ros::Time start_queue_t = ros::Time::now();
    if (ioctl(cameraFileDescriptor_, VIDIOC_QBUF, &bufferInfo_) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tVIDIOC_QBUF, Failed to queue the image" \
        "buffer for cam: "<< cameraInfoStr_.c_str();
        throw std::current_exception();
    }
    // ros::Time end_queue_t = ros::Time::now();

    // The buffer's waiting in the outgoing queue.
    if ( ioctl(cameraFileDescriptor_, VIDIOC_DQBUF, &bufferInfo_) < 0)
    {
        LOG(ERROR)<<"[V4L2_driver]:\tVIDIOC_DQBUF, Failed to dequeue the"\
        "image buffer for Cam: "<<cameraInfoStr_.c_str();
        throw std::current_exception();
    }
    // ros::Time end_de_queue_t = ros::Time::now();
    /** convert buffer timestamp to epoch timestamp
    source: https://stackoverflow.com/questions/10266451/where-does-v4l2-buffer-timestamp-value-starts-counting
    */
    uint64_t bufferTsNS  =  (bufferInfo_.timestamp.tv_sec * 1e9) + \
                                    (bufferInfo_.timestamp.tv_usec * 1e3);
    uint64_t imgTsNS = (uint64_t) (bufferTsNS + (EpochOffsetMS_ * 1e6));
    imageStruct.timeStampNS = (EpochOffsetMS_ * 1e6);
    // TODO: get ts from device instead of buffer.

    //LOG_EVERY_N(DEBUG,100)<<"[V4L2_driver]:\t queue "<< \
    (end_queue_t.toSec()-start_queue_t.toSec())*0.001<<" ms dequeue "<< \
    (end_de_queue_t.toSec() - end_queue_t.toSec())*0.001<<" ms");

    // OpenCV output buffer , BGR -> Three color spaces

    imageStruct.camID = config_.camID;

    if (config_.pixFmtENum == PIX_FMT::YVU420) // 1: agc8 mode
    {
        //   for luma
        cv::Mat imgRGB = cv::Mat(config_.sensorHeight, config_.sensorWidth, CV_8UC3, 1);
        cv::cvtColor(imgFromBuffer_, imgRGB, cv::COLOR_YUV2GRAY_I420,0);
        imageStruct.img = imgRGB.clone();
    }
    else if (config_.pixFmtENum == PIX_FMT::Y16) //2: RAW16 mode
    {
        imgFromBuffer_.convertTo(imageStruct.img, CV_16U);
    }
    else if (config_.pixFmtENum == PIX_FMT::YUYV)
    {
        cv::Mat imgRGB = cv::Mat(config_.sensorHeight, config_.sensorWidth,
                                  CV_8UC3, 0);
        cv::cvtColor(imgFromBuffer_, imgRGB, cv::COLOR_YUV2RGB_YUYV,0);
        imageStruct.img = imgRGB.clone();
        //imageStruct.img = imgFromBuffer_.clone();
    }
};

long v4l2_ns::V4l2_Camera::getEpochTimeShift()
{
        /** source: https://stackoverflow.com/questions/10266451/where-does-v4l2-buffer-timestamp-value-starts-counting

        \note Run this once during beginning of your capture process. \
        If you hibernate your laptop, you might need to recalculate this, \ unless you don't restart the capture process/node after dehibernation */

        struct timeval epochTime;
        struct timespec  vsTime;

        gettimeofday(&epochTime, NULL);
        clock_gettime(CLOCK_MONOTONIC, &vsTime);

        long uptimeMS = vsTime.tv_sec* 1000 + \
                            (long) round( vsTime.tv_nsec/ 1000000.0);
        long epochMS =  epochTime.tv_sec * 1000  + \
                            (long) round( epochTime.tv_usec/1000.0);
        long ret = epochMS - uptimeMS;
        // make sure epochTimeShift is non-zero
        return (ret !=0) ? ret: getEpochTimeShift();
}
